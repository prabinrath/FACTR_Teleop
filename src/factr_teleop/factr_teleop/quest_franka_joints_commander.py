#!/usr/bin/env python3

# ---------------------------------------------------------------------------
# Copyright (c) 2025 Prabin Kumar Rath

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ---------------------------------------------------------------------------

import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy, JointState
from franka_msgs.action import Grasp
from scipy.spatial.transform import Rotation as R
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from ament_index_python.packages import get_package_share_directory
import placo
import os
import time


class QuestFrankaControl(Node):
    def __init__(self):
        super().__init__('quest_franka_control')

        # Params (tweak as needed)
        self.declare_parameter('local_frame', False)
        self.local_frame = self.get_parameter('local_frame').value  # True: q * dq ; False: dq * q

        # Initialize Placo for IK
        package_share_dir = get_package_share_directory('factr_teleop')
        urdf_path = os.path.join(package_share_dir, 'urdf', 'franka_fr3_kinematics.urdf')
        self.robot = placo.RobotWrapper(urdf_path, placo.Flags.collision_as_visual)
        self.solver = self.robot.make_solver()
        
        # Mask the floating base (robot is fixed)
        self.solver.mask_fbase(True)
        
        # Enable joint limits
        self.solver.enable_joint_limits(True)
        
        # Set dt for velocity control (matching joy callback frequency)
        self.solver.dt = 1.0 / 30
        
        # Joint names for FR3
        self.joint_names = [
            'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
            'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
        ]
        
        # Initialize robot to a neutral position
        neutral_positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        for joint_name, position in zip(self.joint_names, neutral_positions):
            self.robot.set_joint(joint_name, position)
        self.robot.update_kinematics()
        
        # Create frame task for end-effector control
        self.frame_task = self.solver.add_frame_task('fr3_hand_tcp', np.eye(4))
        self.frame_task.configure('fr3_hand_tcp', 'soft', 0.5, 0.5)

        # Create joint task for nullspace regularization
        self.joints_task = self.solver.add_joints_task()
        self.joints_task.set_joints({joint_name: position for joint_name, position in zip(self.joint_names, neutral_positions)})
        self.joints_task.configure("nullspace", "soft", 1e-5)
        
        # Add regularization to keep solution smooth
        self.regularization = self.solver.add_regularization_task(1e-6)
        
        self.pub = self.create_publisher(
            JointTrajectory,
            '/fr3_arm_controller/joint_trajectory',
            10
        )

        self.origin_pose_tuple = None
        self.latest_quest_pose_msg = None
        self.tare_quest_pose_tuple = None
        self.latest_joint_state = None
        self.alpha = 1.0
        self.last_solve_time = None
        
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_cb,
            1
        )
        self.create_subscription(
            PoseStamped,
            '/right_hand_pose',
            self.quest_pose_cb,
            1
        )
        self.create_subscription(
            Joy,
            '/right_hand_inputs',
            self.quest_button_cb,
            1
        )
        
        self.is_grasped = False
        self.grasp_toggle = False
        self.gripper_client = ActionClient(self, Grasp, '/franka_gripper/grasp')
        self.gripper_client.wait_for_server()

        self.get_logger().info("Quest → Franka joint trajectory teleop ready.")
    
    def joint_state_cb(self, msg: JointState):
        self.latest_joint_state = msg
    
    def quest_pose_cb(self, msg: PoseStamped):
        self.latest_quest_pose_msg = msg
        
    def quest_button_cb(self, msg: Joy):
        if msg.axes[-1] > 0:
            if self.origin_pose_tuple is None:
                # Compute origin pose once using FK from current joint state
                if self.latest_joint_state is not None:
                    for joint_name in self.joint_names:
                        position = self.latest_joint_state.position[self.latest_joint_state.name.index(joint_name)]
                        self.robot.set_joint(joint_name, position)
                    self.robot.update_kinematics()
                    T_world_ee = self.robot.get_T_world_frame('fr3_hand_tcp')
                    p_org = T_world_ee[0:3, 3]
                    R_org = T_world_ee[0:3, 0:3]
                    self.origin_pose_tuple = (p_org, R_org)
            if self.tare_quest_pose_tuple is None:
                if self.latest_quest_pose_msg is not None:
                    p_tare = np.array([
                        self.latest_quest_pose_msg.pose.position.x,
                        self.latest_quest_pose_msg.pose.position.y,
                        self.latest_quest_pose_msg.pose.position.z
                    ])
                    q_tare = np.array([
                        self.latest_quest_pose_msg.pose.orientation.x,
                        self.latest_quest_pose_msg.pose.orientation.y,
                        self.latest_quest_pose_msg.pose.orientation.z,
                        self.latest_quest_pose_msg.pose.orientation.w
                    ])
                    R_tare = R.from_quat(q_tare).as_matrix()
                    self.tare_quest_pose_tuple = (p_tare, R_tare)
            
            # ---- Use stored origin pose and tare pose ----
            if self.origin_pose_tuple is None or self.tare_quest_pose_tuple is None:
                return
                
            p_org, R_org = self.origin_pose_tuple
            p_tare, R_tare = self.tare_quest_pose_tuple
                
            # add latest_quest_pose to origin_state and get p_new and q_new
            if self.latest_quest_pose_msg is None:
                return

            # ---- Compute relative Quest pose wrt tare ----

            # Current Quest pose
            p_curr = np.array([
                self.latest_quest_pose_msg.pose.position.x,
                self.latest_quest_pose_msg.pose.position.y,
                self.latest_quest_pose_msg.pose.position.z
            ])
            q_curr = np.array([
                self.latest_quest_pose_msg.pose.orientation.x,
                self.latest_quest_pose_msg.pose.orientation.y,
                self.latest_quest_pose_msg.pose.orientation.z,
                self.latest_quest_pose_msg.pose.orientation.w
            ])

            # Relative rotation (R_rel = R_tare^T * R_curr)
            R_curr = R.from_quat(q_curr).as_matrix()
            R_rel = R_tare.T @ R_curr

            # Relative translation (expressed in tare frame)
            p_rel = R_tare.T @ (p_curr - p_tare)

            # Apply relative motion to robot origin pose
            if self.local_frame:
                R_new = R_org @ R_rel
                # Transform relative translation to world frame using origin rotation
                p_new = p_org + R_org @ (p_rel * 2.0)
            else:
                R_new = R_rel @ R_org
                # Transform relative translation to world frame directly
                p_new = p_org + (p_rel * 2.0)
            
            # ---- Use Placo IK to compute joint positions ----
            # Update robot state with current joint positions
            if self.latest_joint_state is not None:
                for joint_name in self.joint_names:
                    position = self.latest_joint_state.position[self.latest_joint_state.name.index(joint_name)]
                    self.robot.set_joint(joint_name, position)
            
            # Create target transformation matrix
            T_target = np.eye(4)
            T_target[0:3, 3] = p_new
            T_target[0:3, 0:3] = R_new
            
            # Update the frame task target
            self.frame_task.T_world_frame = T_target
            
            # Update dt based on actual time difference between solve calls
            current_time = time.perf_counter()
            if self.last_solve_time is not None:
                dt = current_time - self.last_solve_time
                self.solver.dt = dt
            self.last_solve_time = current_time
            
            # Solve IK
            self.robot.update_kinematics()
            self.solver.solve(True)  # Apply solution to robot state
            
            # Get joint positions from solved configuration
            ik_joint_positions = []
            for joint_name in self.joint_names:
                ik_joint_positions.append(self.robot.get_joint(joint_name))
            
            # Apply smoothing between current and IK joint positions
            current_joint_positions = [
                self.latest_joint_state.position[self.latest_joint_state.name.index(joint_name)]
                for joint_name in self.joint_names
            ]
            filtered_joint_positions = [
                self.alpha * ikjp + (1 - self.alpha) * cjp
                for cjp, ikjp in zip(current_joint_positions, ik_joint_positions)
            ]
            
            # ---- Publish JointTrajectory ----
            traj_msg = JointTrajectory()
            traj_msg.header.stamp = self.get_clock().now().to_msg()
            traj_msg.header.frame_id = "base"
            traj_msg.joint_names = self.joint_names
            
            # Create trajectory point
            point = JointTrajectoryPoint()
            point.positions = filtered_joint_positions
            point.time_from_start = Duration(sec=0, nanosec=int(0.1 * 1e9))
            
            traj_msg.points = [point]
            self.pub.publish(traj_msg)  
        else:
            self.origin_pose_tuple = None
            self.tare_quest_pose_tuple = None
            
        if msg.axes[-2] > 0 and not self.grasp_toggle:
            if self.is_grasped:
                grasp_goal = Grasp.Goal()
                grasp_goal.width = 0.08
                grasp_goal.epsilon.inner = 0.08
                grasp_goal.epsilon.outer = 0.08
                grasp_goal.speed = 0.1
                grasp_goal.force = 5.0
                self.gripper_client.send_goal_async(grasp_goal)
                self.is_grasped = False
            else:
                grasp_goal = Grasp.Goal()
                grasp_goal.width = 0.0
                grasp_goal.epsilon.inner = 0.08
                grasp_goal.epsilon.outer = 0.08
                grasp_goal.speed = 0.1
                grasp_goal.force = 5.0
                self.gripper_client.send_goal_async(grasp_goal)
                self.is_grasped = True
            self.grasp_toggle = True
        if not msg.axes[-2]:
            self.grasp_toggle = False        

    def run(self):
        rclpy.spin(self)


def main(args=None):
    rclpy.init(args=args)
    quest_franka_control = QuestFrankaControl()
    try:
        quest_franka_control.run()
    except KeyboardInterrupt:
        quest_franka_control.get_logger().info("Keyboard interrupt received. Shutting down...")
    finally:
        quest_franka_control.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
