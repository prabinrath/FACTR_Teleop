# ---------------------------------------------------------------------------
# Copyright (c) 2025 Prabin Kumar Rath, Anant Sah

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
from factr_teleop.factr_teleop import FACTRTeleop
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from franka_msgs.msg import FrankaRobotState
from sensor_msgs.msg import JointState
import numpy as np
from builtin_interfaces.msg import Duration
from threading import Lock
from franka_msgs.action import Grasp
from rclpy.action import ActionClient


class FACTRTeleopFrankaROS(FACTRTeleop):
    """
    ROS2-based implementation of FACTRTeleopFranka communication.
    """

    def __init__(self):
        super().__init__()
        self.gripper_feedback_gain = self.config["controller"]["gripper_feedback"]["gain"]
        self.gripper_torque_ema_beta = self.config["controller"]["gripper_feedback"]["ema_beta"]
        self.gripper_external_torque = 0.0
        self.alpha = 0.95
        self._latest_franka_joint_state = None
        self.sync_flag = True
        self.js_mutex = Lock()

        self.is_grasped = False
        self.gripper_client = ActionClient(self, Grasp, '/franka_gripper/grasp')
        self.gripper_client.wait_for_server()

    def set_up_communication(self):
        self.franka_cmd_pub = self.create_publisher(JointTrajectory, '/fr3_arm_controller/joint_trajectory', 10)
        self.create_subscription(
            JointState,
            '/joint_states',
            self._franka_joint_state_callback,
            10
        )

        if self.enable_torque_feedback:
            raise NotImplementedError

        if self.enable_gripper_feedback:
            self.create_subscription(
                JointState, f'/gripper/{self.name}/obs_gripper_torque',
                self._gripper_external_torque_callback, 1
            )

    def _franka_joint_state_callback(self, msg):
        with self.js_mutex:
            self._latest_franka_joint_state = msg

    def _gripper_external_torque_callback(self, data):
        gripper_external_torque = data.position[0]
        self.gripper_external_torque = self.gripper_torque_ema_beta * self.gripper_external_torque + \
            (1 - self.gripper_torque_ema_beta) * gripper_external_torque

    def get_leader_gripper_feedback(self):
        return self.gripper_external_torque

    def gripper_feedback(self, leader_gripper_pos, leader_gripper_vel, gripper_feedback):
        torque_gripper = -1.0 * gripper_feedback / self.gripper_feedback_gain
        return torque_gripper
  
    def get_leader_arm_external_joint_torque(self):
        raise NotImplementedError

    def update_communication(self, leader_arm_pos, leader_gripper_pos):
        traj = JointTrajectory()
        traj.joint_names = [f'fr3_joint{i}' for i in range(1, 8)]

        if self._latest_franka_joint_state is None:
            return

        leader_pos = np.asarray(leader_arm_pos[:7])
        leader_pos[-1] -=0.77
        with self.js_mutex:
            follower_pos = np.asarray(self._latest_franka_joint_state.position[:7])

        # print(f"Leader: {leader_pos}")
        # print(f"Follower: {follower_pos}")

        filtered_pos = (1 - self.alpha) * follower_pos + self.alpha * leader_pos
        point = JointTrajectoryPoint()
        distance = np.linalg.norm(follower_pos-leader_pos)
        # print(distance)
        if distance > 0.1 and self.sync_flag:
            dt = max(self.dt * 40 , distance)
            t = Duration(sec=0, nanosec=int(dt * 1e9))
            point.positions = leader_pos.tolist()
        else:
            self.sync_flag = False
            dt = self.dt * 40
            t = Duration(sec=0, nanosec=int(dt * 1e9))
            point.positions = filtered_pos.tolist()

        point.time_from_start = t
        traj.points = [point]
        self.franka_cmd_pub.publish(traj)

        # print(leader_gripper_pos)
        if leader_gripper_pos > 0.05 and self.is_grasped:
            grasp_goal = Grasp.Goal()
            grasp_goal.width = 0.08
            grasp_goal.epsilon.inner = 0.08
            grasp_goal.epsilon.outer = 0.08
            grasp_goal.speed = 0.1
            grasp_goal.force = 5.0
            self.gripper_client.send_goal_async(grasp_goal)
            self.is_grasped = False
            print("grasped")
        if leader_gripper_pos < 0.05 and not self.is_grasped:
            grasp_goal = Grasp.Goal()
            grasp_goal.width = 0.0
            grasp_goal.epsilon.inner = 0.08
            grasp_goal.epsilon.outer = 0.08
            grasp_goal.speed = 0.1
            grasp_goal.force = 5.0
            self.gripper_client.send_goal_async(grasp_goal)  
            self.is_grasped = True
            print("ungrasped")
        

#  echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
def main(args=None):
    rclpy.init(args=args)
    factr_teleop_franka_ros = FACTRTeleopFrankaROS()
    try:
        while rclpy.ok():
            rclpy.spin(factr_teleop_franka_ros)
    except KeyboardInterrupt:
        factr_teleop_franka_ros.get_logger().info("Keyboard interrupt received. Shutting down...")
        factr_teleop_franka_ros.shut_down()
    finally:
        rclpy.shutdown()
