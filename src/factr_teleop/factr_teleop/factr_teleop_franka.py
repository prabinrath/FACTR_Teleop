import numpy as np
import time
import pinocchio as pin
import os
import subprocess
import yaml
from abc import ABC, abstractmethod

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from factr_teleop.dynamixel.driver import DynamixelDriver
from python_utils.zmq_messenger import ZMQPublisher, ZMQSubscriber
from python_utils.utils import get_workspace_root
from python_utils.global_configs import franka_left_real_zmq_addresses, franka_left_sim_zmq_addresses, left_GELLO_gripper_configs
from python_utils.global_configs import franka_right_real_zmq_addresses, franka_right_sim_zmq_addresses, right_GELLO_gripper_configs


class FACTRTeleopFranka(Node, ABC):
    def __init__(self):
        super().__init__('factr_teleop_franka')
        # self.torque_feedback = self.declare_parameter('torque_feedback', False).get_parameter_value().bool_value
        # self.gravity_comp = self.declare_parameter('gravity_comp', False).get_parameter_value().bool_value
        self.is_left = self.declare_parameter('is_left', False).get_parameter_value().bool_value

        config_file_name = self.declare_parameter('config_file', '').get_parameter_value().string_value
        config_path = os.path.join(get_workspace_root(), f"src/factr_teleop/factr_teleop/configs/{config_file_name}")
        with open(config_path, 'r') as config_file:
            self.config = yaml.safe_load(config_file)
        
        self.name = self.config["name"]


        if self.is_left:
            zmq_addresses = franka_left_real_zmq_addresses
        else:
            zmq_addresses = franka_right_real_zmq_addresses
        self.joint_pos_cmd_pub_address = zmq_addresses["joint_pos_cmd_pub"]
        self.joint_torque_sub_address = zmq_addresses["joint_torque_sub"]
        self.joint_pos_sub_address = zmq_addresses["joint_state_sub"]
          

        self.joint_signs = np.array(self.config["dynamixel"]["joint_signs"], dtype=float)
        self.dynamixel_port = self.config["dynamixel"]["dynamixel_port"]

        # checks of the latency timer on ttyUSB of the corresponding port is 1
        # if it is not 1, the control loop cannot run at 200 Hz, which will cause
        # extremely undesirable behaviour to the GELLO. If the latency timer is not
        # 1, one can set it to 1 as follows:
        # echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB{NUM}/latency_timer
        ttyUSBx = self._find_ttyusb(self.dynamixel_port)
        command = f"cat /sys/bus/usb-serial/devices/{ttyUSBx}/latency_timer"        
        result = subprocess.run(command, shell=True, capture_output=True, text=True, check=True)
        ttyUSB_latency_timer = int(result.stdout)
        if ttyUSB_latency_timer != 1:
            raise Exception(
                f"Please ensure the latency timer of {ttyUSBx} is 1. Run: \n \
                echo 1 | sudo tee /sys/bus/usb-serial/devices/{ttyUSBx}/latency_timer"
            )
        

        self.dt = 1/500
        self.num_motors = 8
        self.num_arm_joints = 7
        self.init_joint_pos = np.array([0.0, -0.7854, 0.0, -2.356, 0.0, 1.57, 0.0, 0.0])
        self.arm_joint_limits_max = np.array([2.8973, 1.7628, 2.8973, -0.0698-0.8, 2.8973, 3.7525, 2.8973]) - 0.1
        self.arm_joint_limits_min = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]) + 0.1


        self.gripper_limit_min = 0.0
        self.gripper_limit_max = self.config["gripper_teleop"]["actuation_range"]

        self.joint_pos = np.zeros(self.num_arm_joints)
        self.joint_vel = np.zeros(self.num_arm_joints)
        self.external_torque = np.zeros(self.num_arm_joints)

        self.gripper_pos_prev = 0.0
        self.gripper_pos = 0.0
        self.gripper_vel = 0.0
        self.gripper_external_torque = 0.0

        self._prepare_messengers()
        self._prepare_inverse_dynamics()

        
        # gravity comp
        self.enable_gravity_comp = self.config["controller"]["gravity_comp"]["enable"]
        self.gravity_comp_modifier = self.config["controller"]["gravity_comp"]["gain"]
        self.tau_g = np.zeros(self.num_arm_joints)

        # friction comp
        self.stiction_comp_enable_speed = self.config["controller"]["static_friction_comp"]["enable_speed"]
        self.stiction_comp_gain = self.config["controller"]["static_friction_comp"]["gain"]
        self.stiction_dither_flag = np.ones((self.num_arm_joints), dtype=bool)

        # joint limit barrier:
        self.joint_limit_kp = self.config["controller"]["joint_limit_barrier"]["kp"]
        self.joint_limit_kd = self.config["controller"]["joint_limit_barrier"]["kd"]

        # null space regulation
        self.null_space_joint_target = np.array(self.config["controller"]["null_space_regulation"]["null_space_joint_target"])
        self.null_space_kp = self.config["controller"]["null_space_regulation"]["kp"]
        self.null_space_kd = self.config["controller"]["null_space_regulation"]["kd"]

        # torque feedback
        self.enable_torque_feedback = self.config["controller"]["torque_feedback"]["enable"]
        self.torque_feedback_gain = self.config["controller"]["torque_feedback"]["gain"]
        self.torque_feedback_motor_scalar = self.config["controller"]["torque_feedback"]["motor_scalar"]

        # gripper feedback
        self.enable_gripper_feedback = self.config["controller"]["gripper_feedback"]["enable"]
        self.gripper_feedback_gain = self.config["controller"]["gripper_feedback"]["gain"]


        servo_types = [
            "XC330_T288_T", "XM430_W210_T", "XC330_T288_T", "XM430_W210_T", 
            "XC330_T288_T", "XC330_T288_T", "XC330_T288_T", "XC330_T288_T",
        ]

        try:
            self.driver = DynamixelDriver(
                self.config["dynamixel"]["joint_ids"], 
                servo_types, self.dynamixel_port
            )
        except FileNotFoundError:
            print(f"Port {self.dynamixel_port} not found. Please check the connection.")
            return
    
        self.driver.set_torque_mode(False)
        self.driver.set_operating_mode(0)
        self.driver.set_torque_mode(True)

        self.get_offsets()
        self.match_start_pos()
        self.timer = self.create_timer(self.dt, self.timer_callback)


    def _find_ttyusb(self, port_name):
        base_path = "/dev/serial/by-id/"
        full_path = os.path.join(base_path, port_name)
        if not os.path.exists(full_path):
            raise Exception(f"Port '{port_name}' does not exist in {base_path}.")
        try:
            resolved_path = os.readlink(full_path)
            actual_device = os.path.basename(resolved_path)
            if actual_device.startswith("ttyUSB"):
                return actual_device
            else:
                raise Exception(f"The port '{port_name}' does not correspond to a ttyUSB device. It links to {resolved_path}.")
        except Exception as e:
            raise Exception(f"Unable to resolve the symbolic link for '{port_name}'. {e}")


    def _prepare_messengers(self):
        self.franka_GELLO_pub = ZMQPublisher(self.joint_pos_cmd_pub_address)
        self.franka_pos_sub = ZMQSubscriber(self.joint_pos_sub_address)
        # ros publisher for franka joint angles observations
        self.obs_franka_pos_pub = self.create_publisher(JointState, f'/franka/{self.name}/obs_franka_pos', 10)
        # ros publisher for franka and gripper commands
        self.cmd_franka_pos_pub = self.create_publisher(JointState, f'/gello/{self.name}/cmd_franka_pos', 10)
        self.cmd_gripper_pos_pub = self.create_publisher(JointState, f'/gello/{self.name}/cmd_gripper_pos', 10)

        if self.torque_feedback:
            self.franka_torque_sub = ZMQSubscriber(self.joint_torque_sub_address)
            while self.franka_torque_sub.message is None:
                print(f"Has not received Franka {self.joint_torque_sub_address}'s torques")
            # ros publisher for franka torque observations
            self.obs_franka_torque_pub = self.create_publisher(JointState, f'/franka/{self.name}/obs_franka_torque', 10)
            
            self.obs_gripper_torque_pub = self.create_subscription(
                JointState, f'/gripper/{self.name}/obs_gripper_torque', 
                self._gripper_external_torque_callback, 
                1,
            )
        

    def _prepare_inverse_dynamics(self):
        workspace_root = get_workspace_root()
        urdf_model_path = os.path.join(
            workspace_root, 'src/factr_teleop/factr_teleop/urdf/factr_teleop_franka.urdf'
        )
        self.pin_model, _, _ = pin.buildModelsFromUrdf(
            filename=urdf_model_path, 
            package_dirs=os.path.join(workspace_root, "src/factr_teleop/factr_teleop/urdf")
        )
        self.pin_data = self.pin_model.createData()



    def _get_error(self, offset, index, joint_state):
        joint_sign_i = self.joint_signs[index]
        joint_i = joint_sign_i * (joint_state[index] - offset)
        start_i = self.init_joint_pos[index]
        return np.abs(joint_i - start_i)
        

    def get_offsets(self, verbose=True):
        # warm up
        for _ in range(10):
            self.driver.get_positions_and_velocities()

        # get arm offsets
        self.joint_offsets = []
        curr_joints, _ = self.driver.get_positions_and_velocities()
        for i in range(self.num_arm_joints):
            best_offset = 0
            best_error = 1e9
            # intervals of pi/2
            for offset in np.linspace(-20 * np.pi, 20 * np.pi, 20 * 4 + 1):  
                error = self._get_error(offset, i, curr_joints)
                if error < best_error:
                    best_error = error
                    best_offset = offset
            self.joint_offsets.append(best_offset)

        # get gripper offset:
        curr_gripper_joint = curr_joints[-1]
        self.joint_offsets.append(curr_gripper_joint)

        self.joint_offsets = np.asarray(self.joint_offsets)
        if verbose:
            print(self.joint_offsets)
            print("best offsets               : ", [f"{x:.3f}" for x in self.joint_offsets])
            print(
                "best offsets function of pi: ["
                + ", ".join([f"{int(np.round(x/(np.pi/2)))}*np.pi/2" for x in self.joint_offsets])
                + " ]",
            )
    


    def create_array_msg(self, data):
        msg = JointState()
        msg.position = list(map(float, data))
        return msg
        

    def get_joint_states(self):
        self.gripper_pos_prev = self.gripper_pos

        joint_pos_gripper, joint_vel_gripper = self.driver.get_positions_and_velocities()
        self.joint_pos = (
            joint_pos_gripper[0:self.num_arm_joints] - self.joint_offsets[0:self.num_arm_joints]
        ) * self.joint_signs[0:self.num_arm_joints]
        self.gripper_pos = (joint_pos_gripper[-1] - self.joint_offsets[-1]) * self.joint_signs[-1]
        self.joint_vel = joint_vel_gripper[0:self.num_arm_joints] * self.joint_signs[0:self.num_arm_joints]
        
        self.gripper_vel = (self.gripper_pos - self.gripper_pos_prev) / self.dt
        return self.joint_pos, self.joint_vel, self.gripper_pos, self.gripper_vel
    

    def set_joint_pos(self, goal_joint_pos, goal_gripper_pos):
        interpolation_step_size = np.ones(7)*self.config["controller"]["interpolation_step_size"]
        kp = self.config["controller"]["joint_position_control"]["kp"]
        kd = self.config["controller"]["joint_position_control"]["kd"]

        curr_pos, curr_vel, curr_gripper_pos, curr_gripper_vel = self.get_joint_states()
        while (np.linalg.norm(curr_pos - goal_joint_pos) > 0.1):
            next_joint_pos_target = np.where(
                np.abs(curr_pos - goal_joint_pos) > interpolation_step_size, 
                curr_pos + interpolation_step_size*np.sign(goal_joint_pos-curr_pos),
                goal_joint_pos,
            )
            torque = -kp*(curr_pos-next_joint_pos_target)-kd*(curr_vel)
            gripper_torque = -kp*(curr_gripper_pos-goal_gripper_pos)-kd*(curr_gripper_vel)
            self.set_joint_torque(torque, gripper_torque)
            curr_pos, curr_vel, curr_gripper_pos, curr_gripper_vel = self.get_joint_states()
    

    def match_start_pos(self):
        curr_pos, _, _, _ = self.get_joint_states()
        while (np.linalg.norm(curr_pos - self.init_joint_pos[0:self.num_arm_joints]) > 0.5):
            current_joint_error = np.linalg.norm(curr_pos - self.init_joint_pos[0:self.num_arm_joints])
            self.get_logger().info(
                f"FACTR TELEOP {self.name}: Please match starting joint pos. Current error: {current_joint_error}"
            )
            curr_pos, _, _, _ = self.get_joint_states()
            time.sleep(0.5)
        self.get_logger().info(f"FACTR TELEOP {self.name}: Initial joint position matched.")

    
    def shut_down(self):
        self.set_joint_torque(np.zeros(self.num_arm_joints), 0.0)
        self.driver.set_torque_mode(False)
        
        
    def set_joint_torque(self, torque, gripper_torque):
        full_torque = np.append(torque, gripper_torque)
        self.driver.set_torque(full_torque*self.joint_signs)


    def joint_limit_barrier(self):
        exceed_max_mask = self.joint_pos > self.arm_joint_limits_max
        tau_l = (-self.joint_limit_kp * (self.joint_pos - self.arm_joint_limits_max) - self.joint_limit_kd * self.joint_vel) * exceed_max_mask
        exceed_min_mask = self.joint_pos < self.arm_joint_limits_min
        tau_l += (-self.joint_limit_kp * (self.joint_pos - self.arm_joint_limits_min) - self.joint_limit_kd * self.joint_vel) * exceed_min_mask

        if self.gripper_pos > self.gripper_limit_max:
            tau_l_gripper = -self.joint_limit_kp * (self.gripper_pos - self.gripper_limit_max) - self.joint_limit_kd * self.gripper_vel
        elif self.gripper_pos < self.gripper_limit_min:
            tau_l_gripper = -self.joint_limit_kp * (self.gripper_pos - self.gripper_limit_min) - self.joint_limit_kd * self.gripper_vel
        else:
            tau_l_gripper = 0.0
        return tau_l, tau_l_gripper


    def gravity_compensation(self):
        self.tau_g = pin.rnea(
            self.pin_model, self.pin_data, 
            self.joint_pos, self.joint_vel, np.zeros_like(self.joint_vel)
        )
        self.tau_g *= self.gravity_comp_modifier 
        return self.tau_g


    def friction_compensation(self):
        # static friction compensation
        tau_ss = np.zeros(self.num_arm_joints)
        for i in range(self.num_arm_joints):
            if abs(self.joint_vel[i]) < self.stiction_comp_enable_speed:
                if self.stiction_dither_flag[i]:
                    tau_ss[i] += self.stiction_comp_gain * abs(self.tau_g[i])
                else:
                    tau_ss[i] -= self.stiction_comp_gain * abs(self.tau_g[i])
                self.stiction_dither_flag[i] = ~self.stiction_dither_flag[i]
        return tau_ss
    

    def null_space_regulation(self):
        J = pin.computeJointJacobian(self.pin_model, self.pin_data, self.joint_pos, 7)
        J_dagger = np.linalg.pinv(J)
        null_space_projector = np.eye(self.num_arm_joints) - J_dagger @ J
        q_error = self.joint_pos - self.null_space_joint_target[0:self.num_arm_joints]
        tau_n = null_space_projector @ (-self.null_space_kp*q_error -self.null_space_kd*self.joint_vel)
        return tau_n
    

    def torque_feedback(self):
        self.external_torque = self.franka_torque_sub.message
        self.obs_franka_torque_pub.publish(self.create_array_msg(self.external_torque))

        self.external_torque = np.where(
            np.abs(self.external_torque)>0.0, 
            self.torque_feedback_gain*self.external_torque, 
            0.0
        ) 
        self.external_torque = self.external_torque/self.torque_feedback_motor_scalar
        return -1.0* self.external_torque


    def gripper_feedback(self):
        torque_gripper = -1.0*self.gripper_external_torque / self.gripper_feedback_gain
        return torque_gripper

    
    def compute_combined_torque(self):
        torque_arm = np.zeros(self.num_arm_joints)
        torque_l, torque_gripper = self.joint_limit_barrier()
        torque_arm += torque_l
        torque_arm += self.null_space_regulation()

        if self.enable_gravity_comp:
            torque_arm += self.gravity_compensation()
            torque_arm += self.friction_compensation()
        
        if self.enable_torque_feedback:
            torque_arm += self.torque_feedback()
        
        if self.enable_gripper_feedback:
            torque_gripper += self.gripper_feedback()

        return torque_arm, torque_gripper
    
    
    def _gripper_external_torque_callback(self, data):
        beta = 0.95
        gripper_external_torque = data.position[0]
        self.gripper_external_torque = beta * self.gripper_external_torque + (1-beta) * gripper_external_torque


    def timer_callback(self):
        positions, _, gripper_pos, _ = self.get_joint_states()

        # send gripper position
        self.cmd_gripper_pos_pub.publish(self.create_array_msg([gripper_pos]))

        # obs franka pos
        franka_pos = self.franka_pos_sub.message
        self.obs_franka_pos_pub.publish(self.create_array_msg(franka_pos))

        # obs franka torque, set gello torque
        torque_arm, torque_gripper = self.compute_combined_torque()
        self.set_joint_torque(torque_arm, torque_gripper)
        
        # send franka position
        self.franka_GELLO_pub.send_message(positions)
        self.cmd_franka_pos_pub.publish(self.create_array_msg(positions))
    

    # @abstractmethod
    # def get_leader_arm_joint_state(self):
    #     pass
    

    # @abstractmethod
    # def get_leader_arm_external_joint_torque(self):
    #     pass

    # @abstractmethod
    # def send_leader_arm_joint_position_target(self, joint_position_target):
    #     pass
    

    # @abstractmethod
    # def get_leader_gripper_state(self):
    #     pass

    # @abstractmethod
    # def send_leader_gripper_command(self, gripper_command):
    #     pass

    

    
        

    

def main(args=None):
    rclpy.init(args=args)
    factr_teleop_franka = FACTRTeleopFranka()

    try:
        while rclpy.ok():
            rclpy.spin(factr_teleop_franka)
    except KeyboardInterrupt:
        factr_teleop_franka.get_logger().info("Keyboard interrupt received. Shutting down...")
        factr_teleop_franka.shut_down()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

