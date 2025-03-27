import numpy as np
import time
import pinocchio as pin
import os
import subprocess

import rclpy
from rclpy.node import Node

from factr_teleop.dynamixel.driver import DynamixelDriver
from python_utils.zmq_messenger import ZMQPublisher, ZMQSubscriber
from python_utils.utils import get_workspace_root
from python_utils.global_configs import franka_left_real_zmq_addresses, franka_left_sim_zmq_addresses, left_GELLO_gripper_configs
from python_utils.global_configs import franka_right_real_zmq_addresses, franka_right_sim_zmq_addresses, right_GELLO_gripper_configs
from sensor_msgs.msg import JointState

class FACTRTeleopFranka(Node):
    
    def create_array_msg(self, data):
        msg = JointState()
        msg.position = list(map(float, data))
        return msg
    
    def __init__(self):
        super().__init__('factr_teleop_franka')
        self.connect_to_real = self.declare_parameter('connect_to_real', False).get_parameter_value().bool_value
        self.torque_feedback = self.declare_parameter('torque_feedback', False).get_parameter_value().bool_value
        self.gravity_comp = self.declare_parameter('gravity_comp', False).get_parameter_value().bool_value
        self.teleop_device_attached = self.declare_parameter('teleop_device_attached', False).get_parameter_value().bool_value
        self.is_left = self.declare_parameter('is_left', False).get_parameter_value().bool_value

        if self.is_left:
            self.gello_name = "left"
            gello_configs = left_GELLO_gripper_configs
            if self.connect_to_real:
                zmq_addresses = franka_left_real_zmq_addresses
            else:
                zmq_addresses = franka_left_sim_zmq_addresses
        else:
            self.gello_name = "right"
            gello_configs = right_GELLO_gripper_configs
            if self.connect_to_real:
                zmq_addresses = franka_right_real_zmq_addresses
            else:
                zmq_addresses = franka_right_sim_zmq_addresses

        self.joint_signs = np.array(gello_configs["joint_signs"], dtype=float)
        self.dynamixel_port = gello_configs["dynamixel_port"]
        self.joint_pos_cmd_pub_address = zmq_addresses["joint_pos_cmd_pub"]
        self.joint_torque_sub_address = zmq_addresses["joint_torque_sub"]
        self.joint_pos_sub_address = zmq_addresses["joint_state_sub"]

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
        self.joint_pos_neutral = np.array([0.0, -0.7854, 0.0, -2.356, 0.0, 1.57, 0.0, 0.0])
        self.joint_limits_max = np.array([2.8973, 1.7628, 2.8973, -0.0698-0.8, 2.8973, 3.7525, 2.8973]) - 0.1
        self.joint_limits_min = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]) + 0.1
        self.dither_flag = np.ones((self.num_arm_joints), dtype=bool)

        self.gripper_limit_min = gello_configs["joint_limits"][0]
        self.gripper_limit_max = gello_configs["joint_limits"][1]

        self.joint_pos = np.zeros(self.num_arm_joints)
        self.joint_pos_prev = np.zeros(self.num_arm_joints)
        self.joint_vel = np.zeros(self.num_arm_joints)
        self.external_torque = np.zeros(self.num_arm_joints)
        self.external_torque_sensed_prev = np.zeros(self.num_arm_joints)
        self.external_torque_sensed = np.zeros(self.num_arm_joints)
        self.torque_arm = np.zeros(self.num_arm_joints)

        self.gripper_pos_prev = 0.0
        self.gripper_pos = 0.0
        self.gripper_vel = 0.0
        self.gripper_external_torque = 0.0

        self._prepare_messengers()
        self._prepare_inverse_dynamics()

        servo_types = [
            "XC330_T288_T", "XM430_W210_T", "XC330_T288_T", "XM430_W210_T", 
            "XC330_T288_T", "XC330_T288_T", "XC330_T288_T", "XC330_T288_T",
        ]

        try:
            self.driver = DynamixelDriver(gello_configs["joint_ids"], servo_types, self.dynamixel_port)
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
        self.obs_franka_pos_pub = self.create_publisher(JointState, f'/franka/{self.gello_name}/obs_franka_pos', 10)
        # ros publisher for franka and gripper commands
        self.cmd_franka_pos_pub = self.create_publisher(JointState, f'/gello/{self.gello_name}/cmd_franka_pos', 10)
        self.cmd_gripper_pos_pub = self.create_publisher(JointState, f'/gello/{self.gello_name}/cmd_gripper_pos', 10)

        if self.torque_feedback:
            self.franka_torque_sub = ZMQSubscriber(self.joint_torque_sub_address)
            while self.franka_torque_sub.message is None:
                print(f"Has not received Franka {self.joint_torque_sub_address}'s torques")
            # ros publisher for franka torque observations
            self.obs_franka_torque_pub = self.create_publisher(JointState, f'/franka/{self.gello_name}/obs_franka_torque', 10)
            
            self.obs_gripper_torque_pub = self.create_subscription(
                JointState, f'/gripper/{self.gello_name}/obs_gripper_torque', 
                self._gripper_external_torque_callback, 
                1,
            )
        
    def _prepare_inverse_dynamics(self):
        workspace_root = get_workspace_root()
        if self.teleop_device_attached:
            urdf_model_path = os.path.join(
                workspace_root, 'src', 'factr_teleop', 'factr_teleop', 'urdf', 'factr_teleop_franka.urdf'
            )
        else:
            urdf_model_path = os.path.join(
                workspace_root, 'src', 'factr_teleop', 'factr_teleop', 'urdf', 'factr_teleop_franka.urdf'
            )
        self.pin_model, _, _ = pin.buildModelsFromUrdf(
            filename=urdf_model_path, 
            package_dirs=os.path.join(workspace_root, "src/factr_teleop/factr_teleop/urdf")
        )
        self.pin_data = self.pin_model.createData()


    def _get_error(self, offset: float, index: int, joint_state: np.ndarray) -> float:
        joint_sign_i = self.joint_signs[index]
        joint_i = joint_sign_i * (joint_state[index] - offset)
        start_i = self.joint_pos_neutral[index]
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
        


    def get_joint_states(self):
        self.joint_pos_prev = self.joint_pos.copy()
        self.gripper_pos_prev = self.gripper_pos

        joint_pos_gripper, joint_vel_gripper = self.driver.get_positions_and_velocities()
        self.joint_pos = (
            joint_pos_gripper[0:self.num_arm_joints] - self.joint_offsets[0:self.num_arm_joints]
        ) * self.joint_signs[0:self.num_arm_joints]
        self.gripper_pos = (joint_pos_gripper[-1] - self.joint_offsets[-1]) * self.joint_signs[-1]

        # self.joint_vel = (self.joint_pos - self.joint_pos_prev) / self.dt
        curr_joint_vel = joint_vel_gripper[0:self.num_arm_joints] * self.joint_signs[0:self.num_arm_joints]
        beta = 0.3
        self.joint_vel = beta * self.joint_vel + (1-beta) * curr_joint_vel 
        

        self.gripper_vel = (self.gripper_pos - self.gripper_pos_prev) / self.dt

        return self.joint_pos, self.joint_vel, self.gripper_pos, self.gripper_vel
    

    def set_joint_pos(self, joint_pos, gripper_pos, timeout=0.0):
        interpolation_step_size = np.ones(7)*0.5
        kp = 1200/1158.73
        kd = 50/1158.73
        curr_pos, curr_vel, curr_gripper_pos, curr_gripper_vel = self.get_joint_states()
        while (np.linalg.norm(curr_pos - joint_pos) > 0.1):# or ((time.time() - start) < timeout):
            start_time = time.time()
            next_joint_pos_target = np.where(
                np.abs(curr_pos - joint_pos) > interpolation_step_size, 
                curr_pos + interpolation_step_size*np.sign(joint_pos-curr_pos),
                joint_pos,
            )
            torque = -kp*(curr_pos-next_joint_pos_target)-kd*(curr_vel)
            gripper_torque = -kp*(curr_gripper_pos-gripper_pos)-kd*(curr_gripper_vel)
            self.set_joint_torque(torque, gripper_torque)
            curr_pos, curr_vel, curr_gripper_pos, curr_gripper_vel = self.get_joint_states()
    

    def go_to_start_pos(self):
        self.set_joint_pos(
            self.joint_pos_neutral[0:self.num_arm_joints], 
            self.joint_pos_neutral[-1], 
            2.0
        )
    

    def match_start_pos(self):
        curr_pos, _, _, _ = self.get_joint_states()
        while (np.linalg.norm(curr_pos - self.joint_pos_neutral[0:self.num_arm_joints]) > 0.5):
            print(
                "Please match starting joint pos", 
                np.linalg.norm(curr_pos - self.joint_pos_neutral[0:self.num_arm_joints])
            )
            curr_pos, _, _, _ = self.get_joint_states()
            time.sleep(0.1)
        print("Franka GELLO Joint Position Matched")

    
    def shut_down(self):
        self.set_joint_torque(np.zeros(self.num_arm_joints), 0.0)
        self.driver.set_torque_mode(False)
        
        
    def set_joint_torque(self, torque, gripper_torque):
        full_torque = np.append(torque, gripper_torque)
        self.driver.set_torque(full_torque*self.joint_signs)


    def joint_limit_barrier(self):
        kp_joint_limit = 1200/1158.73
        kd_joint_limit = 50/1158.73
        exceed_max_mask = self.joint_pos > self.joint_limits_max
        tau_l = (-kp_joint_limit * (self.joint_pos - self.joint_limits_max) - kd_joint_limit * self.joint_vel) * exceed_max_mask
        exceed_min_mask = self.joint_pos < self.joint_limits_min
        tau_l += (-kp_joint_limit * (self.joint_pos - self.joint_limits_min) - kd_joint_limit * self.joint_vel) * exceed_min_mask

        if self.gripper_pos > self.gripper_limit_max:
            tau_l_gripper = -kp_joint_limit * (self.gripper_pos - self.gripper_limit_max) - kd_joint_limit * self.gripper_vel
        elif self.gripper_pos < self.gripper_limit_min:
            tau_l_gripper = -kp_joint_limit * (self.gripper_pos - self.gripper_limit_min) - kd_joint_limit * self.gripper_vel

        else:
            tau_l_gripper = 0.0
        return tau_l, tau_l_gripper


    def gravity_compensation(self):
        self.tau_g = pin.rnea(
            self.pin_model, self.pin_data, 
            self.joint_pos, self.joint_vel, np.zeros_like(self.joint_vel)
        )


        modifier = np.ones(self.num_arm_joints)*0.8
        self.tau_g *= modifier 

        tau_gg = np.zeros(self.num_arm_joints)
        tau_gg += self.tau_g


        dither_speeds = [0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9]
        dither_coefficients = np.array([0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4]) * 1
        for i in range(self.num_arm_joints):
            if abs(self.joint_vel[i]) < dither_speeds[i]:
                if self.dither_flag[i]:
                    tau_gg[i] += dither_coefficients[i] * abs(self.tau_g[i])
                else:
                    tau_gg[i] -= dither_coefficients[i] * abs(self.tau_g[i])
                self.dither_flag[i] = ~self.dither_flag[i]

        return tau_gg 
    

    def null_space_regulation(self):
        k_pn = np.ones(self.num_arm_joints) * 0.3
        k_dn = np.ones(self.num_arm_joints) * 0.01

        J = pin.computeJointJacobian(self.pin_model, self.pin_data, self.joint_pos, 7)
        J_dagger = np.linalg.pinv(J)
        null_space_projector = np.eye(self.num_arm_joints) - J_dagger @ J

        q_error = self.joint_pos - self.joint_pos_neutral[0:self.num_arm_joints]
        
        tau_n = null_space_projector @ (-k_pn*q_error -k_dn*self.joint_vel)
    
        return tau_n * 0.7
    

    def compute_combined_torque(self):
        torque_arm = np.zeros(self.num_arm_joints)
        torque_l, torque_gripper = self.joint_limit_barrier()
        torque_arm += torque_l
        if self.gravity_comp:
            torque_arm += self.gravity_compensation()
            torque_arm += self.null_space_regulation()
        else:
            torque_arm += self.null_space_regulation()
        return torque_arm, torque_gripper
    
    
    def _gripper_external_torque_callback(self, data):
        beta = 0.95
        gripper_external_torque = data.position[0]
        self.gripper_external_torque = beta * self.gripper_external_torque + (1-beta) * gripper_external_torque


    def timer_callback(self):
        start = time.time()
        positions, _, gripper_pos, _ = self.get_joint_states()
        # print(gripper_pos)

        # send gripper position
        self.cmd_gripper_pos_pub.publish(self.create_array_msg([gripper_pos]))

        # obs franka pos
        franka_pos = self.franka_pos_sub.message
        self.obs_franka_pos_pub.publish(self.create_array_msg(franka_pos))

        # obs franka torque, set gello torque
        torque_arm, torque_gripper = self.compute_combined_torque()
        if self.torque_feedback:
            self.external_torque_sensed = self.franka_torque_sub.message

            beta = 0.
            self.external_torque = beta * self.external_torque + (1-beta) * self.external_torque_sensed 
            self.obs_franka_torque_pub.publish(self.create_array_msg(self.external_torque_sensed))

            self.external_torque = np.where(np.abs(self.external_torque)>0.0, 3.5*self.external_torque, 0.0) 
            self.external_torque = self.external_torque/94.5652173913

            torque_arm += -1.0* self.external_torque

            if abs(self.gripper_external_torque) > 0.0:
                torque_gripper += -1.0*self.gripper_external_torque / 3.53 

        torque_arm_beta = 0.0
        self.torque_arm = torque_arm_beta * self.torque_arm + (1-torque_arm_beta) * torque_arm
        self.set_joint_torque(self.torque_arm, torque_gripper)
        
        # send franka position
        self.franka_GELLO_pub.send_message(positions)
        self.cmd_franka_pos_pub.publish(self.create_array_msg(positions))
        

    

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

