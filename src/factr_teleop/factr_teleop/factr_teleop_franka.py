import numpy as np
import time
import pinocchio as pin
import os
import subprocess
import yaml
from abc import ABC, abstractmethod

from rclpy.node import Node

from factr_teleop.dynamixel.driver import DynamixelDriver
from python_utils.utils import get_workspace_root


def find_ttyusb(port_name):
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


class FACTRTeleopFranka(Node, ABC):
    def __init__(self):
        super().__init__('factr_teleop_franka')

        config_file_name = self.declare_parameter('config_file', '').get_parameter_value().string_value
        config_path = os.path.join(get_workspace_root(), f"src/factr_teleop/factr_teleop/configs/{config_file_name}")
        with open(config_path, 'r') as config_file:
            self.config = yaml.safe_load(config_file)
        
        self.name = self.config["name"]
        self.dt = 1/500
        self.num_motors = 8
        self.num_arm_joints = 7

        self._prepare_dynamixel()
        self._prepare_inverse_dynamics()

        # leader arm parameters
        self.arm_joint_limits_max = np.array([2.8973, 1.7628, 2.8973, -0.0698-0.8, 2.8973, 3.7525, 2.8973]) - 0.1
        self.arm_joint_limits_min = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]) + 0.1
        # leader gripper parameters
        self.gripper_limit_min = 0.0
        self.gripper_limit_max = self.config["gripper_teleop"]["actuation_range"]
        self.gripper_pos_prev = 0.0
        self.gripper_pos = 0.0

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
        
        # needs to be implemented to establish communication between the leader and the follower
        self.set_up_communication()

        # calibrate the leader arm joints before starting
        self.calibration_joint_pos = np.array(self.config["initialization"]["calibration_joint_pos"])
        self._get_offsets()
        # ensure the leader and the follower arms have the same joint positions before starting
        self.initial_match_joint_pos = np.array(self.config["initialization"]["initial_match_joint_pos"])
        self._match_start_pos()
        # start the control loop
        self.timer = self.create_timer(self.dt, self.control_loop_callback)


    def _prepare_dynamixel(self):
        self.joint_signs = np.array(self.config["dynamixel"]["joint_signs"], dtype=float)
        self.dynamixel_port = self.config["dynamixel"]["dynamixel_port"]

        # checks of the latency timer on ttyUSB of the corresponding port is 1
        # if it is not 1, the control loop cannot run at 200 Hz, which will cause
        # extremely undesirable behaviour to the GELLO. If the latency timer is not
        # 1, one can set it to 1 as follows:
        # echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB{NUM}/latency_timer
        ttyUSBx = find_ttyusb(self.dynamixel_port)
        command = f"cat /sys/bus/usb-serial/devices/{ttyUSBx}/latency_timer"        
        result = subprocess.run(command, shell=True, capture_output=True, text=True, check=True)
        ttyUSB_latency_timer = int(result.stdout)
        if ttyUSB_latency_timer != 1:
            raise Exception(
                f"Please ensure the latency timer of {ttyUSBx} is 1. Run: \n \
                echo 1 | sudo tee /sys/bus/usb-serial/devices/{ttyUSBx}/latency_timer"
            )
        
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


    def _get_offsets(self, verbose=True):
        # warm up
        for _ in range(10):
            self.driver.get_positions_and_velocities()
        
        def _get_error(calibration_joint_pos, offset, index, joint_state):
            joint_sign_i = self.joint_signs[index]
            joint_i = joint_sign_i * (joint_state[index] - offset)
            start_i = calibration_joint_pos[index]
            return np.abs(joint_i - start_i)

        # get arm offsets
        self.joint_offsets = []
        curr_joints, _ = self.driver.get_positions_and_velocities()
        for i in range(self.num_arm_joints):
            best_offset = 0
            best_error = 1e9
            # intervals of pi/2
            for offset in np.linspace(-20 * np.pi, 20 * np.pi, 20 * 4 + 1):  
                error = _get_error(self.calibration_joint_pos, offset, i, curr_joints)
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
    

    def _match_start_pos(self):
        curr_pos, _, _, _ = self.get_leader_joint_states()
        while (np.linalg.norm(curr_pos - self.initial_match_joint_pos[0:self.num_arm_joints]) > 0.5):
            current_joint_error = np.linalg.norm(curr_pos - self.initial_match_joint_pos[0:self.num_arm_joints])
            self.get_logger().info(
                f"FACTR TELEOP {self.name}: Please match starting joint pos. Current error: {current_joint_error}"
            )
            curr_pos, _, _, _ = self.get_leader_joint_states()
            time.sleep(0.5)
        self.get_logger().info(f"FACTR TELEOP {self.name}: Initial joint position matched.")

    
    def get_leader_joint_states(self):
        self.gripper_pos_prev = self.gripper_pos
        joint_pos, joint_vel = self.driver.get_positions_and_velocities()
        joint_pos_arm = (
            joint_pos[0:self.num_arm_joints] - self.joint_offsets[0:self.num_arm_joints]
        ) * self.joint_signs[0:self.num_arm_joints]
        self.gripper_pos = (joint_pos[-1] - self.joint_offsets[-1]) * self.joint_signs[-1]
        joint_vel_arm = joint_vel[0:self.num_arm_joints] * self.joint_signs[0:self.num_arm_joints]
        
        gripper_vel = (self.gripper_pos - self.gripper_pos_prev) / self.dt
        return joint_pos_arm, joint_vel_arm, self.gripper_pos, gripper_vel
    

    def set_leader_joint_pos(self, goal_joint_pos, goal_gripper_pos):
        interpolation_step_size = np.ones(7)*self.config["controller"]["interpolation_step_size"]
        kp = self.config["controller"]["joint_position_control"]["kp"]
        kd = self.config["controller"]["joint_position_control"]["kd"]

        curr_pos, curr_vel, curr_gripper_pos, curr_gripper_vel = self.get_leader_joint_states()
        while (np.linalg.norm(curr_pos - goal_joint_pos) > 0.1):
            next_joint_pos_target = np.where(
                np.abs(curr_pos - goal_joint_pos) > interpolation_step_size, 
                curr_pos + interpolation_step_size*np.sign(goal_joint_pos-curr_pos),
                goal_joint_pos,
            )
            torque = -kp*(curr_pos-next_joint_pos_target)-kd*(curr_vel)
            gripper_torque = -kp*(curr_gripper_pos-goal_gripper_pos)-kd*(curr_gripper_vel)
            self.set_joint_torque(torque, gripper_torque)
            curr_pos, curr_vel, curr_gripper_pos, curr_gripper_vel = self.get_leader_joint_states()
    

    def shut_down(self):
        self.set_joint_torque(np.zeros(self.num_arm_joints), 0.0)
        self.driver.set_torque_mode(False)
        
        
    def set_leader_joint_torque(self, torque, gripper_torque):
        full_torque = np.append(torque, gripper_torque)
        self.driver.set_torque(full_torque*self.joint_signs)


    def joint_limit_barrier(self, arm_joint_pos, arm_joint_vel, gripper_joint_pos, gripper_joint_vel):
        exceed_max_mask = arm_joint_pos > self.arm_joint_limits_max
        tau_l = (-self.joint_limit_kp * (arm_joint_pos - self.arm_joint_limits_max) - self.joint_limit_kd * arm_joint_vel) * exceed_max_mask
        exceed_min_mask = arm_joint_pos < self.arm_joint_limits_min
        tau_l += (-self.joint_limit_kp * (arm_joint_pos - self.arm_joint_limits_min) - self.joint_limit_kd * arm_joint_vel) * exceed_min_mask

        if gripper_joint_pos > self.gripper_limit_max:
            tau_l_gripper = -self.joint_limit_kp * (gripper_joint_pos - self.gripper_limit_max) - self.joint_limit_kd * gripper_joint_vel
        elif gripper_joint_pos < self.gripper_limit_min:
            tau_l_gripper = -self.joint_limit_kp * (gripper_joint_pos - self.gripper_limit_min) - self.joint_limit_kd * gripper_joint_vel
        else:
            tau_l_gripper = 0.0
        return tau_l, tau_l_gripper


    def gravity_compensation(self, arm_joint_pos, arm_joint_vel):
        self.tau_g = pin.rnea(
            self.pin_model, self.pin_data, 
            arm_joint_pos, arm_joint_vel, np.zeros_like(arm_joint_vel)
        )
        self.tau_g *= self.gravity_comp_modifier 
        return self.tau_g


    def friction_compensation(self, arm_joint_vel):
        tau_ss = np.zeros(self.num_arm_joints)
        for i in range(self.num_arm_joints):
            if abs(arm_joint_vel[i]) < self.stiction_comp_enable_speed:
                if self.stiction_dither_flag[i]:
                    tau_ss[i] += self.stiction_comp_gain * abs(self.tau_g[i])
                else:
                    tau_ss[i] -= self.stiction_comp_gain * abs(self.tau_g[i])
                self.stiction_dither_flag[i] = ~self.stiction_dither_flag[i]
        return tau_ss
    

    def null_space_regulation(self, arm_joint_pos, arm_joint_vel):
        J = pin.computeJointJacobian(self.pin_model, self.pin_data, arm_joint_pos, self.num_arm_joints)
        J_dagger = np.linalg.pinv(J)
        null_space_projector = np.eye(self.num_arm_joints) - J_dagger @ J
        q_error = arm_joint_pos - self.null_space_joint_target[0:self.num_arm_joints]
        tau_n = null_space_projector @ (-self.null_space_kp*q_error -self.null_space_kd*arm_joint_vel)
        return tau_n
    

    def torque_feedback(self, external_torque):
        tau_ff = -1.0*self.torque_feedback_gain/self.torque_feedback_motor_scalar * external_torque
        return tau_ff


    def control_loop_callback(self):
        leader_arm_pos, leader_arm_vel, leader_gripper_pos, leader_gripper_vel = self.get_leader_joint_states()

        torque_arm = np.zeros(self.num_arm_joints)
        torque_l, torque_gripper = self.joint_limit_barrier(
            leader_arm_pos, leader_arm_vel, leader_gripper_pos, leader_gripper_vel
        )
        torque_arm += torque_l
        torque_arm += self.null_space_regulation(leader_arm_pos, leader_arm_vel)

        if self.enable_gravity_comp:
            torque_arm += self.gravity_compensation(leader_arm_pos, leader_arm_vel)
            torque_arm += self.friction_compensation(leader_arm_vel)
        
        if self.enable_torque_feedback:
            external_joint_torque = self.get_leader_arm_external_joint_torque()
            torque_arm += self.torque_feedback(external_joint_torque)
        
        if self.enable_gripper_feedback:
            gripper_feedback = self.get_leader_gripper_feedback()
            torque_gripper += self.gripper_feedback(gripper_feedback)

        self.set_leader_joint_torque(torque_arm, torque_gripper)
        self.update_communication(leader_arm_pos, leader_gripper_pos)


    @abstractmethod
    def set_up_communication(self):
        """
        This method should be implemented to set up communication between the leader arm
        and the follower arm for bilateral teleoperation. This method is called once
        in the __init__ method.
        
        For example, a subscriber can  be set up to receive external joint torque from 
        the leader arm and a publisher can be set up to send joint position target commands 
        to the follower arm. Publishers and subscribers can also be set up to record
        the follower arm's joint states

        Raises:
            NotImplementedError: If the method is not implemented in a subclass.
        """
        pass


    @abstractmethod
    def get_leader_arm_external_joint_torque(self):
        """
        This method should retrieve the current external joint torque from the follower arm.
        This is used to compute force-feedback in the leader arm. This method is called at
        every iteration of the control loop if self.enable_torque_feedback is set to True.

        Returns:
            np.ndarray: A NumPy array of shape (num_arm_joints,) containing the external 
            joint torques. 

        Raises:
            NotImplementedError: If the method is not implemented in a subclass.
        """
        pass


    @abstractmethod
    def get_leader_gripper_feedback(self):
        """
        This method should retrieve any data from the follower gripper that might be required
        to achieve force-feedback in the leader gripper. For example, this method can be used
        to get the current position of the follower gripper for position-position force-feedback
        or the current force of the follower gripper for position-force force-feedback in the
        leader gripper. This method is called at every iteration of the control loop if 
        self.enable_gripper_feedback is set to True.

        Returns:
            Any: Feedback data required by the leader gripper. This can be a NumPy array, a 
            scalar, or any other data type depending on the implementation.

        Raises:
            NotImplementedError: If the method is not implemented in a subclass.
        """
        pass


    @abstractmethod
    def gripper_feedback(self, gripper_feedback):
        """
        Processes feedback data from the follower gripper. This method is intended to compute 
        force-feedback for the leader gripper. This method is called at every iteration of the 
        control loop if self.enable_gripper_feedback is set to True.

        Args:
            gripper_feedback (Any): Feedback data from the gripper. The format can vary depending 
            on the implementation, such as a NumPy array, scalar, or custom object.
        
        Returns:
            float: The computed joint torque value to apply force-feedback to the leader gripper.

        Raises:
            NotImplementedError: If the method is not implemented in a subclass.
        """
        pass


    @abstractmethod
    def update_communication(self, leader_arm_pos, leader_gripper_pos):
        """
        This method is intended to be called at every iteration of the control loop to transmit 
        relevant data, such as joint position targets, from the leader to the follower arm.

        Args:
            leader_arm_pos (np.ndarray): A NumPy array containing the joint positions of the leader arm.
            leader_gripper_pos (np.ndarray): A NumPy array containing the position of the leader gripper.

        Raises:
            NotImplementedError: If the method is not implemented in a subclass.
        """
        pass
