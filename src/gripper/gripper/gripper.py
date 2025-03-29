import numpy as np
import time
import os
import subprocess

import rclpy
from rclpy.node import Node

from gripper.dynamixel.driver import DynamixelDriver

from std_msgs.msg import Header
from python_utils.global_configs import left_gripper_configs, right_gripper_configs
from python_utils.global_configs import right_factr_teleop_gripper_configs

from sensor_msgs.msg import JointState


class Gripper(Node):
    
    def create_array_msg(self, data):
        msg = JointState()
        msg.position = list(map(float, data))
        return msg
    
    def __init__(self):
        super().__init__('gripper')    
        self.connect_to_real = self.declare_parameter('connect_to_real', False).get_parameter_value().bool_value
        self.torque_feedback = self.declare_parameter('torque_feedback', False).get_parameter_value().bool_value
        self.is_left = self.declare_parameter('is_left', False).get_parameter_value().bool_value

        if self.is_left:
            self.gripper_name = "left"
            gripper_configs = left_gripper_configs
        else:
            self.gripper_name = "right"
            gripper_configs = right_gripper_configs
        
        self.joint_sign = gripper_configs["joint_signs"][0]
        self.dynamixel_port = gripper_configs["dynamixel_port"]

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

        self.dt = 1/200

        # 0.0 rad is closed
        self.gripper_limit_max = 1.74
        self.gripper_limit_min = -0.3

        # teleop gripper limits
        self.teleop_gripper_limits = right_factr_teleop_gripper_configs["joint_limits"]

        self.gripper_pos_prev = 0.0
        self.gripper_pos = 0.0
        self.gripper_vel = 0.0

        self.gripper_cmd_pos = 0.0
        self.gripper_cmd_torque = 0.0
        self.gripper_sensed_torque = 0.0
        self.gripper_external_torque = 0.0

        self._prepare_messengers()

        try:
            self.driver = DynamixelDriver(gripper_configs["joint_ids"], port=self.dynamixel_port)
        except FileNotFoundError:
            print(f"Port {self.dynamixel_port} not found. Please check the connection.")
            return
    
        self.driver.set_torque_mode(False)
        self.driver.set_operating_mode(0)
        self.driver.set_torque_mode(True)
        
        self.get_offsets()
        self.timer = self.create_timer(self.dt, self._timer_callback)


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
        self.gripper_pos_pub = self.create_publisher(
            JointState, f'/gripper/{self.gripper_name}/obs_gripper_pos', 1
        )
        self.gripper_cmd_pos_sub = self.create_subscription(
            JointState, f'/gello/{self.gripper_name}/cmd_gripper_pos', 
            self._gripper_cmd_pos_callback, 
            1,
        )

        if self.torque_feedback:
            self.gripper_ext_torque_pub = self.create_publisher(
                JointState, f'/gripper/{self.gripper_name}/obs_gripper_torque', 1
            )
    
    def _gripper_cmd_pos_callback(self, data):
        # min, max
        
        teleop_gripper_pos = data.position[0]

        gripper_cmd_pos = ((teleop_gripper_pos - self.teleop_gripper_limits[0]) / (self.teleop_gripper_limits[1] - self.teleop_gripper_limits[0])) \
            * (self.gripper_limit_max - self.gripper_limit_min) + self.gripper_limit_min

        self.gripper_cmd_pos = np.clip(
            gripper_cmd_pos,
            self.gripper_limit_min,
            self.gripper_limit_max,
        )


    def get_offsets(self, verbose=True):
        # warm up
        for _ in range(10):
            self.driver.get_positions_velocities_and_currents()

        print("Gripper started calibrating")
        start_time = time.time()
        while (time.time() - start_time) < 3.0:
            self.set_joint_torque(-0.3)
        print("Gripper finished calibrating")

        # get gripper offset:
        curr_gripper_joint, _, _ = self.driver.get_positions_velocities_and_currents()
        self.joint_offset = curr_gripper_joint[0]
        if verbose:
            print(self.joint_offset)


    def get_joint_states(self):
        self.gripper_pos_prev = self.gripper_pos
        pos, vel, cur = self.driver.get_positions_velocities_and_currents()
        self.gripper_pos = (pos - self.joint_offset) * self.joint_sign
        self.gripper_vel = (self.gripper_pos - self.gripper_pos_prev) / self.dt
        self.gripper_sensed_torque = cur* self.joint_sign / 1000.0
        return self.gripper_pos, self.gripper_vel, self.gripper_sensed_torque


    def set_joint_pos(self, gripper_pos_target, timeout=0.0):
        kp = 1200/1158.73
        kd = 50/1158.73
        start = time.time()
        curr_pos, curr_vel, _, = self.get_joint_states()
        while (abs(curr_pos - gripper_pos_target) > 0.1): # or ((time.time() - start) < timeout):
            start_time = time.time()
            torque = -kp*(curr_pos-gripper_pos_target)-kd*(curr_vel)
            self.set_joint_torque(torque)
            curr_pos, curr_vel, _, = self.get_joint_states()
            print(time.time() - start_time)


    def go_to_start_pos(self):
        self.set_joint_pos(0.0, 2.0)


    def set_joint_torque(self, torque):
        self.driver.set_torque(np.array([torque])*self.joint_sign)

    
    def shut_down(self):
        self.set_joint_torque(0.0)
        self.driver.set_torque_mode(False)



    def _timer_callback(self):
        kp = 1800/1000.0
        kd = 50/1000.0

        position, velocity, sensed_torque = self.get_joint_states()

        # print(position, self.gripper_cmd_pos)
        # print(sensed_torque, self.gripper_cmd_torque)
        self.gripper_external_torque = sensed_torque
        
        self.gripper_pos_pub.publish(self.create_array_msg([position]))
        if self.torque_feedback:
            self.gripper_ext_torque_pub.publish(self.create_array_msg([self.gripper_external_torque]))

        self.gripper_cmd_torque = -kp*(position-self.gripper_cmd_pos)-kd*(velocity)
        self.set_joint_torque(self.gripper_cmd_torque)



def main(args=None):
    rclpy.init(args=args)
    gripper = Gripper()

    try:
        while rclpy.ok():
            rclpy.spin(gripper)
    except KeyboardInterrupt:
        gripper.get_logger().info("Keyboard interrupt received. Shutting down...")
        gripper.shut_down()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()






