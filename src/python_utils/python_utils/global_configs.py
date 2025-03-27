sim_desktop_ip_address = "172.16.0.9"
franka_left_ip_address = "172.16.0.1"
franka_right_ip_address = "172.16.0.3"


franka_right_real_zmq_addresses = {
    "joint_state_sub":  f"tcp://{franka_right_ip_address}:3099",
    "joint_torque_sub": f"tcp://{franka_right_ip_address}:3087",
    "joint_pos_cmd_pub": f"tcp://{sim_desktop_ip_address}:2098",

}

franka_right_sim_zmq_addresses = {
    "joint_state_sub":  f"tcp://{sim_desktop_ip_address}:3099",
    "joint_torque_sub": f"tcp://{sim_desktop_ip_address}:3087",
    "joint_pos_cmd_pub": f"tcp://{sim_desktop_ip_address}:2098",

}

franka_left_real_zmq_addresses = {
    "joint_state_sub":  f"tcp://{franka_left_ip_address}:5099",
    "joint_torque_sub": f"tcp://{franka_left_ip_address}:5087",
    "joint_pos_cmd_pub": f"tcp://{sim_desktop_ip_address}:4098",

}

franka_left_sim_zmq_addresses = {
    "joint_state_sub":  f"tcp://{sim_desktop_ip_address}:5099",
    "joint_torque_sub": f"tcp://{sim_desktop_ip_address}:5087",
    "joint_pos_cmd_pub": f"tcp://{sim_desktop_ip_address}:4098",
}




# GELLO with gripper
left_GELLO_gripper_configs = {
    "joint_signs": [1, -1, 1, -1, 1, -1, 1, 1],
    "joint_ids": [1, 2, 3, 4, 5, 6, 7, 8],
    "dynamixel_port": "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT9HDB0V-if00-port0",
    "joint_limits": [0.0, 0.5],
}

right_GELLO_gripper_configs = {
    "joint_signs": [1, 1, 1, -1, 1, -1, 1, -1],
    "joint_ids": [1, 2, 3, 4, 5, 6, 7, 8],
    "dynamixel_port": "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT951EJA-if00-port0",
    "joint_limits": [0.0, 1.0],
}


# Gripper
left_gripper_configs = {
    "joint_signs": [1],
    "joint_ids": [1],
    "dynamixel_port": "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7W91VW-if00-port0"
}

right_gripper_configs = {
    "joint_signs": [1],
    "joint_ids": [1],
    "dynamixel_port": "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7WBF8S-if00-port0",
}
