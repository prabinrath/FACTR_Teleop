#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

from launch import LaunchDescription
from launch_ros.actions import Node
real_robot = True

if real_robot:
    torque_feedback = True
else:
    torque_feedback = False

torque_feedback = True


def generate_launch_description():

    data_record_node = Node(
        package='bc',
        executable='data_record',
        name='data_record_node',          
        output='screen',                
        parameters=[
            {
                "state_topics": [
                    "/gello/right/cmd_franka_pos",
                    "/franka/right/obs_franka_pos",
                    "/gripper/right/obs_gripper_pos",
                ]
            },
            {
                "image_topics": [
                    "/zed/front/im_left",
                ]
            },
            {"dataset_name": "test_test_2"}
        ]
    )
    
    factr_teleop_franka_right = Node(
        package='factr_teleop',
        executable='factr_teleop_franka',
        name='factr_teleop_franka_right',
        output='screen',
        emulate_tty=True,
        parameters=[
            {"connect_to_real": real_robot},
            {"torque_feedback": torque_feedback},
            {"gravity_comp": torque_feedback},
            {"teleop_device_attached": False},
            {"is_left": False},
        ]
    )

    gripper_right_node = Node(
        package='gripper',
        executable='gripper',
        name='gripper_right',
        output='screen',
        emulate_tty=True,
        parameters=[
            {"connect_to_real": real_robot},
            {"torque_feedback": torque_feedback},
            {"is_left": False},
        ]
    )
    
    zed_node = Node(
        package='cameras',
        executable='zed',
        name='front',
        output='screen',
        emulate_tty=True,
        parameters=[
            {"serial": 22176523},
            {"name": "front"},
        ]
    )
    
    return LaunchDescription([
        factr_teleop_franka_right,
        gripper_right_node,
        data_record_node,
        zed_node
    ])