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

    gripper_node = Node(
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

    franka_bridge_node = Node(
        package='bc',
        executable='franka_bridge',
        name='franka_bridge_node',          
        output='screen',                
        parameters=[
            {"connect_to_real": real_robot},
            {"torque_feedback": torque_feedback},
        ]
    )
    
    policy_rollout_node = Node(
        package='bc',
        executable='policy_rollout',
        name='policy_rollout_node',          
        output='screen',                
        parameters=[
            {"save_data": True},
            {"data_dir":"checkpoints/test/rollout"}

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
        gripper_node,
        policy_rollout_node,
        zed_node,
        franka_bridge_node,
        zed_node
    ])



