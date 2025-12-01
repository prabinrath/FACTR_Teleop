# ---------------------------------------------------------------------------
# Prabin Kumar Rath
# Licensed under the Apache License, Version 2.0
# ---------------------------------------------------------------------------

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch args
    namespace = LaunchConfiguration("namespace")
    robot_ip = LaunchConfiguration("robot_ip")

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Robot namespace (e.g. 'fr3'). Leave empty for none.",
    )
    declare_robot_ip = DeclareLaunchArgument(
        "robot_ip",
        default_value="172.16.0.2",
        description="IP address of the Franka FCI controller.",
    )

    # Franka bringup include
    controllers_yaml = PathJoinSubstitution(
        [
            FindPackageShare("factr_teleop"),
            "configs",
            "fr3_ros_controllers.yaml",
        ]
    )
    franka_bringup_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("factr_teleop"), "launch", "franka.launch.py"]
            )
        ),
        launch_arguments={
            "robot_ip": robot_ip,
            "arm_id": "fr3",
            "load_gripper": "true",
            "controllers_yaml": controllers_yaml,
            "namespace": namespace,
        }.items(),
    )

    # Spawn the arm controller
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        namespace=namespace,
        arguments=[
            "fr3_arm_controller",
            "--controller-manager-timeout",
            "60",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # https://github.com/ros-drivers/joystick_drivers
    spacemouse = Node(
            package='spacenav',          
            executable='spacenav_node',  
            name='spacenav_node',        
            output='screen',             
            parameters=[{
                'publish_joy': True,     
                'publish_twist': True,
                'zero_when_static': True,
            }],
        )

    franka_rollout_node = Node(
        package='bc',
        executable='franka_rollout',
        name='franka_rollout',
        output='screen',
        namespace=namespace,
    )

    franka_error_recovery_node = Node(
        package='bc',
        executable='franka_error_recovery',
        name='franka_error_recovery',
        output='screen',
        namespace=namespace,
    )

    return LaunchDescription(
        [
            declare_namespace,
            declare_robot_ip,
            franka_bringup_launch_file,
            controller_spawner,
            spacemouse,
            franka_rollout_node,
            franka_error_recovery_node,
        ]
    )
