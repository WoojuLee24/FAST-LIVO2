#!/usr/bin/python3
# -- coding: utf-8 --**
#
# Scout robot + Ouster OS1-64 + RealSense colour camera
# Dataset: 260317_scout_noeun
#
# Run:
#   ros2 launch fast_livo mapping_scout_noeun.launch.py use_rviz:=True
#   ros2 bag play /datasets/bags/260317_scout_noeun/my_camera_bag_20260317_080412/m --clock

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():

    config_file_dir = os.path.join(get_package_share_directory("fast_livo"), "config")
    rviz_config_file = os.path.join(get_package_share_directory("fast_livo"), "rviz_cfg", "scout.rviz")

    main_config_cmd   = os.path.join(config_file_dir, "scout_noeun.yaml")
    camera_config_cmd = os.path.join(config_file_dir, "camera_scout_noeun.yaml")

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="False",
        description="Whether to launch RViz2",
    )

    main_config_arg = DeclareLaunchArgument(
        "avia_params_file",
        default_value=main_config_cmd,
        description="Full path to the main fast_livo2 parameter file",
    )

    camera_config_arg = DeclareLaunchArgument(
        "camera_params_file",
        default_value=camera_config_cmd,
        description="Full path to the vikit camera parameter file",
    )

    use_respawn_arg = DeclareLaunchArgument(
        "use_respawn",
        default_value="True",
        description="Whether to respawn if a node crashes",
    )

    avia_params_file   = LaunchConfiguration("avia_params_file")
    camera_params_file = LaunchConfiguration("camera_params_file")
    use_respawn        = LaunchConfiguration("use_respawn")

    return LaunchDescription([
        use_rviz_arg,
        main_config_arg,
        camera_config_arg,
        use_respawn_arg,

        # Global parameter server for vikit camera intrinsics
        Node(
            package="demo_nodes_cpp",
            executable="parameter_blackboard",
            name="parameter_blackboard",
            parameters=[camera_params_file],
            output="screen",
        ),

        # Main FAST-LIVO2 mapping node
        Node(
            package="fast_livo",
            executable="fastlivo_mapping",
            name="laserMapping",
            parameters=[avia_params_file],
            output="screen",
            respawn=use_respawn,
        ),

        Node(
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_file],
            output="screen",
        ),
    ])
