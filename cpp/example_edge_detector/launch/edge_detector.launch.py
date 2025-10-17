#!/usr/bin/env python3

import launch
import os

from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
        EnvironmentVariable,
        LaunchConfiguration
        )

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "example_edge_detector"

    this_pkg_path = get_package_share_directory(pkg_name)

    # #{ uav_name
    ld.add_action(DeclareLaunchArgument(
        'uav_name',
        default_value=EnvironmentVariable('UAV_NAME',default_value='uav1'),
        description="The uav name used for namespacing",
    ))
    # # }

    # uav_name = LaunchConfiguration('uav_name')
    uav_name=os.getenv('UAV_NAME', "uav1")
    # # } end of uav_name.

    namespace = 'waypoint_flier',

    # #{ log_level

    ld.add_action(DeclareLaunchArgument(name='log_level', default_value='info'))

    # #} end of log_level

    # # { standalone

    ld.add_action(DeclareLaunchArgument(
        'standalone',
        default_value='true'
    ))

    standalone = LaunchConfiguration('standalone')
    # #}

    # #{ use_sim_time

    use_sim_time = LaunchConfiguration('use_sim_time')

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value=os.getenv('USE_SIM_TIME', "false"),
        description="Should the node subscribe to sim time?",
    ))

    # #} end of custom_config

    # #{ edge detector node

    edge_detector_node = ComposableNode(

            package=pkg_name,
            plugin='example_edge_detector::EdgeDetector',
            namespace=uav_name,
            name='edge_detector',
            parameters=[
                {"uav_name": uav_name},
                {"use_sim_time": use_sim_time},
                {"config": this_pkg_path+'/config/edge_detector.yaml'},

            ],
            remappings=[
                # subscribers
                ("~/image_in/image", "bluefox_front/image_raw"),
                ("~/image_in/camera_info", "bluefox_front/camera_info"),
            ],
        )

    # #} end of waypoint flier node

    # #{ container_name

    container_name = LaunchConfiguration('container_name')

    declare_container_name = DeclareLaunchArgument(
        'container_name',
        default_value='',
        description='Name of an existing container to load into (if standalone is false)'
    )

    ld.add_action(declare_container_name)

    # #} end of container_name

    # #{ load into container

    load_into_existing = LoadComposableNodes(
        target_container= container_name,
        composable_node_descriptions = [edge_detector_node],
        condition = UnlessCondition(standalone)
    )

    ld.add_action(load_into_existing)

    # #} end of load into container

    # #{ standalone container

    ld.add_action(ComposableNodeContainer(
        namespace=uav_name,
        name= 'edge_detector_container',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        arguments = ['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        composable_node_descriptions=[edge_detector_node],
        condition = IfCondition(standalone)

    ))

    # #} end of standalone container

    return ld
