#!/usr/bin/env python3

import launch
import os

from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
        LaunchConfiguration,
        EnvironmentVariable,
        )

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "example_blob_detector"

    this_pkg_path = get_package_share_directory(pkg_name)

    venv_path = this_pkg_path + "/python-env/bin/python3"

    # #{ uav_name

    uav_name = LaunchConfiguration('uav_name')

    ld.add_action(DeclareLaunchArgument(
        'uav_name',
        default_value=os.getenv('UAV_NAME', "uav1"),
        description="The uav name used for namespacing.",
    ))

    # #} end of custom_config

    # #{ blob detector node

    blob_detector = Node(
        package=pkg_name,
        namespace=uav_name,
        name='blob_detector',
        executable='blob_detector.py',
        prefix=[venv_path + ' '],
        parameters=[
            get_package_share_directory('example_blob_detector') + '/config/blob_detector.yaml',
        ],
        remappings=[
            # subscribers
            ("~/image_raw_in", "bluefox_front/image_raw"),
            ("~/camera_info_in", "bluefox_front/camera_info"),
            # publishers
            ("~/image_raw_out", "~/blobs/image_raw"),
            ("~/camera_info_out", "~/blobs/camera_info"),
        ],
    )

    # #} end of sweeping generator node

    ld.add_action(blob_detector)

    return ld
