import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

import os
import yaml


def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('tpms_plugin'),
        'config',
        'parameters.yaml'
        )

    with open(config, 'r') as f:
        params = yaml.safe_load(f)['tpms_plugin']['ros_parameters']
        
    node=Node(
        package = 'tpms_plugin',
        name = 'tpms_plugin',
        executable = 'tpms_plugin',
        parameters = [params]
    )
    ld.add_action(node)
    return ld
