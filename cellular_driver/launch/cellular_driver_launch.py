from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from carma_ros2_utils.launch.get_current_namespace import GetCurrentNamespace

import os
import yaml


def generate_launch_description():
    # ld = LaunchDescription()

    # config = os.path.join(
    #     get_package_share_directory('cellular_driver'),
    #     'config',
    #     'parameters.yaml'
    #     )

    # with open(config, 'r') as f:
    #     params = yaml.safe_load(f)['cellular_driver']['ros_parameters']
        
    # node=Node(
    #     package = 'cellular_driver',
    #     name = 'your_amazing_node',
    #     executable = 'cellular_driver',
    #     parameters = [params]
    # )
    # ld.add_action(node)
    # return ld

# Declare the log_level launch argument
    log_level = LaunchConfiguration('log_level')
    declare_log_level_arg = DeclareLaunchArgument(
        name ='log_level', default_value='WARN')
    
    # Get parameter file path
    param_file_path = os.path.join(
        get_package_share_directory('cellular_driver'), 'config/parameters.yaml')

        
    # Launch node(s) in a carma container to allow logging to be configured
    container = ComposableNodeContainer(
        package='carma_ros2_utils',
        name='cellular_driver_container',
        namespace=GetCurrentNamespace(),
        executable='carma_component_container_mt',
        composable_node_descriptions=[
            
            # Launch the core node(s)
            ComposableNode(
                    package='cellular_driver',
                    plugin='cellular_driver::CellularDriver',
                    name='cellular_driver_node',
                    extra_arguments=[
                        {'use_intra_process_comms': True},
                        {'--log-level' : log_level }
                    ],
                    parameters=[ param_file_path ]
            ),
        ]
    )

    return LaunchDescription([
        declare_log_level_arg,
        container
    ])
