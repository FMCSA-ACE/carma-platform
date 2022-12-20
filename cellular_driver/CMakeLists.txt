cmake_minimum_required(VERSION 3.5)
project(cellular_driver)

# Declare carma package and check ROS version
find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)
carma_package()

## Find dependencies using ament auto
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Name build targets
set(node_exec cellular_driver_node_exec)
set(node_lib cellular_driver_node_lib)

# Includes
include_directories(
  include
)

# Build
ament_auto_add_library(${node_lib} SHARED
        src/CellularDriver.cpp
        src/easywsclient.cpp
)

ament_auto_add_executable(${node_exec} 
        src/main.cpp 
)

# Register component
# rclcpp_components_register_nodes(${node_lib} "cellular_driver::CellularDriver")

# All locally created targets will need to be manually linked
# ament auto will handle linking of external dependencies
target_link_libraries(${node_exec}
        ${node_lib}
)

# Install
ament_auto_package(
        INSTALL_TO_SHARE config launch
)