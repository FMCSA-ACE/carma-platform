
# Copyright (C) 2020-2022 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

cmake_minimum_required(VERSION 3.5)
project(motion_computation)

# Declare carma package and check ROS version
find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)
carma_package()

## Find dependencies using ament auto
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Name build targets
set(worker_lib motion_computation_worker_lib)
set(node_exec motion_computation_node_exec)
set(node_lib motion_computation_node_lib)

# Includes
include_directories(
  include
)

# boost::posix_time definition for using nanoseconds
add_definitions(-DBOOST_DATE_TIME_POSIX_TIME_STD_CONFIG)

# Build

ament_auto_add_library(${worker_lib} SHARED
        src/motion_computation_worker.cpp
        src/mobility_path_to_external_object.cpp
        src/psm_to_external_object_convertor.cpp
        src/bsm_to_external_object_convertor.cpp
)

ament_auto_add_library(${node_lib} SHARED
        src/motion_computation_node.cpp
)

ament_auto_add_executable(${node_exec} 
        src/main.cpp 
)

# Register component
rclcpp_components_register_nodes(${node_lib} "motion_computation::MotionComputationNode")

# All locally created targets will need to be manually linked
# ament auto will handle linking of external dependencies
target_link_libraries(${node_lib}
        ${worker_lib}
)

target_link_libraries(${node_exec}
        ${node_lib}
)

# Testing
if(BUILD_TESTING)  

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # This populates the ${${PROJECT_NAME}_FOUND_TEST_DEPENDS} variable

  ament_add_gtest(test_motion_computation 
        test/MotionComputationTest.cpp
        test/TestMain.cpp)

  ament_target_dependencies(test_motion_computation ${${PROJECT_NAME}_FOUND_TEST_DEPENDS})

  target_link_libraries(test_motion_computation ${node_lib})

endif()

# Install
ament_auto_package(
        INSTALL_TO_SHARE config launch
)
