# Copyright (C) 2018-2021 LEIDOS.
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

cmake_minimum_required(VERSION 2.8.3)
project(platoon_control)

find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(1)
carma_package()

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  carma_utils
  cav_msgs
  roscpp
  std_msgs
  autoware_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################


catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS carma_utils cav_msgs roscpp std_msgs autoware_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

file(GLOB_RECURSE headers */*.hpp */*.h)

add_executable( ${PROJECT_NAME}
  ${headers}
  src/platoon_control.cpp
  src/main.cpp)


add_library(platoon_control_plugin_lib
 src/platoon_control.cpp
 src/platoon_control_worker.cpp
 src/pid_controller.cpp
 src/pure_pursuit.cpp
)

add_dependencies(platoon_control_plugin_lib ${catkin_EXPORTED_TARGETS})

target_link_libraries(platoon_control_plugin_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME} platoon_control_plugin_lib)


#############
## Install ##
#############


install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Install C++
install(TARGETS ${PROJECT_NAME} platoon_control_plugin_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


#############
## Testing ##
#############

catkin_add_gmock(${PROJECT_NAME}-test
  test/test_pid.cpp
  test/test_pure.cpp
  test/test_worker.cpp
  test/test_control.cpp
  test/test_main.cpp)

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test platoon_control_plugin_lib ${catkin_LIBRARIES})
endif()

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_platoon_control test/mynode.test test/test_mynode.cpp)
  target_link_libraries(test_platoon_control ${catkin_LIBRARIES})
endif()


