#
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
#

cmake_minimum_required(VERSION 2.8.3)
project(route_following_plugin)

find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(1)
carma_package()

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cav_msgs
  cav_srvs
  carma_utils
  carma_wm
  tf
  tf2
  tf2_geometry_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs cav_srvs carma_utils cav_msgs carma_wm tf tf2 tf2_geometry_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include
)


add_executable( ${PROJECT_NAME}
  src/route_following_plugin.cpp
  src/main.cpp)

add_library(route_following_plugin_lib src/route_following_plugin.cpp)
add_dependencies(route_following_plugin_lib ${catkin_EXPORTED_TARGETS})

target_link_libraries(route_following_plugin_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES})

target_link_libraries(${PROJECT_NAME} route_following_plugin_lib)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Install C++
install(TARGETS ${PROJECT_NAME} route_following_plugin_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install Other Resources
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
catkin_add_gmock(${PROJECT_NAME}-test
test/test_route_following_plugin.cpp
 WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)

catkin_add_gmock(${PROJECT_NAME}-stopping-test
test/test_stop_at_end_of_route.cpp
 WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
 
target_link_libraries(${PROJECT_NAME}-test route_following_plugin_lib ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}-stopping-test route_following_plugin_lib ${catkin_LIBRARIES})
