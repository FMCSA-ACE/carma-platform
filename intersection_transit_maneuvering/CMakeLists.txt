#
# Copyright (C) 2021 LEIDOS.
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
project(intersection_transit_maneuvering)

find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(1)
carma_package()

set( CATKIN_DEPS
  roscpp
  std_msgs
  cav_msgs
  cav_srvs
  carma_utils
  trajectory_utils
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
 ${CATKIN_DEPS}
)

## System dependencies are found with CMake's conventions
find_package(Boost)
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${CATKIN_DEPS}
  DEPENDS Boost EIGEN3
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable( ${PROJECT_NAME}
  src/main.cpp)

  
add_library(intersection_transit_maneuvering_library 
  src/intersection_transit_maneuvering.cpp
  src/itm_service.cpp
  src/call_test.cpp
)
target_link_libraries(intersection_transit_maneuvering_library ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(intersection_transit_maneuvering_library ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} intersection_transit_maneuvering_library ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${PROJECT_NAME} intersection_transit_maneuvering_library ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Install C++
install(TARGETS ${PROJECT_NAME} intersection_transit_maneuvering_library
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install Other Resources
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries

catkin_add_gmock(${PROJECT_NAME}-test 
    test/test_intersection_transit_maneuvering.cpp
    test/test_main.cpp
)
#catkin_add_gtest(${PROJECT_NAME}-test test/test_intersection_transit_maneuvering.cpp)
if(TARGET ${PROJECT_NAME}-test)
 target_link_libraries(${PROJECT_NAME}-test intersection_transit_maneuvering_library ${catkin_LIBRARIES})
endif()
