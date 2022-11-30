cmake_minimum_required(VERSION 3.5)
project(cellular_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(std_msgs REQUIRED)
find_package(cav_msgs REQUIRED)
find_package(fmcsa_ace_msgs REQUIRED)

set(pluins_dependencies
  pluginlib
)

set(node_dependencies
   ${plugin_dependencies}
   rclcpp
   rclcpp_components
   rcutils
   std_msgs
   cav_msgs
   carma_v2x_msgs
)

link_directories(/workspaces/carma_ws/src/fmcsa-ace/curlpp)


add_executable( ${PROJECT_NAME}
  src/main.cpp
  src/CellularDriver.cpp
  src/easywsclient.cpp
)

ament_target_dependencies( ${PROJECT_NAME} ${node_dependencies})


target_include_directories(cellular_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
   /opt/ros/foxy/include
   /workspaces/carma_ws/src/fmcsa-ace/rapidjson
)

ament_target_dependencies(
  cellular_driver
  "fmcsa_ace_msgs"
)

#install(TARGETS cellular_driver
  #DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(TARGETS
   cellular_driver
   DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
 
