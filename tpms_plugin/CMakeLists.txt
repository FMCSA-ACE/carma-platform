cmake_minimum_required(VERSION 3.5)
project(tpms_plugin)

# Declare carma package and check ROS version
find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)
carma_package()

## Find dependencies using ament auto
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Name build targets
set(node_exec tpms_plugin_node_exec)
set(node_lib tpms_plugin__node_lib)

# Includes
include_directories(
  include
)

add_compile_options(-std=gnu++11 -D_GLIBCXX_USE_ABI=0 -I/ads_ws/src/carma_platform/curlpp/include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils)

find_package(std_msgs REQUIRED)
find_package(carma_v2x_msgs REQUIRED)

set(pluins_dependencies
  pluginlib
)

set(node_dependencies
   ${plugin_dependencies}
   rclcpp
   rclcpp_components
   rcutils
   std_msgs
   carma_v2x_msgs
)

link_directories(/workspaces/carma_ws/src/fmcsa-ace/curlpp)


add_executable( ${PROJECT_NAME}
  src/main.cpp
  src/TPMSPlugin.cpp
  src/TPMSAnalytics.cpp
)

ament_target_dependencies( ${PROJECT_NAME} ${node_dependencies})


target_include_directories(tpms_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
   /opt/ros/foxy/include
   /workspaces/carma_ws/src/fmcsa-ace/rapidjson
)

ament_target_dependencies(
  tpms_plugin
  carma_v2x_msgs
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(TARGETS tpms_plugin
  DESTINATION lib/${PROJECT_NAME})

add_link_options(-L/workspaces/carma_ws/src/ests_plugins/curlpp -l:libcurlpp.a -lcurl -lstdc++ )

target_link_libraries( ${PROJECT_NAME} -l:libcurlpp.a curl)

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
