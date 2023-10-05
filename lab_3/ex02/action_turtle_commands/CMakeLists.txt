cmake_minimum_required(VERSION 3.8)
project(action_turtle_commands)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(action_messages REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(turtlesim REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
# server build
add_library(action_server SHARED
  src/action_turtle_server.cpp)
target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_server
  PRIVATE "MESSAGE_TURTLE_CPP_BUILDING_DLL")
ament_target_dependencies(action_server
  rclcpp
  action_messages
  rclcpp_action
  rclcpp_components
  geometry_msgs
  std_msgs
  turtlesim)

rclcpp_components_register_node(action_server PLUGIN "action_turtle_commands::TurtleCmdActionServer" EXECUTABLE action_turtle_server)
install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
  
  
  
# client build

add_executable(action_turtle_client
  src/action_turtle_client.cpp
  )
  
  

#add_library(action_client SHARED
#  src/action_turtle_client.cpp)
#target_include_directories(action_client PRIVATE
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)
#target_compile_definitions(action_client
#  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(action_turtle_client
  action_messages
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  std_msgs
  turtlesim)
  
install(TARGETS
  action_turtle_client
  DESTINATION lib/${PROJECT_NAME})
#rclcpp_components_register_node(action_client PLUGIN "action_tutorials_cpp::FibonacciActionClient" EXECUTABLE action_turtle_client)
#install(TARGETS
#  action_client
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION bin)

ament_package()
