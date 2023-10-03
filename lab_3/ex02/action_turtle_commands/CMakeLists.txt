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

add_library(action_server SHARED
  src/server.cpp)
  
target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
target_compile_definitions(action_server
  PRIVATE "MESSAGE_TURTLE_CPP_BUILDING_DLL")
  

add_executable(action_turtle_client src/client.cpp)
ament_target_dependencies(action_turtle_client rclcpp action_messages)

add_executable(action_turtle_server src/server.cpp)
ament_target_dependencies(action_turtle_server
  rclcpp
  action_messages
  rclcpp_action
  rclcpp_components)

rclcpp_components_register_node(action_turtle_server PLUGIN "action_turtle_commands::TurtleCmdActionServer" EXECUTABLE server)



install(TARGETS
  action_turtle_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
