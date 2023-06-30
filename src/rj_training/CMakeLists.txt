cmake_minimum_required(VERSION 3.8)
project(rj_training)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(turtlesim REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/first_node.cpp
  src/topics_demo_node.cpp
  src/parameters_demo_node.cpp
  src/service_client_demo_node.cpp
  src/service_server_demo_node.cpp
  src/tf_broadcaster_demo_node.cpp
  src/tf_listener_demo_node.cpp
  src/action_server_demo_node.cpp
  src/action_client_demo_node.cpp
  )

ament_target_dependencies(${PROJECT_NAME} 
rclcpp
rclcpp_components 
std_msgs 
turtlesim
example_interfaces
tf2_ros
tf2_geometry_msgs
rclcpp_action
)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "rj_training::FirstNode"
  EXECUTABLE first_node
)
rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "rj_training::TopicsDemoNode"
  EXECUTABLE topics_demo_node
)
rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "rj_training::ParametersDemoNode"
  EXECUTABLE parameters_demo_node
)
rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "rj_training::ServiceClientDemoNode"
  EXECUTABLE service_client_demo_node
)
rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "rj_training::ServiceServerDemoNode"
  EXECUTABLE service_server_demo_node
)
rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "rj_training::TfBroadcasterDemoNode"
  EXECUTABLE tf_broadcaster_demo_node
)
rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "rj_training::TfListenerDemoNode"
  EXECUTABLE tf_listener_demo_node
)
rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "rj_training::ActionServerDemoNode"
  EXECUTABLE action_server_demo_node
)
rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "rj_training::ActionClientDemoNode"
  EXECUTABLE action_client_demo_node
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

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

ament_package()
