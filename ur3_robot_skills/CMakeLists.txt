cmake_minimum_required(VERSION 3.10)
project(ur3_robot_skills)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(shape_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteSkill.action"
)

add_executable(robot_skill_server src/robot_skill_server.cpp)
ament_target_dependencies(robot_skill_server
  control_msgs
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  rclcpp_action
  shape_msgs
)
target_link_libraries(robot_skill_server
  ignition-msgs8::core
  ignition-transport11::core
)

add_executable(robot_skill_demo src/robot_skill_demo.cpp)
ament_target_dependencies(robot_skill_demo rclcpp rclcpp_action)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(robot_skill_server "${cpp_typesupport_target}")
target_link_libraries(robot_skill_demo "${cpp_typesupport_target}")

install(TARGETS robot_skill_server robot_skill_demo
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY action config launch
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
