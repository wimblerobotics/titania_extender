cmake_minimum_required(VERSION 3.16)
project(titania_extender)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveToPosition.action"
  "msg/MotorStatus.msg"
  DEPENDENCIES std_msgs action_msgs
)

add_executable(titania_extender_node
  src/titania_extender_node.cpp
  src/roboclaw.cpp
  src/bno055.cpp
  src/vl53l4cd.cpp
)
add_executable(i2c_scan src/i2c_scan.cpp)
target_include_directories(titania_extender_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(titania_extender_node
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}
  rosidl_typesupport_cpp)
target_link_libraries(titania_extender_node "${cpp_typesupport_target}")

install(TARGETS titania_extender_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS i2c_scan
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(homing_test test/homing_test.cpp)
  target_include_directories(homing_test PRIVATE include)
  add_test(NAME homing_test COMMAND homing_test)
  add_executable(homing_test_node
    src/titania_extender_node.cpp src/roboclaw.cpp test/fake_sensors.cpp)
  target_include_directories(homing_test_node PRIVATE include)
  target_link_libraries(homing_test_node
    rclcpp::rclcpp rclcpp_action::rclcpp_action
    ${std_msgs_TARGETS} ${std_srvs_TARGETS} ${geometry_msgs_TARGETS}
    ${sensor_msgs_TARGETS} "${cpp_typesupport_target}")
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME homing_integration
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_homing.py
      $<TARGET_FILE:homing_test_node>)
  set_tests_properties(homing_integration PROPERTIES TIMEOUT 90)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
