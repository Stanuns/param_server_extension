cmake_minimum_required(VERSION 3.8)
project(param_server_extension)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(param_mgmt_server src/param_mgmt_server.cpp)
ament_target_dependencies(param_mgmt_server
  robot_interfaces
  rclcpp
  rclcpp_action
  rclcpp_components
  nav2_msgs
  std_srvs
  nav_msgs
)
target_link_libraries(param_mgmt_server
  ${YAML_CPP_LIBRARIES}
)

add_executable(param_dumper_client_test src/param_dumper_client_test.cpp)
ament_target_dependencies(param_dumper_client_test
  rclcpp
  rcl_interfaces
  yaml-cpp
  ament_index_cpp
)
target_link_libraries(param_dumper_client_test
  ${YAML_CPP_LIBRARIES}
)

target_include_directories(param_mgmt_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(param_mgmt_server PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS param_mgmt_server param_dumper_client_test
  DESTINATION lib/${PROJECT_NAME})

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
