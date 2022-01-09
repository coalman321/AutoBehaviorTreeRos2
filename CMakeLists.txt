cmake_minimum_required(VERSION 3.8)
project(auto_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_components
  behaviortree_cpp_v3
  std_srvs
  std_msgs
)

add_executable(auto_bt_node src/auto_bt.cpp src/rclcpp_logger.cpp)
target_include_directories(auto_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(auto_bt_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS auto_bt_node
  DESTINATION lib/${PROJECT_NAME})

ament_target_dependencies(auto_bt_node
  ${dependencies}
)


file(GLOB MSG_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} msg/*.msg)
file(GLOB SRV_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} srv/*.srv)
file(GLOB ACT_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} action/*.action)

#generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_SOURCES}
  ${SRV_SOURCES}
  ${ACT_SOURCES}
  DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)
rosidl_target_interfaces(auto_bt_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_package()
