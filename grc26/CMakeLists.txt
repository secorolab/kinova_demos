cmake_minimum_required(VERSION 3.8)
project(grc26)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(bdd_ros2_interfaces REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)

include_directories(include)

# ------------------------
# Messages
# ------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PIDDebug.msg"
  "msg/CartesianPIDDebug.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# ------------------------
# Executables
# ------------------------

add_executable(kinova_single_arm_demo src/kinova_single_arm_demo.cpp)
target_include_directories(kinova_single_arm_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(kinova_single_arm_demo PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(kinova_single_arm_demo
  rclcpp
  ament_index_cpp
  std_msgs
  sensor_msgs
  geometry_msgs
  bdd_ros2_interfaces
  kdl_parser
  orocos_kdl
)

install(TARGETS kinova_single_arm_demo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(TARGETS kinova_single_arm_demo
  DESTINATION lib/${PROJECT_NAME})

ament_package()
