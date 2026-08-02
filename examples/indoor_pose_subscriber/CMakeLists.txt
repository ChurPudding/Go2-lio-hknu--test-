cmake_minimum_required(VERSION 3.8)
project(indoor_pose_subscriber)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(indoor_pose_subscriber src/indoor_pose_subscriber.cpp)
ament_target_dependencies(indoor_pose_subscriber rclcpp nav_msgs)

install(TARGETS indoor_pose_subscriber DESTINATION lib/${PROJECT_NAME})

ament_package()
