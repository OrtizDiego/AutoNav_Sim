cmake_minimum_required(VERSION 3.8)
project(person_actor_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_library(person_actor_plugin SHARED src/person_actor_plugin.cpp)
target_compile_features(person_actor_plugin PUBLIC cxx_std_17)
ament_target_dependencies(person_actor_plugin
  gazebo_dev
  gazebo_ros
  rclcpp
  geometry_msgs
  nav_msgs
)

install(TARGETS person_actor_plugin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
