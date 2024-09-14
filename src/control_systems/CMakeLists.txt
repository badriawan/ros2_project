cmake_minimum_required(VERSION 3.5)
project(control_systems)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


set(EIGEN_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/../../extern/eigen)
find_package(Eigen3 REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(angles REQUIRED)







# add_executable(talker src/publisher.cpp)
# ament_target_dependencies(talker rclcpp std_msgs)

# add_executable(listener src/subscriber.cpp)
# ament_target_dependencies(listener rclcpp std_msgs)

# add_executable(drive_turtle src/drive_turtle.cpp)
# ament_target_dependencies(drive_turtle rclcpp std_msgs geometry_msgs)

add_executable(goal_linear src/goal_linear.cpp)
ament_target_dependencies(goal_linear rclcpp std_msgs geometry_msgs nav_msgs tf2_ros)

add_executable(goal_planner src/goal_planner.cpp)
ament_target_dependencies(goal_planner rclcpp std_msgs geometry_msgs nav_msgs tf2_ros)

#installing custom library
add_library(lqr_lib STATIC src/lqr_lib.cpp)
target_include_directories(lqr_lib PRIVATE ${EIGEN_INCLUDE_DIR} include)
target_link_libraries(lqr_lib Eigen3::Eigen)

add_executable(lqr_node src/lqr_node.cpp)
target_link_libraries(lqr_node lqr_lib Eigen3::Eigen)
ament_target_dependencies(lqr_node rclcpp nav_msgs geometry_msgs tf2 angles)
target_include_directories(lqr_node PRIVATE ${EIGEN_INCLUDE_DIR} include)


install(TARGETS
  goal_linear
  goal_planner
  lqr_node
  lqr_lib
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  urdf
  config
  DESTINATION share/${PROJECT_NAME})

ament_package()