cmake_minimum_required(VERSION 3.0.2)
project(trajectory_decider)

add_compile_options(-O3)
add_compile_options(-mtune=native)
add_compile_options(-march=native)
add_compile_options(-mfpmath=both)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  kalman_filter
  color_detector_params
  roomba_500driver_meiji
  trajectory_generator
)

catkin_package(
   INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/trajectory_decider.cpp src/trajectory_decider_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
