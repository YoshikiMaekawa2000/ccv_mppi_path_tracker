cmake_minimum_required(VERSION 3.0.2)
project(ccv_mppi_path_tracker)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ccv_dynamixel_msgs
  sq2_ccv_roll_pitch_msgs
  geometry_msgs
  roscpp
  sensor_msgs
  tf
)
find_package(Eigen3 REQUIRED COMPONENTS system)


catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES ccv_mppi_path_tracker
 CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)
link_directories(/opt/ros/noetic/lib)
add_executable(diff_drive_mppi src/diff_drive_mppi.cpp)
add_executable(steering_diff_drive_mppi src/steering_diff_drive_mppi.cpp)
add_executable(full_body_mppi src/full_body_mppi.cpp)
add_executable(reference_path_creator src/reference_path_creator.cpp)
add_executable(dkan_path_creator src/dkan_path_creator.cpp)
add_dependencies(diff_drive_mppi ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(reference_path_creator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(dkan_path_creator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



target_link_libraries(diff_drive_mppi
  ${catkin_LIBRARIES}
)
target_link_libraries(steering_diff_drive_mppi
  ${catkin_LIBRARIES}
)
target_link_libraries(full_body_mppi
  ${catkin_LIBRARIES}
)
target_link_libraries(reference_path_creator
  ${catkin_LIBRARIES}
)
target_link_libraries(dkan_path_creator
  ${catkin_LIBRARIES}
)
