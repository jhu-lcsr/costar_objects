cmake_minimum_required(VERSION 2.8.3)
project(costar_objects)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  python_pcd
  std_msgs
  message_generation
)

add_service_files(
  FILES AddObject.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gazebo_kinect_training
 CATKIN_DEPENDS gazebo_ros
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
