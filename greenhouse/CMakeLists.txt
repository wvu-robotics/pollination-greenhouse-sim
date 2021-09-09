cmake_minimum_required(VERSION 3.0.2)
project(greenhouse)

find_package(catkin REQUIRED COMPONENTS
  roslaunch
  gazebo_ros
)

find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(
  CATKIN_DEPENDS gazebo_ros
  DEPENDS GAZEBO
)

catkin_add_env_hooks(50.greenhouse
  SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
  