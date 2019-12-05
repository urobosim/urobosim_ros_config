cmake_minimum_required(VERSION 2.8.3)
project(urobosim_ros_config)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

find_package(Boost REQUIRED COMPONENTS thread)

include_directories(
		${catkin_INCLUDE_DIRS}
		)

catkin_package(
		INCLUDE_DIRS 
		CATKIN_DEPENDS
		message_runtime
		message_generation
		roscpp
		rospy
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
add_compile_options(-std=c++11)
