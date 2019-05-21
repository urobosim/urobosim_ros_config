cmake_minimum_required(VERSION 2.8.3)
project(unreal_simulator_demo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

find_package(Boost REQUIRED COMPONENTS thread)

# find_package(Eigen REQUIRED)
# include_directories(${Eigen_INCLUDE_DIRS})

# find_package(catkin REQUIRED COMPONENTS roslaunch)
# 	roslaunch_add_file_check(launch)

add_service_files(
		DIRECTORY srv FILES
)

# generate_messages(
# 		DEPENDENCIES
# 		std_msgs
# 		geometry_msgs
# )

include_directories(
		${catkin_INCLUDE_DIRS}
		include

		)

catkin_package(
		INCLUDE_DIRS include
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
# ## Declare a C++ library
# add_library(PR2ControllerManager
#   src/UPR2ControllerManager.cpp
# )

# add_executable(unreal_simulator_demo src/UPR2ControllerManager.cpp)

# target_link_libraries(unreal_simulator_demo
#   ${catkin_LIBRARIES}
# )

# install(
# 		TARGETS
# 		${PROJECT_NAME}
# 		# UPR2ControllerManager
# 		DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# 		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# 	   )
#
