cmake_minimum_required(VERSION 2.8.3)
project(unreal_controller_manager)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pr2_controller_manager
  pr2_hardware_interface
  pr2_mechanism_model
  message_generation
)

find_package(Boost REQUIRED COMPONENTS thread)


add_service_files(
		DIRECTORY srv FILES
		RegisterRobot.srv
)

generate_messages(
		DEPENDENCIES
		std_msgs
)

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

# ## Declare a C++ library
# add_library(PR2ControllerManager
#   src/UPR2ControllerManager.cpp
# )

add_executable(PR2ControllerManager src/UPR2ControllerManager.cpp)

target_link_libraries(PR2ControllerManager
  ${catkin_LIBRARIES}
)

install(
		TARGETS
		PR2ControllerManager
		DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	   )

