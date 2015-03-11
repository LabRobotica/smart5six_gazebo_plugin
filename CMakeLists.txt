cmake_minimum_required(VERSION 2.8.3)
project(smart5six_gazebo_plugin)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

find_package(catkin REQUIRED COMPONENTS roscpp rospy gazebo_ros std_msgs trajectory_msgs genmsg message_generation tf urdf industrial_utils industrial_robot_client simple_message hardware_interface controller_manager controller_interface actionlib_msgs actionlib control_msgs)

#build using LINUX SOCKETS libraries
add_definitions(-DLINUXSOCKETS=1)  

set(MESSAGE_DEPENDENCIES std_msgs)
add_message_files(
	FILES 
        endEffectorPosition.msg
)
generate_messages(
	DEPENDENCIES
	${MESSAGE_DEPENDENCIES}
	actionlib_msgs
)

catkin_package(
	INCLUDE_DIRS include
	DEPENDS roscpp gazebo_ros message_runtime std_msgs trajectory_msgs genmsg actionlib control_msgs
)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

include_directories(include include/Reflexxes ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

	
add_library(ReflexxesLib SHARED IMPORTED)
set_property(TARGET ReflexxesLib PROPERTY IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/libReflexxesTypeII.so)


if (EXISTS /opt/lib/libeORL.so)

    #define preprocessor variable COMAU
    add_definitions(-DCOMAU)
	add_library(ComauLib SHARED IMPORTED)
	set_property(TARGET ComauLib PROPERTY IMPORTED_LOCATION /opt/lib/libeORL.so)
	
	#add Gazebo's plugin as dynamic library 
	add_library(${PROJECT_NAME} SHARED 
		src/RobotMovementComauLib.cpp
		src/ComauGazeboPlugin.cpp 
	)
	target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_libraries} ComauLib ReflexxesLib)

	#compile NoGazebo node
	add_executable(no_gazebo 
		src/RobotMovementComauLib.cpp
		src/NoGazebo.cpp
	)
	target_link_libraries(no_gazebo ${catkin_LIBRARIES} ComauLib ReflexxesLib)

else()
	#add Gazebo's plugin as dynamic library 
	add_library(${PROJECT_NAME} SHARED 
		src/RobotMovementPID.cpp
		#src/RobotMovementComauLib.cpp
		src/ComauGazeboPlugin.cpp 
	)
	target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_libraries})

	#compile NoGazebo node
	add_executable(no_gazebo 
		src/RobotMovementPID.cpp
		src/NoGazebo.cpp
	)
	target_link_libraries(no_gazebo ${catkin_LIBRARIES})
endif()


#compile tcp_server node
add_executable(tcp_server
    src/TCPServer.cpp
)
target_link_libraries(tcp_server ${catkin_LIBRARIES})
