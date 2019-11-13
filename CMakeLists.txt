cmake_minimum_required(VERSION 2.8.3)
project(controllino_control)

find_package(catkin REQUIRED COMPONENTS
  rosserial_arduino
  rosserial_client
)


rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)

rosserial_configure_client(
  DIRECTORY firmware
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

# Do not just add this line to your CMakeLists.txt, modify the existing line
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

#add_service_files(
#  FILES
#  SetDigitalSignal.srv
#)

generate_messages(
	DEPENDENCIES
	)

catkin_package(
	CATKIN_DEPENDS roscpp std_msgs message_runtime
	)


#rosserial_add_client_target(firmware hello ALL)
#rosserial_add_client_target(firmware hello-upload)
