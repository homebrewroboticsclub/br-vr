cmake_minimum_required(VERSION 3.0.2)
project(teleop_fetch)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
  std_msgs
  ainex_interfaces
  ros_robot_controller
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs sensor_msgs std_msgs ainex_interfaces ros_robot_controller
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
  scripts/teleop_node.py
  scripts/pose_source_node.py
  scripts/calibration_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/ launch/ web/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
