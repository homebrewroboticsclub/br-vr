cmake_minimum_required(VERSION 3.0.2)
project(teleop_fetch)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  message_generation
  ainex_interfaces
  ros_robot_controller
  rospy_x402
  rostest
)

add_service_files(
  FILES
  ReceiveGrant.srv
  EndSession.srv
  SetPeaqDatasetClaim.srv
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs sensor_msgs std_msgs std_srvs message_runtime ainex_interfaces ros_robot_controller rospy_x402
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
  scripts/teleop_node.py
  scripts/pose_source_node.py
  scripts/calibration_node.py
  scripts/vr_remapper_node.py
  scripts/dataset_recorder_node.py
  scripts/dataset_upload_server.py
  scripts/dataset_web_server.py
  test/test_teleop_kyr_arm_stream.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/ launch/ web/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(tests/test_teleop_state_contract.py)
  catkin_add_nosetests(tests/test_operator_buttons.py)
  catkin_add_nosetests(tests/test_upload_payload_session_fields.py)
  add_rostest(test/teleop_kyr_arm_stream.test)
endif()
