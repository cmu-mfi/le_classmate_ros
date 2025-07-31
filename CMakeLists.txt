cmake_minimum_required(VERSION 3.0.2)
project(le_classmate_ros)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    fc_msgs
    geometry_msgs
    roscpp
    fc_tasks
    message_generation
)
catkin_python_setup()

add_service_files(
    FILES
    Weld.srv
    LaserArm.srv
    LaserEmit.srv
)
generate_messages(
    DEPENDENCIES
    std_msgs
    fc_msgs
)

catkin_package(
    CATKIN_DEPENDS fc_msgs geometry_msgs roscpp rospy
)


install(DIRECTORY src/le_classmate_ros/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  PATTERN "*.py"
  PATTERN "__pycache__" EXCLUDE
)

catkin_install_python(PROGRAMS

  scripts/dxf_script.py
  scripts/test_script.py
  scripts/laser_run.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)




