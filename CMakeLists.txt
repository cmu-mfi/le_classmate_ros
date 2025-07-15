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
)

catkin_package(
    CATKIN_DEPENDS fc_msgs geometry_msgs roscpp rospy
)

catkin_install_python(
  PROGRAMS
    dxf_script.py
    io_test.py
    test_script.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


