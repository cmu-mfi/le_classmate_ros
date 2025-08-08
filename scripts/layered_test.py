import rospy
import ezdxf 
import tf 
import math 
import numpy as np

from geometry_msgs.msg import Pose, PoseStamped
from fc_msgs.srv import ExecuteCartesianTrajectory, SetPose
from ezdxf.math import BoundingBox2d, Vec2
import comet_rpc as rpc
import time
from le_classmate_ros.Welder import Welder
from le_classmate_ros.srv import LaserArm, LaserEmit, Weld, SetIO
from std_srvs.srv import Trigger


FIXED_Z_1 = 0.403
FIXED_Z_2 = 0.405
FIXED_QUAT = (-0.707, 0 , -0.707, 0) 
FIXED_Y = 0.02

PointA_1 = PoseStamped()
PointA_1.pose.position.x, PointA_1.pose.position.y, PointA_1.pose.position.z = 0.51, FIXED_Y, FIXED_Z_1
PointA_1.pose.orientation.x, PointA_1.pose.orientation.y, PointA_1.pose.orientation.z, PointA_1.pose.orientation.w = FIXED_QUAT

PointB_1 = PoseStamped()
PointB_1.pose.position.x, PointB_1.pose.position.y, PointB_1.pose.position.z = 0.60, FIXED_Y,  FIXED_Z_1
PointB_1.pose.orientation.x, PointB_1.pose.orientation.y, PointB_1.pose.orientation.z, PointB_1.pose.orientation.w = FIXED_QUAT


PointA_2 = PoseStamped()
PointA_2.pose.position.x, PointA_2.pose.position.y, PointA_2.pose.position.z = 0.51, FIXED_Y,  FIXED_Z_2
PointA_2.pose.orientation.x, PointA_2.pose.orientation.y, PointA_2.pose.orientation.z, PointA_2.pose.orientation.w = FIXED_QUAT

PointB_2 = PoseStamped()
PointB_2.pose.position.x, PointB_2.pose.position.y, PointB_2.pose.position.z = 0.60, FIXED_Y, FIXED_Z_2
PointB_2.pose.orientation.x, PointB_2.pose.orientation.y, PointB_2.pose.orientation.z, PointB_2.pose.orientation.w = FIXED_QUAT

if __name__ == '__main__':

    rospy.init_node('dxf_trajectory')

    rospy.wait_for_service('/real/fc_set_pose')
    rospy.wait_for_service('/real/fc_execute_cartesian_trajectory_async')
    rospy.wait_for_service('/weld_start')
    rospy.wait_for_service('/weld_end')
    rospy.wait_for_service('/laser_emit_start')
    rospy.wait_for_service('/laser_emit_stop')
    rospy.wait_for_service('/laser_ready_arm')
    rospy.wait_for_service('/laser_disarm')
    rospy.wait_for_service('/set_io_value')
    rospy.wait_for_service('/set_override')


    set_pose = rospy.ServiceProxy('/real/fc_set_pose', SetPose)
    execTraj = rospy.ServiceProxy('/real/fc_execute_cartesian_trajectory_async', ExecuteCartesianTrajectory)
    weldOn = rospy.ServiceProxy('/weld_start', Weld)
    weldOff = rospy.ServiceProxy('/weld_end', Weld)
    LaserOn = rospy.ServiceProxy('/laser_emit_start', LaserEmit)
    LaserOff = rospy.ServiceProxy('/laser_emit_stop', LaserEmit)
    Laser_Arm = rospy.ServiceProxy('/laser_ready_arm', LaserArm)
    Laser_Disarm = rospy.ServiceProxy('/laser_disarm', LaserArm) 
    Set_IO = rospy.ServiceProxy('/set_io_value', SetIO)
    set_override = rospy.ServiceProxy('/set_override', Trigger)


    server = '192.168.2.151'
    welder = Welder(server=server)


    set_override(100) # Set override to 100
    Set_IO('Digital_OUT', 47, 1) # Enable external control

    set_pose(PointA_1.pose, '/base_link', 0.01, 0.1, 'PTP')

    Laser_Arm(True) # Arm the laser
    time.sleep(2)
    LaserOn(True) # Start laser emission
    weldOn(True) # Start welding

    set_pose(PointB_1.pose, '/base_link', 0.0075, 0.1, 'PTP')

    weldOff(True) # Stop welding
    LaserOff(True) # Stop laser emission

    set_pose(PointA_2.pose, '/base_link', 0.01, 0.1, 'PTP')

    LaserOn(True) # Start laser emission
    weldOn(True) # Start welding

    set_pose(PointB_2.pose, '/base_link', 0.001, 0.1, 'PTP')

    weldOff(True) # Stop welding
    set_pose(PointB_2.pose, '/base_link', 0.3, 0.1, 'PTP')
    LaserOff(True) # Stop laser emission
    Laser_Disarm(True) # Disarm the laser
