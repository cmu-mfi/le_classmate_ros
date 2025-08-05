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
from le_classmate_ros.Welding import Welder
from le_classmate_ros.srv import LaserArm, LaserEmit, Weld


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
    set_pose = rospy.ServiceProxy('/real/fc_set_pose', SetPose)

    server = '192.168.2.151'
    welder = Welder(server=server)

    rpc.vmip_writeva(server, "*SYSTEM*", "$MCR.$GENOVERRIDE", value=100)
    rpc.iovalset(server, rpc.IoType.DigitalOut, index=47, value=1)
    rpc.iovalset(server, rpc.IoType.DigitalOut, index=43, value=0)

    res = set_pose(PointA_1.pose, '/base_link', 0.01, 0.1, 'PTP')

    welder.laser_ready_arm()
    time.sleep(2)
    welder.laser_start_emit()
    welder.weld_start()

    res = set_pose(PointB_1.pose, '/base_link', 0.0075, 0.1, 'PTP')

    welder.weld_end()
    welder.laser_stop_emit()

    res = set_pose(PointA_2.pose, '/base_link', 0.01, 0.1, 'PTP')

    welder.laser_start_emit()
    welder.weld_start()

    res = set_pose(PointB_2.pose, '/base_link', 0.001, 0.1, 'PTP')

    welder.weld_end()
    welder.laser_stop_emit()
    response_null = set_pose(PointB_2.pose, '/base_link_link', 1, 1, 'PTP')
    welder.laser_disarm()