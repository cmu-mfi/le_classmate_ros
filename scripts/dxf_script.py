#!/usr/bin/env python3
import rospy
import ezdxf 
import tf 
import math 
import numpy as np

from geometry_msgs.msg import Pose 
from fc_msgs.srv import ExecuteCartesianTrajectory, SetPose
from ezdxf.math import BoundingBox2d, Vec2
import comet_rpc as rpc
import time
from le_classmate_ros.Welding import Welder
from le_classmate_ros.srv import LaserArm, LaserEmit, Weld


# Currently, this script can process Lines and Polygons. 


DXF_FILE_PATH = "/root/ros1_ws/src/fanuc_ros1/le_classmate_ros/data/MFI4.dxf"
FIXED_Z = 0.4
FIXED_QUAT = (-1, 0 , 0, 0)  # Quaternion representing a 180-degree rotation around the x-axis

# workspace bounds: lateral-800mm, front-600mm with centre at 0.55
target_length_x = 0.6
target_length_y = 0.8

#home pose 
home = Pose()
home.position.x, home.position.y, home.position.z = 0.55, 0, 0.805
home.orientation.x, home.orientation.y, home.orientation.z, home.orientation.w = 0.707, 0, 0.707, 0

def transform_to_centre(center_x, center_y, poses, scale) -> list:
    workspace_centre_x = 0.55
    workspace_centre_y = 0.0

    delta_x = center_x - workspace_centre_x
    delta_y = center_y - workspace_centre_y

    transform_matrix = np.array([[1, 0, 0, -delta_x], [0, 1, 0, -delta_y], [0, 0, 1, 0], [0, 0, 0, 1]])
    
    for pose in poses: 
        point = [pose.position.x, pose.position.y, pose.position.z, 1]
        new_point = np.dot(transform_matrix, point)
        pose.position.x = new_point[0] * scale
        pose.position.y = new_point[1] * scale
        pose.position.z = new_point[2]

    return poses

def parse_dxf_to_poses(dxf_file, centred = True) -> list:
    doc = ezdxf.readfile(dxf_file)
    msp = doc.modelspace()

    poses = []
    pose_types = []

    for lwpolyline in msp.query('LWPOLYLINE'):
        for point in lwpolyline.get_points():
            x, y = point[0], point[1]
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = FIXED_Z
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = FIXED_QUAT
            poses.append(pose)
            pose_types.append('LWPOLYLINE')

    for line in msp.query("LINE"):
        for pt in [line.dxf.start, line.dxf.end]:
            pose = Pose()
            pose.position.x = pt[0]
            pose.position.y = pt[1]
            pose.position.z = FIXED_Z
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = FIXED_QUAT
            poses.append(pose)
            pose_types.append('LINE')
    
    for polyline in msp.query("POLYLINE"):
        for vertex in polyline.vertices:
            x, y, z = vertex.dxf.location.x, vertex.dxf.location.y, FIXED_Z
            pose = Pose()                                                                       
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = FIXED_QUAT
            poses.append(pose)
            pose_types.append('POLYLINE')


    for mtext in msp.query("MTEXT"):
        pose = Pose()
        pose.position.x = mtext.dxf.insert[0]
        pose.position.y = mtext.dxf.insert[1]
        pose.position.z = FIXED_Z
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = FIXED_QUAT
        poses.append(pose)
        pose_types.append('MTEXT')
    
    points=[]
    for entity in msp: 
        try: 
            if entity.dxftype() == "LINE":
                points.append(Vec2(entity.dxf.start))
                points.append(Vec2(entity.dxf.end))

            elif entity.dxftype() == "POLYLINE":
                for v in entity.vertices:
                    loc = v.dxf.location
                    points.append(Vec2(loc.x, loc.y))

            elif entity.dxftype() == "MTEXT":
                loc = entity.dxf.insert
                points.append(Vec2(loc.x, loc.y))

            elif entity.dxftype() == "LWPOLYLINE":
                for x, y, *_ in entity.get_points():
                    points.append(Vec2(x, y))


        except Exception as e:
            print("Unsupported")

    xs = [p.x for p in points]
    ys = [p.y for p in points]

    center_x = (min(xs) + max(xs)) / 2
    center_y = (min(ys) + max(ys)) / 2

    source_length_x = max(xs) - min(xs)
    source_length_y = max(ys) - min(ys)

    scale = 1.
    if source_length_x > target_length_x or source_length_y > target_length_y:
        scale = min(target_length_x/source_length_x, target_length_y/source_length_y)
        print('Scaling to fit into workspace with scaling factor - ', scale)

    print("source length: ", source_length_x, source_length_y)
    if centred == True:
        print("center: ", center_x, center_y)
        poses = transform_to_centre(center_x, center_y, poses, scale)    
    # poses.append(home)

    return poses 


def monitor_pose_callback(msg, to_monitor_off:list, to_monitor_on:list):

    pos_tolerance = 0.02
    current_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        
    for idx, target in enumerate(to_monitor_off):
        distance = np.linalg.norm(current_pos - target)
        if distance < pos_tolerance and idx not in found_targets:
            found_targets.add(idx)
            res = weldOff(True)
            res = LaserOff(True)

    for idx, target in enumerate(to_monitor_on):
        distance = np.linalg.norm(current_pos - target)
        if distance < pos_tolerance and idx not in found_targets:
            found_targets.add(idx)
            res = weldOn(True)
            res = LaserOn(True)



if __name__ == '__main__':


    poses = parse_dxf_to_poses(DXF_FILE_PATH, True)
    for pose in poses: 
        print(pose, "\n")

    to_monitor_off_indices = [4,7,9]
    to_monitor_off = [poses[i] for i in to_monitor_off_indices]
    to_monitor_on_indices = [5,8,10]
    to_monitor_on = [poses[i] for i in to_monitor_on_indices]

    rospy.init_node('dxf_trajectory')
    rospy.wait_for_service('/real/fc_execute_cartesian_trajectory')
    execTraj = rospy.ServiceProxy('/real/fc_execute_cartesian_trajectory', ExecuteCartesianTrajectory)

    rospy.wait_for_service('/real/fc_set_pose')
    set_pose = rospy.ServiceProxy('/real/fc_set_pose', SetPose)
    rospy.Subscriber('/real/tool0_pose', Pose, monitor_pose_callback, callback_args=(to_monitor_off, to_monitor_on))


    rospy.wait_for_service('/weld_start')
    rospy.wait_for_service('/weld_end')
    rospy.wait_for_service('/laser_emit_start')
    rospy.wait_for_service('/laser_emit_stop')

    weldOn = rospy.ServiceProxy('/weld_start', Weld)
    weldOff = rospy.ServiceProxy('/weld_end', Weld)
    LaserOn = rospy.ServiceProxy('/laser_emit_start', LaserEmit)
    LaserOff = rospy.ServiceProxy('/laser_emit_stop', LaserEmit)
    found_targets = set()


    server = '192.168.2.151'
    welder = Welder(server=server)

    rpc.vmip_writeva(server, "*SYSTEM*", "$MCR.$GENOVERRIDE", value=100)
    rpc.iovalset(server, rpc.IoType.DigitalOut, index=47, value=1)
    rpc.iovalset(server, rpc.IoType.DigitalOut, index=43, value=0)


    response_null = set_pose(poses[0], '/real/base_link', 0.1, 0.1, 'PTP')

    welder.laser_ready_arm()
    time.sleep(2)
    welder.laser_start_emit()
    rpc.iovalset(server, rpc.IoType.DigitalOut, index=20, value=1)
    response = execTraj(poses, 0.01, 0.0, 0.5, 0.03, 0.0)

    rpc.iovalset(server, rpc.IoType.DigitalOut, index=21, value=1)
    response_null = set_pose(poses[-1], '/real/base_link', 0.1, 0.1, 'PTP')
    # time.sleep(5)
    welder.laser_stop_emit()
    welder.laser_disarm()

    # time.sleep(10)
        # rpc.iovalset(server, rpc.IoType.DigitalOut, index=21, value=1)
    # response_null = set_pose(poses[-1], '/real/base_link', 0.1, 0.1, 'PTP')
