#!/usr/bin/env python3
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


# Currently, this script can process Lines and Polygons. 


DXF_FILE_PATH = "/root/ros1_ws/src/le_classmate_ros/data/MFI8.dxf"
FIXED_Z = 0.405
FIXED_QUAT = (0.707, 0 , 0.707, 0) 

# workspace bounds: lateral-800mm, front-600mm with centre at 0.55
target_length_x = 0.6
target_length_y = 0.8

#home pose 
home = PoseStamped()
home.pose.position.x, home.pose.position.y, home.pose.position.z = 0.55, 0, 0.805
home.pose.orientation.x, home.pose.orientation.y, home.pose.orientation.z, home.pose.orientation.w = 0.707, 0, 0.707, 0

def transform_to_centre(center_x, center_y, poses, scale) -> list:
    workspace_centre_x = 0.53
    workspace_centre_y = 0.01

    delta_x = center_x - workspace_centre_x
    delta_y = center_y - workspace_centre_y

    transform_matrix = np.array([[1, 0, 0, -delta_x], [0, 1, 0, -delta_y], [0, 0, 1, 0], [0, 0, 0, 1]])
    
    for pose in poses: 
        point = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, 1]
        new_point = np.dot(transform_matrix, point)
        pose.pose.position.x = new_point[0] * scale
        pose.pose.position.y = new_point[1] * scale
        pose.pose.position.z = new_point[2]

    return poses

def parse_dxf_to_poses(dxf_file, centred = True) -> list:
    doc = ezdxf.readfile(dxf_file)
    msp = doc.modelspace()

    poses = []
    pose_types = []

    for lwpolyline in msp.query('LWPOLYLINE'):
        for point in lwpolyline.get_points():
            x, y = point[0], point[1]
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = FIXED_Z
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = FIXED_QUAT
            poses.append(pose)
            pose_types.append('LWPOLYLINE')

    for line in msp.query("LINE"):
        for pt in [line.dxf.start, line.dxf.end]:
            pose = PoseStamped()
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            pose.pose.position.z = FIXED_Z
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = FIXED_QUAT
            poses.append(pose)
            pose_types.append('LINE')
    
    for polyline in msp.query("POLYLINE"):
        for vertex in polyline.vertices:
            x, y, z = vertex.dxf.location.x, vertex.dxf.location.y, FIXED_Z
            pose = PoseStamped()                                                                       
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = FIXED_QUAT
            poses.append(pose)
            pose_types.append('POLYLINE')


    for mtext in msp.query("MTEXT"):
        pose = PoseStamped()
        pose.pose.position.x = mtext.dxf.insert[0]
        pose.pose.position.y = mtext.dxf.insert[1]
        pose.pose.position.z = FIXED_Z
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = FIXED_QUAT
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

def calculate_pose_error(pose1: Pose, pose2: Pose):
    dx = pose1.pose.position.x - pose2.pose.position.x
    dy = pose1.pose.position.y - pose2.pose.position.y
    dz = pose1.pose.position.z - pose2.pose.position.z
    position_error = math.sqrt(dx*dx + dy*dy + dz*dz)

    q1 = [pose1.pose.orientation.w, pose1.pose.orientation.x, pose1.pose.orientation.y, pose1.pose.orientation.z]
    q2 = [pose2.pose.orientation.w, pose2.pose.orientation.x, pose2.pose.orientation.y, pose2.pose.orientation.z]

    norm = lambda q: math.sqrt(sum(x*x for x in q))
    q1 = [x / norm(q1) for x in q1]
    q2 = [x / norm(q2) for x in q2]

    dot = sum(a*b for a, b in zip(q1, q2))
    dot = max(-1.0, min(1.0, dot))

    orientation_error = 2.0 * math.acos(abs(dot))

    return position_error, orientation_error

def monitor_pose_callback(msg, callback_args):
    combined_targets = callback_args
    pos_tolerance = 0.001

    for i,(idx, target_pose, action) in enumerate(combined_targets):
        previous_done = (i == 0) or (combined_targets[i-1][0] in found_targets)
        distance, _ = calculate_pose_error(msg, target_pose)
        if distance < pos_tolerance and idx not in found_targets and previous_done:
            found_targets.add(idx)
            if action == "off":
                print(f"Target {idx} is within tolerance, turning OFF laser")
                LaserOff(True)
                weldOff(True)
            else:
                print(f"Target {idx} is within tolerance, turning ON laser")
                LaserOn(True)
                weldOn(True)


if __name__ == '__main__':

    rospy.init_node('dxf_trajectory')
    rospy.wait_for_service('/real/fc_set_pose')
    set_pose = rospy.ServiceProxy('/real/fc_set_pose', SetPose)


    server = '192.168.2.151'
    welder = Welder(server=server)
    poses = parse_dxf_to_poses(DXF_FILE_PATH, True)
    found_targets = set()

    new_poses = []
    for pose in poses:
        new_poses.append(pose.pose)
    response_null = set_pose(new_poses[0], '/base_link', 0.3, 0.1, 'PTP')

    with open('/root/ros1_ws/src/le_classmate_ros/data/poses.txt', 'w') as f:
        for index, pose in enumerate(new_poses):
            f.write(f"{index} - {pose.position.x}, {pose.position.y}, {pose.position.z}\n")


    
    to_monitor_off_indices = [4,7,9]
    to_monitor_off = [poses[i] for i in to_monitor_off_indices]
    to_monitor_on_indices = [5,8,10]
    to_monitor_on = [poses[i] for i in to_monitor_on_indices]

    combined_targets = []
    for idx in to_monitor_off_indices:
        combined_targets.append( (idx, poses[idx], "off") )

    for idx in to_monitor_on_indices:
        combined_targets.append( (idx, poses[idx], "on") )
    combined_targets.sort(key=lambda x: x[0])

    rospy.wait_for_service('/real/fc_execute_cartesian_trajectory')
    execTraj = rospy.ServiceProxy('/real/fc_execute_cartesian_trajectory', ExecuteCartesianTrajectory)

    
    rospy.Subscriber('/real/tool0_pose', PoseStamped, monitor_pose_callback, callback_args=(combined_targets))


    rospy.wait_for_service('/weld_start')
    rospy.wait_for_service('/weld_end')
    rospy.wait_for_service('/laser_emit_start')
    rospy.wait_for_service('/laser_emit_stop')

    weldOn = rospy.ServiceProxy('/weld_start', Weld)
    weldOff = rospy.ServiceProxy('/weld_end', Weld)
    LaserOn = rospy.ServiceProxy('/laser_emit_start', LaserEmit)
    LaserOff = rospy.ServiceProxy('/laser_emit_stop', LaserEmit)


    rpc.vmip_writeva(server, "*SYSTEM*", "$MCR.$GENOVERRIDE", value=100)
    rpc.iovalset(server, rpc.IoType.DigitalOut, index=47, value=1)
    rpc.iovalset(server, rpc.IoType.DigitalOut, index=43, value=0)


    welder.laser_ready_arm()
    time.sleep(2)
    welder.laser_start_emit()
    welder.weld_start()
    response = execTraj(new_poses, 0.01, 0.0, 0.05, 0.1, 0.0)

    welder.weld_end()
    response_null = set_pose(new_poses[-1], '/base_link', 0.3, 0.1, 'PTP')
    time.sleep(5)
    welder.laser_stop_emit()
    welder.laser_disarm()


    # FUME EXTRACT
    rpc.iovalset(server, rpc.IoType.DigitalOut, index=56, value=1) # FUME EXTRACT
    time.sleep(10)
    rpc.iovalset(server, rpc.IoType.DigitalOut, index=56, value=0) # FUME EXTRACT