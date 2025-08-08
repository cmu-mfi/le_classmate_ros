# DXF Trajectory Execution with Laser & Welding Integration

The ![dxf_script.py](scripts/dxf_script.py) is a complete script to perform laser welding using a dxf. The .dxf file must be contained within the workspace bounds (600mm x 800mm). The script also has functionality to scale and center the dxf to parse (x,y) coordinates for robot poses. The script is written using only ROS services to control peripherals. The code is explained below:

This tutorial walks through loading a DXF file, parsing it into robot poses, and executing a Cartesian welding trajectory using ROS1. Laser and weld control are also integrated via services.

---

## 1. Prerequisites

Install required Python packages:

```
pip install ezdxf numpy
```

Ensure the following ROS services are available and active:

- `/real/fc_set_pose`
- `/real/fc_execute_cartesian_trajectory_async`
- `/weld_start`, `/weld_end`
- `/laser_emit_start`, `/laser_emit_stop`
- `/laser_ready_arm`, `/laser_disarm`
- `/set_io_value`

---

## 2. Load and Parse the DXF File

```python
import ezdxf
from geometry_msgs.msg import PoseStamped

DXF_FILE_PATH = "/root/ros1_ws/src/le_classmate_ros/data/MFI8.dxf"
FIXED_Z = 0.405
FIXED_QUAT = (0.707, 0, 0.707, 0)

def parse_dxf_to_poses(dxf_file) -> list:
    doc = ezdxf.readfile(dxf_file)
    msp = doc.modelspace()
    poses = []

    for line in msp.query("LINE"):
        for pt in [line.dxf.start, line.dxf.end]:
            pose = PoseStamped()
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            pose.pose.position.z = FIXED_Z
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = FIXED_QUAT
            poses.append(pose)
    return poses
```

This function reads a DXF file and extracts `LINE` entities, converting them into 3D ROS poses with a fixed height and orientation.

---

## 3. Center and Scale the Path

```python
import numpy as np

def transform_to_centre(center_x, center_y, poses, scale):
    delta_x = center_x - 0.53
    delta_y = center_y - 0.01
    transform_matrix = np.array([[1, 0, 0, -delta_x], [0, 1, 0, -delta_y], [0, 0, 1, 0], [0, 0, 0, 1]])

    for pose in poses:
        p = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, 1]
        p_new = np.dot(transform_matrix, p)
        pose.pose.position.x = p_new[0] * scale
        pose.pose.position.y = p_new[1] * scale
    return poses
```

This function recenters the poses around a fixed workspace origin and scales them to fit the robot's reach.

---

## 4. Monitor Tool Pose and Trigger IO

```python
import math

def calculate_pose_error(pose1, pose2):
    dx = pose1.pose.position.x - pose2.pose.position.x
    dy = pose1.pose.position.y - pose2.pose.position.y
    dz = pose1.pose.position.z - pose2.pose.position.z
    return math.sqrt(dx**2 + dy**2 + dz**2), 0.0  # Orientation skipped for simplicity

def monitor_pose_callback(msg, targets):
    for i, (idx, target_pose, action) in enumerate(targets):
        if idx in found_targets: continue
        prev_ok = (i == 0) or (targets[i-1][0] in found_targets)
        error, _ = calculate_pose_error(msg, target_pose)
        if error < 0.001 and prev_ok:
            found_targets.add(idx)
            LaserOn(True) if action == "on" else LaserOff(True)
            weldOn(True) if action == "on" else weldOff(True)
```

The `monitor_pose_callback` is a subscriber callback that triggers welding and laser IO actions when the robot's pose reaches specified targets within tolerance.

---

## 5. Initialize ROS Node and Services

```python
import rospy
from fc_msgs.srv import ExecuteCartesianTrajectory, SetPose, SetIO
from le_classmate_ros.srv import LaserEmit, Weld, LaserArm
from geometry_msgs.msg import PoseStamped
import comet_rpc as rpc
import time

rospy.init_node('dxf_trajectory')

# Wait for required services
rospy.wait_for_service('/real/fc_set_pose')
rospy.wait_for_service('/real/fc_execute_cartesian_trajectory_async')
rospy.wait_for_service('/weld_start')
rospy.wait_for_service('/weld_end')
rospy.wait_for_service('/laser_emit_start')
rospy.wait_for_service('/laser_emit_stop')
rospy.wait_for_service('/set_io_value')
rospy.wait_for_service('/laser_ready_arm')
rospy.wait_for_service('/laser_disarm')
rospy.wait_for_service('/set_override')

# Create service proxies
set_pose = rospy.ServiceProxy('/real/fc_set_pose', SetPose)
execTraj = rospy.ServiceProxy('/real/fc_execute_cartesian_trajectory_async', ExecuteCartesianTrajectory)
weldOn = rospy.ServiceProxy('/weld_start', Weld)
weldOff = rospy.ServiceProxy('/weld_end', Weld)
LaserOn = rospy.ServiceProxy('/laser_emit_start', LaserEmit)
LaserOff = rospy.ServiceProxy('/laser_emit_stop', LaserEmit)
Set_IO = rospy.ServiceProxy('/set_io_value', SetIO)
Laser_Arm = rospy.ServiceProxy('/laser_ready_arm', LaserArm)
Laser_Disarm = rospy.ServiceProxy('/laser_disarm', LaserArm)
set_override = rospy.ServiceProxy('/set_override', Trigger)
```

This block initializes your ROS node and service clients needed to control the welding process.

---

## 6. Setup Monitoring, Execute Trajectory

```python
poses = parse_dxf_to_poses(DXF_FILE_PATH)
found_targets = set()

# Save poses to file for debugging
with open('/root/ros1_ws/src/le_classmate_ros/data/poses.txt', 'w') as f:
    for i, p in enumerate(poses):
        f.write(f"{i} - {p.pose.position.x}, {p.pose.position.y}, {p.pose.position.z}\n")

# Monitor only certain indices
to_monitor_off_indices = [4,7,9]
to_monitor_on_indices = [5,8,10]

combined_targets = [(i, poses[i], "off") for i in to_monitor_off_indices] +                    [(i, poses[i], "on") for i in to_monitor_on_indices]
combined_targets.sort()

rospy.Subscriber('/real/tool0_pose', PoseStamped, monitor_pose_callback, callback_args=(combined_targets,))
```

This section selects key trajectory points to monitor and binds them to the callback for IO control.

---

## 7. Begin Motion Execution

```python
# Move to starting pose
set_pose(poses[0].pose, '/base_link', 0.3, 0.1, 'PTP')

# Setup welding systems
set_override(100) # Ensure override is set to 100
Set_IO('Digital_OUT', 47, 1)
Laser_Arm(True)
time.sleep(2)
LaserOn(True)
weldOn(True)

# Execute trajectory
execTraj([p.pose for p in poses], 0.01, 0.0, 0.01, 0.01, 0.0)

# Shut down
weldOff(True)
set_pose(poses[-1].pose, '/base_link', 0.3, 0.1, 'PTP')
LaserOff(True)
Laser_Disarm(True)
```

Finally, the robot moves through the full trajectory, with laser/weld triggered as it hits waypoints. After execution, the system is shut down cleanly.

---

## Summary

- Parse DXF into `PoseStamped` waypoints
- Scale/center into workspace
- Monitor robot motion and trigger actions using `Subscriber`
- Execute path with real-time IO via `ServiceProxy`

This pattern can be reused for painting, welding, inspection, and more.

---
