# ROS Welding Routine Explained

This Python script performs a simple two-pass laser welding operation using predefined poses and ROS service calls. Below is a breakdown of the key parts of the code.

---

## 1. **Imports and Constants**

```python
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
from le_classmate_ros.srv import LaserArm, LaserEmit, Weld, SetIO
```

These import necessary libraries for ROS control, DXF processing, math, and I/O. The services and message types are specific to a laser welding robot setup.

---

## 2. **Fixed Parameters and Pose Definitions**

```python
FIXED_Z_1 = 0.403
FIXED_Z_2 = 0.405
FIXED_QUAT = (-0.707, 0 , -0.707, 0) 
FIXED_Y = 0.02
```

We define fixed heights and orientations for the welding tool. Then we create 4 poses: two for the first weld path, and two for the second (slightly offset in Z).

```python
PointA_1 = PoseStamped()
PointA_1.pose.position.x, PointA_1.pose.position.y, PointA_1.pose.position.z = 0.51, FIXED_Y, FIXED_Z_1
PointA_1.pose.orientation.x, PointA_1.pose.orientation.y, PointA_1.pose.orientation.z, PointA_1.pose.orientation.w = FIXED_QUAT
```

This is repeated similarly for `PointB_1`, `PointA_2`, `PointB_2`.

---

## 3. **Main Execution Block**

```python
if __name__ == '__main__':
    rospy.init_node('dxf_trajectory')
```

We initialize the ROS node and wait for all required services to be available.

### 4. **Service Clients Setup**

```python
set_pose = rospy.ServiceProxy('/real/fc_set_pose', SetPose)
execTraj = rospy.ServiceProxy('/real/fc_execute_cartesian_trajectory_async', ExecuteCartesianTrajectory)
...
```

These are the clients used to command robot motion, welding, and laser behavior.

### 5. **Welder Initialization**

```python
server = '192.168.2.151'
welder = Welder(server=server)
rpc.vmip_writeva(server, "*SYSTEM*", "$MCR.$GENOVERRIDE", value=100)
_ = Set_IO('Digital_OUT', 47, 1) # Enable external control
```

We initialize the Fanuc welder and set necessary flags to enable external control.

---

## 6. **Welding Routine**

### First Pass:

```python
_ = set_pose(PointA_1.pose, '/base_link', 0.01, 0.1, 'PTP')
_ = Laser_Arm(True)
time.sleep(2)
_ = LaserOn(True)
_ = weldOn(True)
_ = set_pose(PointB_1.pose, '/base_link', 0.0075, 0.1, 'PTP')
_ = weldOff(True)
_ = LaserOff(True)
```

We move to start position, arm the laser, begin welding, move to the end point, and stop.

### Second Pass:

```python
_ = set_pose(PointA_2.pose, '/base_link', 0.01, 0.1, 'PTP')
_ = LaserOn(True)
_ = weldOn(True)
_ = set_pose(PointB_2.pose, '/base_link', 0.001, 0.1, 'PTP')
_ = weldOff(True)
_ = set_pose(PointB_2.pose, '/base_link', 0.3, 0.1, 'PTP')  # Move away
_ = LaserOff(True)
_ = Laser_Disarm(True)
```

Same as the first pass, but slightly higher in Z (a "second layer" weld).

---

## 7. Summary

This code executes a controlled laser weld routine along two horizontal lines using ROS services. Each phase—arming, emitting, welding, and disarming—is explicitly timed and ordered.

To extend this:
- Add more poses to trace complex geometries.
- Convert DXF lines to pose sequences.
- Add feedback/error handling for real deployments.