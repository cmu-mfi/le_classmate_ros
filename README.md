# le_classmate_ros

## ROS Interface for Laser and Arc Welding on Lincoln Electric Classmate Cells

This package provides a ROS-compatible control layer for a Fanuc-driven Lincoln Electric Classmate welding cell. It enables service-based control of welding operations via `comet_rpc`, conversion of CAD geometries into robot trajectories, and execution of coordinated laser or arc weld paths.

---

## Features

- ROS service interface for laser and arc welding control
- DXF-based trajectory generation for weld paths
- Integration with `comet_rpc` for direct I/O-level control of Fanuc systems
- Custom weld macros support via LS files
- Launch file and service definitions for easy orchestration
- Standalone API for non-ROS workflows via the `Welder` class

---

## Directory Structure

```
le_classmate_ros/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── welder.launch                  # Launches the welder service node
├── scripts/
│   ├── dxf_script.py                  # Converts DXF to Cartesian poses and executes them
│   ├── test_script.py                 # Basic test for arc welding I/O
│   └── laser_run.py                   # ROS node providing weld + laser services
├── srv/
│   ├── Weld.srv                       # Service: Start/Stop welding
│   ├── LaserArm.srv                   # Service: Arm/Disarm laser
│   └── LaserEmit.srv                  # Service: Start/Stop laser emission
├── data/
│   └── rect2.dxf                      # Example DXF path input
├── ls/
│   └── ros_movesm.ls                  # Fanuc LS macro for ROS motion
├── src/
│   └── le_classmate_ros/
│       └── Welding.py                 # Core welder class wrapping comet_rpc I/O
```

---

## Dependencies

### ROS Packages
- `rospy`
- `roscpp`
- `geometry_msgs`
- `fc_msgs`
- `fc_tasks`

### Python Packages
- `comet_rpc` (https://github.com/gavanderhoorn/comet_rpc)
- `ezdxf`
- `numpy`

---

## Services

Provided by `laser_run.py` node:

| Service Name         | Type            | Description                          |
|----------------------|-----------------|--------------------------------------|
| `/weld_start`        | `Weld.srv`      | Start arc welding process            |
| `/weld_end`          | `Weld.srv`      | End arc welding process              |
| `/laser_arm`         | `LaserArm.srv`  | Prepare laser for emission           |
| `/laser_disarm`      | `LaserArm.srv`  | Disable laser safely                 |
| `/laser_emit_start`  | `LaserEmit.srv` | Begin laser emission                 |
| `/laser_emit_stop`   | `LaserEmit.srv` | Stop laser emission                  |

All services return a `bool State` field indicating success.

---

## Usage

### 1. Build

```bash
cd ~/ros1_ws
catkin_make
source devel/setup.bash
```

### 2. Launch Welder Node

```bash
roslaunch le_classmate_ros welder.launch
```

This launches `laser_run.py`, which exposes service interfaces for welding.

### 3. DXF Trajectory Execution

```bash
rosrun le_classmate_ros dxf_script.py
```

Reads `data/rect2.dxf`, transforms geometry to robot coordinates, and sends it to the controller via `fc_execute_cartesian_trajectory`.

### 4. Manual Welding Test

```bash
rosrun le_classmate_ros test_script.py
```

Runs a minimal weld start + end test using arc welding control.

---

## Using the `Welder` Class Without ROS

The `Welding.Welder` class can be used directly in standalone Python scripts to control welding I/O over RPC:

```python
from le_classmate_ros.Welding import Welder
import comet_rpc as rpc

with rpc.Server('192.168.2.151') as server:
    welder = Welder(server,
                    laser_power_watts=800,
                    weld_voltage=21,
                    weld_current=180,
                    weld_wirefeed_speed=12)

    # Prepare and start laser
    welder.laser_ready_arm()
    welder.laser_start_emit()

    # Move robot using your control stack...

    # Stop laser and disarm
    welder.laser_stop_emit()
    welder.laser_disarm()

    # Start arc weld
    welder.weld_start()
    # Wait for motion...
    welder.weld_end()
```

The class handles all relevant I/O mappings and safety interlocks.

---

## I/O Mapping Summary

| Type     | Index | Function                  |
|----------|-------|---------------------------|
| DOUT     | 25    | Weld Start                |
| DOUT     | 26    | Gas Start                 |
| DIN      | 25    | Arc Detect                |
| DIN      | 27    | Gas Fault                 |
| DIN      | 28    | Wire Fault                |
| AOUT     | 1–6   | Weld Cmds (voltage, etc.) |
| AIN      | 1–4   | Weld Feedback             |
| DOUT     | 20    | ArcTool Weld Start        |
| DOUT     | 21    | ArcTool Weld End          |

---

## Author

Ankit Aggarwal  
ankitagg@andrew.cmu.edu  
Carnegie Mellon University | Manufacturing Futures Institute

---
