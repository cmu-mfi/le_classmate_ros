# le_classmate_ros

ROS Integration for Welding Automation on the Lincoln Electric Classmate using Fanuc + `comet_rpc`

## Overview

This package provides a ROS interface for controlling welding operations on a Lincoln Electric Classmate cell equipped with a Fanuc robot and a `comet_rpc` I/O server. It enables trajectory-driven laser and arc welding by:

- Parsing DXF files into Cartesian robot trajectories.
- Executing safe, high-level laser and welding control commands.
- Integrating analog/digital I/O over the `comet_rpc` protocol.
- Providing reusable helper classes and scripts to facilitate automation.

## Features

- Welding Control: High-level Python interface for laser and arc welding operations.
- DXF Parsing: Converts 2D CAD geometry to 3D Cartesian poses.
- ROS Services: Uses `fc_msgs` and `fc_tasks` for robot motion and welding coordination.
- RPC Integration: Full compatibility with Fanuc controllers via `comet_rpc`.

## Dependencies

Make sure the following packages are installed:

### ROS Dependencies
- rospy
- roscpp
- geometry_msgs
- fc_msgs
- fc_tasks

### Python Dependencies
- comet_rpc (https://github.com/gavanderhoorn/comet_rpc)
- ezdxf
- numpy

## Usage

### 1. Launch the ROS core and your robot stack
Ensure your robot controller and `comet_rpc` server are running. ROS services for trajectory execution should be available:
- {namespace}/fc_execute_cartesian_trajectory
- {namespace}/fc_set_pose

### 2. Run DXF-to-Trajectory Execution

```
rosrun le_classmate_ros dxf_script.py
```

This script parses a DXF file into poses, scales and centers the geometry to fit the robot workspace, and calls the appropriate trajectory service.

### 3. Weld Execution with Laser Control

```
rosrun le_classmate_ros io_test.py
```

This performs a full cycle:
- Arms the laser
- Starts emission
- Executes a weld trajectory
- Stops and disarms the laser

### 4. Test I/O Manually

```
rosrun le_classmate_ros test_script.py
```

A lightweight I/O toggle script for rapid verification and debugging.

## Welder Class (Python API)

The `Welder` class in `Welding.py` abstracts the I/O logic for:

### Laser Control
- `laser_ready_arm()`
- `laser_start_emit()`
- `laser_stop_emit()`
- `laser_disarm()`

### Arc Welding Control
- `weld_start()`
- `weld_end()`

### Shielding Gas Control
- `gas_start()`
- `gas_end()`

Refer to the inline docstrings or the source file for detailed logic and I/O mapping.


## Author

Ankit Aggarwal  
ankitagg@andrew.cmu.edu  
Carnegie Mellon University | Manufacturing Futures Institute

## License

TODO – Please update `package.xml` with the appropriate license before use.