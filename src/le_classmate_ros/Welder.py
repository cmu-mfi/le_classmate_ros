#!/usr/bin/env python3
"""
@file Welder.py
@brief ROS service interface for welding operations using the Welding utility class

This node provides a ROS service interface to control:
- Laser arming/disarming
- Laser emission start/stop
- Welding process start/stop

The node wraps the Welding.Welder class functionality into ROS services for integration
with other ROS nodes. All services return boolean success states.

@section Services
- /weld_start : Initiate welding process
- /weld_end : Terminate welding process
- /laser_arm : Prepare laser for emission
- /laser_disarm : Safely disable laser
- /laser_emit_start : Begin laser emission
- /laser_emit_stop : Stop laser emission

@note All services return Response objects with a State field (bool) indicating success
@warning Requires properly configured comet_rpc server connection
"""
import Welding
import rospy 
import comet_rpc as rpc
from le_classmate_ros.srv import Weld, WeldResponse, LaserArm, LaserArmResponse, LaserEmit, LaserEmitResponse

class Welder_Node():
    """ROS service wrapper for Welding operations
    
    Provides service interfaces to control welding and laser operations through
    the Welding utility class. Maintains connection to comet_rpc server.
    
    @section Usage
    ```python
    rospy.init_node('welder_node')
    welder_node = Welder_Node()
    rospy.spin()
    ```
    """

    def __init__(self):
        """Initialize ROS services and Welding utility
        
        Reads server IP from ROS parameter '~server' (default: 192.168.2.151)
        Creates all service endpoints and maintains Welding.Welder instance
        """
        rospy.loginfo("Welder Node Started")
        server_ip = rospy.get_param('~server', '192.168.2.151')
        self.welder_util = Welding.Welder(server=server_ip)

        rospy.Service('weld_start', Weld, self.weld_start_srv)
        rospy.Service('weld_end', Weld, self.weld_end_srv)
        rospy.Service('laser_arm', LaserArm, self.laser_arm_srv)
        rospy.Service('laser_disarm', LaserArm, self.laser_disarm_srv)
        rospy.Service('laser_emit_start', LaserEmit, self.laser_emit_start_srv)
        rospy.Service('laser_emit_stop', LaserEmit, self.laser_emit_stop_srv)

    def weld_start_srv(self, request):
        """Service handler for weld start request
        
        @param request: Empty Weld service request
        @return: WeldResponse with State=True on success
        """
        self.welder_util.weld_start()
        return WeldResponse(State=True)

    def weld_end_srv(self, request):
        """Service handler for weld end request
        
        @param request: Empty Weld service request
        @return: WeldResponse with State=True on success
        """
        self.welder_util.weld_end()
        return WeldResponse(State=True)

    def laser_arm_srv(self, request):
        """Service handler for laser arming request
        
        @param request: Empty LaserArm service request
        @return: LaserArmResponse with State=True on success
        """
        self.welder_util.laser_ready_arm()
        return LaserArmResponse(State=True)

    def laser_disarm_srv(self, request):
        """Service handler for laser disarming request
        
        @param request: Empty LaserArm service request
        @return: LaserArmResponse with State=True on success
        """
        self.welder_util.laser_disarm()
        return LaserArmResponse(State=True)

    def laser_emit_start_srv(self, request):
        """Service handler for laser emission start
        
        @param request: Empty LaserEmit service request
        @return: LaserEmitResponse with State=True on success
        """
        self.welder_util.laser_start_emit()
        return LaserEmitResponse(State=True)

    def laser_emit_stop_srv(self, request):
        """Service handler for laser emission stop
        
        @param request: Empty LaserEmit service request
        @return: LaserEmitResponse with State=False (normal stop condition)
        """
        self.welder_util.laser_stop_emit()
        return LaserEmitResponse(State=False)

if __name__ == "__main__":
    """Main execution block for welder node
    
    Initializes ROS node and enters control loop at 50Hz
    """
    rospy.init_node('welder_node')
    welder_node = Welder_Node()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        rate.sleep()