#!/usr/bin/env python3

import comet_rpc as rpc 
import time

'''
laser_... functions control the laser 
weld_... functions control the ArcTool (Wire Feed)
'''

class Welder():

    timeout = 15 # seconds 
    poll_interval = 0.1 #seconds  

    def __init__(self, 
                 server, 
                 laser_power_watts = 900, 
                 weld_voltage = 0, 
                 weld_current = 0, 
                 weld_wirefeed_speed = 0):
        
        self.server = server
        self.laser_power_watts = laser_power_watts
        self.weld_voltage = weld_voltage
        self.weld_current = weld_current
        self.weld_wirefeed_speed = weld_wirefeed_speed
        rpc.vmip_writeva(server, "*SYSTEM*", "$MCR.$GENOVERRIDE", value=100)
        rpc.iovalset(server, rpc.IoType.DigitalOut, index=47, value=1)

        

    def laser_ready_arm(self):
        rpc.iovalset(self.server, rpc.IoType.GroupedOut, index=2, value=self.laser_power_watts) # LASER POWER WATTS

        if rpc.iovalrd(self.server, rpc.IoType.UserOpPanelOut, 6).value == 1:
            self.laser_error("SYS FAULT-CHK ALARMS")
            return
        
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=43, value=0) # AIMING LASER ON
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=70, value=1) # DOOR LOCK
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=47, value=1) # EXT_CONTROL_ENABLE

        start_time = time.time()
        while time.time() - start_time < self.timeout: 
            if rpc.iovalrd(self.server, rpc.IoType.DigitalIn, index=47).value == 1: # READY FOR EXTERNAL CON
                break
            time.sleep(self.poll_interval)
        else:
            self.laser_error("LASER RDY TIMEOUT - external")
            return
            
        time.sleep(0.05)
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=44, value=1) # SYSTEM ON

        start_time = time.time()
        while time.time() - start_time < self.timeout: 
            if rpc.iovalrd(self.server, rpc.IoType.DigitalIn, index=45).value == 1: # LASER IN STANDBY STATE
                break
            time.sleep(self.poll_interval)
        else:
            self.laser_error("LASER RDY TIMEOUT - standby")
            return
            
        time.sleep(0.05)
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=42, value=1) # EXTEN IN   

        start_time = time.time()
        while time.time() - start_time < self.timeout: 
            if rpc.iovalrd(self.server, rpc.IoType.DigitalIn, index=43).value == 1: # LASER IN EMISSION STAT
                break
            time.sleep(self.poll_interval)
        else:
            self.laser_error("LASER RDY TIMEOUT - emission")
            return   
             
        time.sleep(0.05)
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=69, value=1) # AIR KNIFE   
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=56, value=1) # FUME EXTRACT
        print("Laser Ready and Armed")
        return

    def laser_start_emit(self):
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=41, value=1) # GATE IN/EMIT
        print("Laser Emission Started")
        return 

    def laser_stop_emit(self):
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=41, value=0) # GATE IN/EMIT
        
        # Pulsing DO[58] PRO_STOP
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=58, value=1) # PRO_STOP
        time.sleep(0.1)
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=58, value=0) # PRO_STOP

        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=59, value=0) # PRO_START
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=57, value=0) # SYNC IN

        print("Laser Emission Stopped")
        return 

    def laser_disarm(self):
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=42, value=0) # EXTEN IN   
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=44, value=0) # SYSTEM ON
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=47, value=0) # EXT_CONTROL_ENABLE
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=41, value=0) # GATE IN/EMIT

        #Pulsing DO[69] AIR KNIFE
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=69, value=1) # AIR KNIFE
        time.sleep(3)
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=69, value=0) # AIR KNIFE

        #Pulsing DO[56] FUME EXTRACT
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=56, value=1) # FUME EXTRACT
        time.sleep(5)
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=56, value=0) # FUME EXTRACT

        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=45, value=0) # PULSE/PROFILE_ENABLE 
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=49, value=0) # PRO_B1
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=59, value=0) # PRO_START
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=70, value=0) # DOOR LOCK

        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=49, value=0) # PRO_B1
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=50, value=0) # PRO_B2
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=51, value=0) # PRO_B3
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=52, value=0) # PRO_B4
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=53, value=0) # PRO_B5
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=54, value=0) # PRO_B6
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=55, value=0) # PRO_B7
        
        print("Laser Disarmed")
        return 
    
    def laser_error(self, msg):
        if getattr(self, '_handling_laser_error', False):
            print("Recursive laser_error call prevented:", msg)
            return
        self._handling_laser_error = True
        print(msg)
        self.laser_disarm()
        self._handling_laser_error = False

    def weld_start(self): 
        # Also starts gas
        if rpc.iovalrd(self.server, rpc.IoType.DigitalIn, index=27).value == 1:
            print("Gas Fault Detected")
            return
        elif rpc.iovalrd(self.server, rpc.IoType.DigitalIn, index=28).value == 1:
            print("Wire Fault Detected")
            return
        else: 

            rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=20, value=1)
            return  
        
    def weld_end(self):

        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=21, value=1)
        print("Weld Ended")

    def gas_start(self):
        if rpc.iovalrd(self.server, rpc.IoType.DigitalIn, index=27).value == 1:
            print("Gas Fault Detected")
            return
        else:
            rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=26, value=1) # GAS START
            time.sleep(0.5)

    def gas_end(self):
        time.sleep(0.5)
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=26, value=0) # GAS START

    def set_override(self, value=100):
        rpc.vmip_writeva(self.server, "*SYSTEM*", "$MCR.$GENOVERRIDE", value=value)

'''
DOUT [25] - Weld Start 
DOUT [26] - Gas Start 
DOUT [27] - Touch cmd -- could be added if we want another check and/or want some feedback 
DOUT [28] - Inch forward 
DOUT [29] - Inch backward 
DOUT [30] - Feed forward 
DOUT [31] - Feed backward

AIN [1] - Voltage
AIN [2] - Current 
AIN [3] - Wire feed speed 
AIN [4] - Motor Current 

AOUT [1] - Weld Cmnd 1 - Typically Voltage (AIN 1)
AOUT [2] - Weld Cmmd 2 - Typically Current (AIN 2)
AOUT [3] - Weld Cmnd 3 - Typically Feed Speed (AIN 3)
AOUT [4] - Weld Cmnd 4 - Speculative: Crater FIll
AOUT [5] - Weld Cmnd 5 - Speculative: Burnback 
AOUT [6] - Weld Cmnd 6 - Speculative: Pulse Settings

DIN [25] - Arc detect      
DIN [26] - Touch detect   
DIN [27] - Gas fault       
DIN [28] - Wire fault      
'''