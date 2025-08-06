from fanuc_ros1.le_classmate_ros.src.Welding import Welder
import comet_rpc as rpc
import time



if __name__ == '__main__':
    
    # Setting up the server
    server = '192.168.2.151'
    welder = Welder(server=server)

    # Setting overwrite to 100
    rpc.vmip_writeva(server, "*SYSTEM*", "$MCR.$GENOVERRIDE", value=100)

    # Enabling external control
    rpc.iovalset(server, rpc.IoType.DigitalOut, index=47, value=1)

    # Arming the Laser
    welder.laser_ready_arm()

    # Starting Laser Emission
    welder.laser_start_emit()

    # Starting welding (robot must be in motion when this state is active)
    welder.weld_start()

    # Move the robot here using your preferred method
    
    # Stopping welding
    welder.weld_end()

    # Stopping Laser Emission
    welder.laser_stop_emit()

    # Disarming the Laser
    welder.laser_disarm()

    

 
