from fanuc_ros1.le_classmate_ros.src.Welding import Welder
import comet_rpc as rpc
import time

'''
def vmip_writeva(
    server: str, prog_name: str, var_name: str, value: t.Union[str, int, float]
) -> VmIpWriteVaResponse:
    """Write 'value' to the variable 'var_name' in program 'prog_name'.
    Set `prog_name` to `"*SYSTEM*"` to write to system variables.
    `value` will always be submitted as a string, even for (system) variables
    which are of a different type. `COMET` apparently tries to parse the
    string representation and converts it to the required type when possible.
    The string representations are expected to be identical to those found in
    `.VA` files.

    OVERRIDE is a system variable which can be set to a value between 0 and 100. It needs to be set to 100 to allow welding and movement.
    the prog_name is "*SYSTEM*" as override is a system variable

    THIS MUST BE DONE BEFORE EVERY PROGRAM RUN TO ENSURE THAT OVERRIDE IS SET TO 100. NOT DOING SO WILL RESULT IN AN ERROR WHEN TRYING TO START WELDING.
    The override value is set to 100 in the constructor of the Welder class, however, it still may need to be set again here. 
    Best practice is to always set it in every program.
'''


if __name__ == '__main__':
    
    # Setting up the server
    server = '192.168.2.151'
    welder = Welder(server=server)

    # Setting overwrite to 100 - refer to the docstring of vmip_writeva above for more information
    rpc.vmip_writeva(server, "*SYSTEM*", "$MCR.$GENOVERRIDE", value=100)

    # Enabling external control - i/o value that control the external control must be set to 1
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

    

 
