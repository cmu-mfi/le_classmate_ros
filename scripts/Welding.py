"""
@file Welding.py
@brief High-level control interface for laser and arc welding operations over Fanuc robots using
       the *comet_rpc* protocol (for the Lincoln Electric Classmate system)

This module defines a :class:`~Welder` helper that wraps the low-level I/O calls made available by
*comet_rpc* and exposes convenience methods that handle common timing, sequencing and safety
checks required to:

* Arm / disarm laser source.
* Start / stop laser emission.
* Start / stop welding with ArcTool (wire feed + gas).
* Individually toggle shielding-gas flow.

All timing-related constants (timeouts, polling rate and inter-signal delays) are configurable at
class level and can be tuned depending on the process hardware.

@authors Ankit Aggarwal <ankitagg@andrew.cmu.edu>
@date    July 2025
"""

import comet_rpc as rpc 
import time

class Welder():

    """High-level convenience wrapper around *comet_rpc* Digital/Analog I/O calls.

    The class assumes that the Fanuc robot controller is running the *comet_rpc* server and that the
    digital / analog signal mapping matches the table documented in the project README.  No attempt
    is made to validate the mapping at runtime — ensure that the I/O numbers below correspond to
    your cell wiring before running in production.

    All parameters are stored as instance attributes so they may be inspected or modified after
    creation (e.g., via a GUI panel).

    @section Usage

    ```python
    with rpc.Server('192.168.2.151') as srv:
        welder = Welder(srv, laser_power_watts=900, weld_voltage=21, weld_current=180,
                        weld_wirefeed_speed=12)
        welder.laser_ready_arm()
        welder.laser_start_emit()
        # move robot …
        welder.laser_stop_emit()
        welder.laser_disarm()
    ```

    @note All *laser_…* methods implement their own basic error-handling and will automatically call
          :py:meth:`laser_disarm` when a timeout or fault is detected.
    """


    #: Maximum time (in seconds) to wait for an I/O‑based condition before bailing out.
    timeout: float = 15.

    #: Delay (in seconds) between consecutive state polls.
    poll_interval: float = 0.1

    def __init__(self, 
                 server : rpc.Server, 
                 laser_power_watts:int = 700, 
                 weld_voltage:float = 0., 
                 weld_current:float = 0., 
                 weld_wirefeed_speed:float = 0.) -> None:
        
        """Create a new :class:`Welder` bound to an active *comet_rpc* server.

        @param server               Connected :pyclass:`comet_rpc.Server` instance.
        @param laser_power_watts    Requested laser power in **W** .
        @param weld_voltage         Target welding voltage in **V**.
        @param weld_current         Target welding current in **A**.
        @param weld_wirefeed_speed  Wire-feed speed in **inch/min**.
        """
        
        self.server = server
        self.laser_power_watts = laser_power_watts
        self.weld_voltage = weld_voltage
        self.weld_current = weld_current
        self.weld_wirefeed_speed = weld_wirefeed_speed
        
    # ---------------------------------------------------------------------
    # Laser helpers
    # ---------------------------------------------------------------------

    def laser_ready_arm(self):

        """Prepare the laser for emission (arm).

        Sequence:
        1. Set GO[2] with requested power.
        2. Enable external control and lock safety door.
        3. Wait until *READY FOR EXTERNAL CONTROL* (DI[47]).
        4. Switch *SYSTEM ON* (DO[44]) → wait *STANDBY* (DI[45]).
        5. Toggle *EXTEN IN* (DO[42]) → wait *EMISSION* (DI[43]).
        6. Enable air-knife and fume extraction.

        Any timeout or fault detected along the way calls :py:meth:`laser_error`.
        """

        # Analog power request ---------------------------------------------------
        rpc.iovalset(self.server, rpc.IoType.GroupedOut, index=2, value=self.laser_power_watts) # LASER POWER WATTS

        # Safety interlocks ------------------------------------------------------
        if rpc.iovalrd(self.server, rpc.IoType.DigitalOut, 43).value == 0:
            self.laser_error("SYS FAULT-CHK ALARMS")
            return
        
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=43, value=0) # AIMING LASER ON
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=70, value=1) # DOOR LOCK

        # External Connection ----------------------------------------------------
        start_time = time.time()
        while time.time() - start_time < self.timeout: 
            if rpc.iovalrd(self.server, rpc.IoType.DigitalIn, index=47).value == 1: # READY FOR EXTERNAL CON
                break
            time.sleep(self.poll_interval)
        else:
            self.laser_error("LASER RDY TIMEOUT - external")
            return
        
        # Stand‑by ---------------------------------------------------------------
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
        
        # Emission ready ---------------------------------------------------------
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
        
        # Auxiliaries ------------------------------------------------------------
        time.sleep(0.05)
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=69, value=1) # AIR KNIFE   
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=56, value=1) # FUME EXTRACT
        print("Laser Ready and Armed")
        return

    def laser_start_emit(self):

        """Start laser emission (gating signal)."""

        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=41, value=1) # GATE IN/EMIT
        print("Laser Emission Started")
        return 

    def laser_stop_emit(self):

        """Stop laser emission and pulse *PRO_STOP* for process termination."""

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

        """Bring the laser back to a safe, completely disarmed state."""

        # Disable emission / external ctrl --------------------------------------
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=42, value=0) # EXTEN IN   
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=44, value=0) # SYSTEM ON
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=47, value=0) # EXT_CONTROL_ENABLE
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=41, value=0) # GATE IN/EMIT

        # Air‑knife purge --------------------------------------------------------
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=69, value=1) # AIR KNIFE
        time.sleep(3)
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=69, value=0) # AIR KNIFE

        # Fume extraction cooldown ---------------------------------------------
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=56, value=1) # FUME EXTRACT
        time.sleep(5)
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=56, value=0) # FUME EXTRACT

        # Reset auxiliaries ------------------------------------------------------
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=45, value=0) # PULSE/PROFILE_ENABLE 
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=59, value=0) # PRO_START
        for idx in range(49, 56):
            rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=idx, value=0)
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=70, value=0) # DOOR LOCK
        print("Laser Disarmed")
        return 
    
    def laser_error(self, msg):

        """Log *msg* and immediately attempt to disarm the laser safely."""

        if getattr(self, '_handling_laser_error', False):
            print("Recursive laser_error call prevented:", msg)
            return
        self._handling_laser_error = True
        print(msg)
        self.laser_disarm()
        self._handling_laser_error = False

    # ---------------------------------------------------------------------
    # Welding helpers
    # ---------------------------------------------------------------------

    def weld_start(self): 

        """Initiate welding: enable gas flow and start the *ArcTool* process.

        Checks DI[27] (gas fault) and DI[28] (wire fault) before attempting to start.  If either fault
        is active, the function aborts early.

        On success, waits until *ARC DETECT* (DI[25]) is observed; otherwise calls
        :py:meth:`weld_end` to shut everything down and returns.
        """

        if rpc.iovalrd(self.server, rpc.IoType.DigitalIn, index=27).value == 1:
            print("Gas Fault Detected")
            return
        elif rpc.iovalrd(self.server, rpc.IoType.DigitalIn, index=28).value == 1:
            print("Wire Fault Detected")
            return
        else: 
            # User may uncomment AO writes below once signals are mapped -----------
            # rpc.iovalset(self.server, rpc.IoType.AnalogOut, index=1, value=self.weld_voltage)
            # rpc.iovalset(self.server, rpc.IoType.AnalogOut, index=2, value=self.weld_current)
            # rpc.iovalset(self.server, rpc.IoType.AnalogOut, index=3, value=self.weld_wirefeed_speed)
            # rpc.iovalset(self.server, rpc.IoType.AnalogOut, index=4, value=self.weld_wirefeed_speed)
            # rpc.iovalset(self.server, rpc.IoType.AnalogOut, index=5, value=self.weld_wirefeed_speed)
            # rpc.iovalset(self.server, rpc.IoType.AnalogOut, index=6, value=self.weld_wirefeed_speed)
            rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=26, value=1) # GAS START
            time.sleep(0.5)
            rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=49, value=1) # WELD START 

            start_time = time.time()
            while time.time() - start_time < self.timeout: 
                if rpc.iovalrd(self.server, rpc.IoType.DigitalIn, index=25).value == 1: #ARC DETECT
                    break
                time.sleep(self.poll_interval)
            else:
                print("ARC NOT ESTABLISHED")
                self.weld_end()
                return
            return  
        
    def weld_end(self):
        
        """Terminate welding and stop shielding-gas flow."""

        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=49, value=0) # WELD START 
        time.sleep(0.5)
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=26, value=0) # GAS START
        print("Weld Ended")

    # ---------------------------------------------------------------------
    # Gas utilities (manual override)
    # ---------------------------------------------------------------------

    def gas_start(self):

        """Manually enable shielding-gas flow (without arc)."""

        if rpc.iovalrd(self.server, rpc.IoType.DigitalIn, index=27).value == 1:
            print("Gas Fault Detected")
            return
        else:
            rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=26, value=1) # GAS START
            time.sleep(0.5)

    def gas_end(self):

        """Stop shielding-gas flow."""

        time.sleep(0.5)
        rpc.iovalset(self.server, rpc.IoType.DigitalOut, index=26, value=0) # GAS START


# -----------------------------------------------------------------------------
# I/O MAPPING SUMMARY (Fanuc controller ⇄ Welder functions)
# -----------------------------------------------------------------------------

"""
DOUT[25] - Weld Start
DOUT[26] - Gas Start
DOUT[27] - Touch cmd (future feedback)
DOUT[28] - Inch forward
DOUT[29] - Inch backward
DOUT[30] - Feed forward
DOUT[31] - Feed backward

AIN[1] - Voltage
AIN[2] - Current
AIN[3] - Wire feed speed
AIN[4] - Motor Current

AOUT[1] - Weld Cmd 1 (Voltage → AIN1)
AOUT[2] - Weld Cmd 2 (Current → AIN2)
AOUT[3] - Weld Cmd 3 (Feed Speed → AIN3)
AOUT[4] - Weld Cmd 4 (Crater Fill - speculative)
AOUT[5] - Weld Cmd 5 (Burnback - speculative)
AOUT[6] - Weld Cmd 6 (Pulse Settings - speculative)

DIN[25] - Arc detect
DIN[26] - Touch detect
DIN[27] - Gas fault
DIN[28] - Wire fault
"""

