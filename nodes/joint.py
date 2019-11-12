#!/usr/bin/env python
import time
import odrive
import odrive.enums
import odrive.utils
import math
import fibre

AXIS_STATE_UNDEFINED = 0
AXIS_STATE_IDLE = 1
AXIS_STATE_STARTUP_SEQUENCE = 2
AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3
AXIS_STATE_MOTOR_CALIBRATION = 4
AXIS_STATE_SENSORLESS_CONTROL = 5
AXIS_STATE_ENCODER_INDEX_SEARCH = 6
AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7
AXIS_STATE_CLOSED_LOOP_CONTROL = 8
AXIS_STATE_LOCKIN_SPIN = 9
AXIS_STATE_ENCODER_DIR_FIND = 10

CTRL_MODE_VOLTAGE_CONTROL = 0
CTRL_MODE_CURRENT_CONTROL = 1
CTRL_MODE_VELOCITY_CONTROL = 2
CTRL_MODE_POSITION_CONTROL = 3
CTRL_MODE_TRAJECTORY_CONTROL = 4

LIVEPLOTTER_POSITION_ESTIMATE = 0
LIVEPLOTTER_VELOCITY_ESTIMATE = 1
LIVEPLOTTER_TORQUE_ESTIMATE = 2
LIVEPLOTTER_CURRENT_COMMANDED = 3
LIVEPLOTTER_CURRENT_MEASURED = 4



class Joint:
    """Implementation of exoskeleton "joint" using odrive

    Atributes:
        my_drive: odrive object, see https://docs.odriverobotics.com/commands
        for commands/documentation.
    """

    def __init__(self):

        print("Finding an odrive...")
        self.my_drive = odrive.find_any()

    def calibrate(self, debug):
        """<Brief Description>

        <Detailed Description>

        Args:

        Returns:

        Raises:

        """

        ##################Calibration Phase 1 - Set Variables###################
        '''
        Based off https://docs.odriverobotics.com/
        '''
        self.reset()

        print("Calibration Phase 1")
        # These 3 values are the ones we had to change from default, they will
        # change with different power supplies:
        #   Garage Power Supply: 3,3,5
        #   Turnigy ReaktorPro: 10, 10, 3 (Defaults)
        #
        self.my_drive.axis0.motor.config.calibration_current = 10
        self.my_drive.axis0.motor.config.current_lim = 10
        self.my_drive.axis0.motor.config.resistance_calib_max_voltage = 3
        self.my_drive.axis0.controller.config.vel_limit = 60000

        # Display the changed parameters if in debug mode
        if debug:
            print("Calibration current set to " + str(self.my_drive.axis0.motor.config.calibration_current))
            print("Current limit set to " + str(self.my_drive.axis0.motor.config.current_lim))
            print("Resistance_calib_max_voltage set to " + str(self.my_drive.axis0.motor.config.resistance_calib_max_voltage))

        print("Calibration Phase 1 Complete.")
        '''
        At this point setting self.my_drive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        should make the motor Beep, then spin one direction, then the other.
        Running dump_errors(odrv0) in the odrivetool should show no errors.
        '''

        self.my_drive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        time.sleep(20)

        #####################Calibration Phase 2 - Encoder######################
        print("\nPhase 2\n")
        self.my_drive.axis0.encoder.config.use_index = True # Done
        self.my_drive.axis0.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
        time.sleep(10)
        self.my_drive.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION # Error seems to happen here
        time.sleep(10)
        #
        if debug:
            # Should be 0
            print("axis.error = " + str(self.my_drive.axis0.error) + " expected = 0")
            # Should be some integer, like -326 or 1364
            print("offset = " + str(self.my_drive.axis0.encoder.config.offset) + " expected = Integer (+ or -)")
            # Should be 1 or -1
            print("direction = " + str(self.my_drive.axis0.motor.config.direction) + " expected = -1 or +1" )

        print("Setting encoder and motor pre-calibrated flags to True...The variable printed below should be true if the odrive agrees calibration worked")
        self.my_drive.axis0.encoder.config.pre_calibrated = True
        self.my_drive.axis0.motor.config.pre_calibrated = True
        # Search for an index at startup, i.e. encoder should spin and stop at the same position
        self.my_drive.axis0.config.startup_encoder_index_search = True
        print("self.my_drive.axis0.encoder.config.pre_calibrated = ")
        print(self.my_drive.axis0.encoder.config.pre_calibrated)

        if self.my_drive.axis0.encoder.config.pre_calibrated:
            self.my_drive.save_configuration()
            self.reboot()
            print("Done full calibration sequence - successful")
        else:
            print("Done full calibration sequence - not successful")

    def reset(self):
        """<Brief Description>Erase current configuration settings

        Requires recalibration, this is called at start of every calibration
        sequence.

        Args: None

        Returns: None

        Raises: None

        """
        print("Reseting Configuration - Recalibration Required")
        self.my_drive.erase_configuration()
        self.reboot()

    def reboot(self):
        """<Brief Description>

        <Detailed Description>

        Args:

        Returns:

        Raises:

        """
        print("Rebooting and Reconnecting")
        try:
            self.my_drive.reboot()
        except fibre.ChannelBrokenException:
            print("Expected connection loss due to reboot")
        self.my_drive = odrive.find_any()
        print("Reconnection Successful")

    def dump_errors(self, clear=False):
        # Figure out the path to call functions from utils.py
        odrive.utils.dump_errors(self.my_drive)

    def liveplotter(self, plot):

        if plot == 0:
            odrive.utils.start_liveplotter(self.my_drive.axis0.encoder.pos_estimate)

        if plot == 1:
            odrive.utils.start_liveplotter(self.my_drive.axis0.encoder.vel_estimate)

        if plot == 2:
            odrive.utils.start_liveplotter(self.my_drive.axis0.motor.current_control.Iq_setpoint)

        if plot == 3:
            odrive.utils.start_liveplotter(self.my_drive.axis0.motor.current_control.Iq_measured)

        if plot == 4:
            # INCOMPLETE: Figure out the KV for our motor
            odrive.utils.start_liveplotter(self.my_drive.axis0.motor.current_control.Iq_measured*8.27/KV)

    def dump_motor_config(self):
        """
        TODO
        """

    def dump_encoder_config(self):
        """
        TODO
        """

    def set_global_current_limit(self, new_current_lim):
        """<Brief Description>

        <Detailed Description>

        Args:

        Returns:

        Raises:

        """
        self.my_drive.axis0.motor.config.current_lim = new_current_lim

    def set_global_velocity_limit(self, new_velocity_lim):
        """<Brief Description>

        <Detailed Description>

        Args:

        Returns:

        Raises:

        """
        self.my_drive.axis0.controller.config.vel_limit = new_velocity_lim

    def get_global_current_limit(self):
        """<Brief Description>

        <Detailed Description>

        Args:

        Returns:

        Raises:

        """
        return self.my_drive.axis0.motor.config.current_lim

    def get_global_velocity_limit(self):
        """<Brief Description>

        <Detailed Description>

        Args:

        Returns:

        Raises:

        """
        return self.my_drive.axis0.controller.config.vel_limit

    def set_control_mode(self, mode):
        """
        CTRL_MODE_VOLTAGE_CONTROL = 0
        CTRL_MODE_CURRENT_CONTROL = 1
        CTRL_MODE_VELOCITY_CONTROL = 2
        CTRL_MODE_POSITION_CONTROL = 3
        CTRL_MODE_TRAJECTORY_CONTROL = 4
        """
        if mode in range(6):
            self.my_drive.axis0.controller.config.control_mode = mode
        else:
            print("Error: Invalid control mode")

    def set_requested_state(self, state):
        """
        AXIS_STATE_UNDEFINED = 0
        AXIS_STATE_IDLE = 1
        AXIS_STATE_STARTUP_SEQUENCE = 2
        AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3
        AXIS_STATE_MOTOR_CALIBRATION = 4
        AXIS_STATE_SENSORLESS_CONTROL = 5
        AXIS_STATE_ENCODER_INDEX_SEARCH = 6
        AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7
        AXIS_STATE_CLOSED_LOOP_CONTROL = 8
        AXIS_STATE_LOCKIN_SPIN = 9
        AXIS_STATE_ENCODER_DIR_FIND = 10
        """
        if state in range(11):
            self.my_drive.axis0.requested_state = state
        else:
            print("Error: Invalid requested state")

    def sinusoidal_test_move(self, verbose):
        """Do a sinusoidal test movement for 10 seconds

        See https://github.com/madcowswe/ODrive/blob/master/tools/odrive_demo.py

        Args:
            verbose: True to display position each time

        Returns:

        Raises:

        """
        self.my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.my_drive.axis0.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
        print("Executing sinusoidal test move...")
        t0 = time.monotonic()
        while (time.monotonic() - t0) < 10:
            setpoint = 10000.0 * math.sin((time.monotonic() - t0)*2)
            if verbose:
                print("goto " + str(int(setpoint)))
            self.my_drive.axis0.controller.pos_setpoint = setpoint
            time.sleep(0.01)

    def set_position(self, position):
        r"""Set the position of motor, position is in encoder units.

        <Detailed Description>

        Args:
            Position: double from <INSERT RANGE>, units = encoder units

        Returns:
            None

        Raises:
            None
        """
        self.my_drive.axis0.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
        self.my_drive.axis0.controller.pos_setpoint = position
        print("Set position to: " + position + " encoder units")

    def set_position_trajectory_control(target_vel, accel_limit, decel_limit):
        """TODO

        https://docs.odriverobotics.com/#trajectory-control

        Args:

        Returns:

        Raises:

        """
        pass

    def set_position_circular_control():
        """TODO

        See https://docs.odriverobotics.com/#trajectory-control

        Args:

        Returns:

        Raises:

        """
        pass

    def set_velocity(self, velocity):
        """Non-ramped velocity control

        UNTESTED

        Args:

        Returns:

        Raises:

        """
        self.my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.my_drive.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
        # Set the velocity [counts/s]
        self.my_drive.axis0.controller.vel_setpoint = velocity
        print("Set Velocity to: " + velocity + "counts/s (Non-Ramped)")

    def set_velocity_ramped(self, velocity, ramp_rate):
        """Ramped velocity control i.e. accelerate to velocity at ramp rate

        UNTESTED

        Args:

        Returns:

        Raises:

        """
        self.my_drive.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
        # Set the velocity ramp rate (acceleration) [counts/s^2]
        self.my_drive.axis0.controller.config.vel_ramp_rate = ramp_rate
        # Activate ramped velocty mode
        self.my_drive.axis0.controller.vel_ramp_enable = True
        # Set the target velocity to be ramped to
        self.my_drive.axis0.controller.vel_ramp_target = velocity
        print("Set Velocity to: " + velocity + " counts/s , Ramp-Rate: " + ramp_rate + " counts/s^2")

    def set_current_control(self, current):
        """UNTESTED<Brief Description>

        <Detailed Description>

        Args:

        Returns:

        Raises:

        """
        self.my_drive.axis0.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL
        # Set the current, proportional to torque
        # NOTE: There is no velocity limiting in current control mode, make
        # sure to not overrev the motor or exceed max speed for encode.
        # Uncomment below if you dare :)
        #self.my_drive.axis0.controller.current_setpoint = current

    def set_voltage_control(self, current):
        """UNTESTED<Brief Description>

        <Detailed Description>

        Args:

        Returns:

        Raises:

        """
        self.my_drive.axis0.controller.config.control_mode = CTRL_MODE_VOLTAGE_CONTROL

    def get_position(self):
        """<Brief Description>

        <Detailed Description>

        Args:

        Returns:

        Raises:

        """
        return self.my_drive.axis0.encoder.pos_estimate

    def get_velocity(self):
        """<Brief Description>

        <Detailed Description>

        Args:

        Returns:

        Raises:

        """
        return self.my_drive.axis0.encoder.vel_estimate

    def check_calibration_values(self):
        """<Brief Description>

        <Detailed Description>

        Args:

        Returns:

        Raises:

        """
        print("Checking Calibration Values")

        print("Velocity set to " + str(self.my_drive.axis0.controller.config.vel_limit))
        print("Current set to " + str(self.my_drive.axis0.motor.config.current_lim))
        print("Encoder ser to " + str(self.my_drive.axis0.encoder.config.cpr))

        '''
        Encoder Calibration Values
        '''

        '''
        Values indicating succesful calibration (or at least partially succesful):
        Error: Sould be 0
        Offset: Some positive or negative integer
        Direction: 1 or -1
        '''
        print("my_drive.axis.error = " + str(self.my_drive.axis0.error))
        print("my_drive.axis0.encoder.config.offset = " + str(self.my_drive.axis0.encoder.config.offset))
        print("my_drive.axis0.motor.config.direction = " + str(self.my_drive.axis0.motor.config.direction))
        print("Expected Value: error = 0, offset = Integer (+ or -), direction = -1 or 1.")

        '''
        End of calibration values
        '''
        print("my_drive.axis0.encoder.config.pre_calibrated=" + self.my_drive.axis0.encoder.config.pre_calibrated)
        print("my_drive.axis0.config.startup_encoder_index_search=" + self.my_drive.axis0.config.startup_encoder_index_search)
        print("my_drive.axis0.motor.config.pre_calibrated=" + self.my_drive.axis0.motor.config.pre_calibrated)
        print("my_drive.axis0.requested_state=" + self.my_drive.axis0.requested_state)
