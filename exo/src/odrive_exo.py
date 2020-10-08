#!/usr/bin/env python
import time
import odrive
from odrive.enums import *
import odrive.utils
import math
from math_conversion import *
import fibre
import rospy
from std_msgs.msg import String, Float32

LIVEPLOTTER_POSITION_ESTIMATE = 0
LIVEPLOTTER_VELOCITY_ESTIMATE = 1
LIVEPLOTTER_TORQUE_ESTIMATE = 2
LIVEPLOTTER_CURRENT_COMMANDED = 3
LIVEPLOTTER_CURRENT_MEASURED = 4

class odrive_exo():
    """Implementation of exoskeleton "joint" using odrive

    Atributes:
        my_drive: odrive object, see https://docs.odriverobotics.com/commands
        for commands/documentation.
    """
    encoder_cpr = 8192
    encoder_kv = 192

    last_speed = 0 # TODO: Do we want these
    last_pos = 0


    def __init__(self):
        """
        Make launch files so that we can say which axis is to use, if odrive
        should be calibrated on startup, ...
        """
        print("Finding an odrive...")
        self.odrv = odrive.find_any()
        print("Found an odrive...")

        if self.odrv:
            self.axis0 = self.odrv.axis0
            self.axis1 = self.odrv.axis1
            self.encoder_cpr = self.axis0.encoder.config.cpr
            # TODO: Determine and set these
            #self.ARM_LOWER_LIMIT = 0
            #self.ARM_UPPER_LIMIT = 8192

        self.TERMpub = rospy.Publisher("odrive_info", String, queue_size = 10)
        # TODO: til PID is integrated this is not needed
        # rospy.Subscriber("pid_channel", Float32, odrive.pid_callback)
        self.PIDpub = rospy.Publisher("odrive_channel", Float32, queue_size=10) # not sure what queue_size should be
        rospy.Subscriber("term_channel", String, odrv.term_callback)
        rospy.Subscriber("imu_input", Float32, odrv.imu_callback)

    def term_callback(self, data):
        cmd_data = data.data.strip()
        rospy.loginfo(cmd_data)
        cmd_length = len(cmd_data)
        response = -1

        cmd = cmd_data[0]
        args = cmd_data[1:].strip()

        print("Got command: {} with data: {}".format(cmd, args))

        if (cmd == "p"):
            response = self.set_motor_position(args)
        elif (cmd == "c"):
            response = self.start_calibration(args)
        elif (cmd == "l"):
            response = self.set_limit(args)
        elif (cmd == "f"):
            response = self.print_motor_config(args)
        elif (cmd == "e"):
            response = self.print_error(args)
        else:
            rospy.loginfo("Undefined error")

    def PID_callback(self, data):
        pid_val = data.data
        rospy.login(pid_val)

    def imu_callback(self, data):
        imu_val = data.data
        rospy.loginfo(imu_val)

    def auto_calibrate(self, debug):
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
        print("Reseting Configuration - Recalibration Required")
        self.odrv.erase_configuration()
        self.reboot_odrive()

        print("Calibration Phase 1")
        # These 3 values are the ones we had to change from default, they will
        # change with different power supplies:
        #   Garage Power Supply: 3,3,5
        #   Turnigy ReaktorPro: 10, 10, 3 (Defaults)
        #
        self.axis0.motor.config.calibration_current = 10
        self.axis0.motor.config.current_lim = 10
        self.axis0.motor.config.resistance_calib_max_voltage = 3
        self.axis0.controller.config.vel_limit = 20000

        # Display the changed parameters if in debug mode
        if debug:
            print("Calibration current set to " + str(self.axis0.motor.config.calibration_current))
            print("Current limit set to " + str(self.axis0.motor.config.current_lim))
            print("Resistance_calib_max_voltage set to " + str(self.axis0.motor.config.resistance_calib_max_voltage))

        print("Calibration Phase 1 Complete.")
        '''
        At this point setting self.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        should make the motor Beep, then spin one direction, then the other.
        Running dump_errors(odrv0) in the odrivetool should show no errors.
        '''

        self.set_requested_state(self.axis0, AXIS_STATE_FULL_CALIBRATION_SEQUENCE)
        time.sleep(20)

        #####################Calibration Phase 2 - Encoder######################
        print("\nPhase 2\n")
        self.axis0.encoder.config.use_index = True # Done
        self.set_requested_state(self.axis0, AXIS_STATE_ENCODER_INDEX_SEARCH)
        time.sleep(10)
        self.set_requested_state(self.axis0, AXIS_STATE_ENCODER_OFFSET_CALIBRATION) # Error seems to happen here
        time.sleep(10)
        #
        if debug:
            # Should be 0
            print("axis.error = " + str(self.axis0.error) + " expected = 0")
            # Should be some integer, like -326 or 1364
            print("offset = " + str(self.axis0.encoder.config.offset) + " expected = Integer (+ or -)")
            # Should be 1 or -1
            print("direction = " + str(self.axis0.motor.config.direction) + " expected = -1 or +1" )

        print("Setting encoder and motor pre-calibrated flags to True...The variable printed below should be true if the odrive agrees calibration worked")
        self.axis0.encoder.config.pre_calibrated = True
        self.axis0.motor.config.pre_calibrated = True
        # Search for an index at startup, i.e. encoder should spin and stop at the same position
        self.axis0.config.startup_encoder_index_search = False
        print("self.axis0.encoder.config.pre_calibrated = ")
        print(self.axis0.encoder.config.pre_calibrated)

        if self.axis0.encoder.config.pre_calibrated:
            self.odrv.save_configuration()
            self.reboot_odrive()
            print("Done full calibration sequence - successful")
        else:
            print("Done full calibration sequence - not successful")

    def user_calibrate(self):
        """
        This calibration is completely user driven, only sets values and then
        allows the user to freely move arm until the index is found
        - MUST do the above full calibration sequence at least once before this
        will work. i.e. run calibrate() with motor disconnected from arm, power
        down odrive, connect arm and restart odrive, run user_calibrate and
        manually rotate motor through the index position (marked with red line
        on motor)
        """
        # These 3 values are the ones we had to change from default, they will
        # change with different power supplies:
        #   Garage Power Supply: 3,3,5
        #   Turnigy ReaktorPro: 10, 10, 3 (Defaults)
        self.axis0.motor.config.calibration_current = 10
        self.axis0.motor.config.current_lim = 10
        self.axis0.motor.config.resistance_calib_max_voltage = 3
        self.axis0.controller.config.vel_limit = 20000
        self.odrv.save_configuration()
        self.reboot_odrive()
        print("\nIndex Values Before: " + str(self.axis0.encoder.is_ready) + " " + str(self.axis0.encoder.index_found))
        print("Delaying 10 seconds for manual calibration movement")
        time.sleep(10)
        print("\nIndex Values After: " + str(self.axis0.encoder.is_ready) + " " + str(self.axis0.encoder.index_found))

    def reboot_odrive(self):
        """<Brief Description>

        <Detailed Description>

        Args:

        Returns:

        Raises:

        """
        print("Rebooting and Reconnecting")
        try:
            self.odrv.reboot()
        except fibre.ChannelBrokenException:
            print("Expected connection loss due to reboot")
        self.odrv = odrive.find_any()
        if self.odrv:
            self.axis0 = self.odrv.axis0
            self.axis1 = self.odrv.axis1
        print("Reconnection Successful")

    def dump_errors(self, clear=False):
        # Figure out the path to call functions from utils.py
        odrive.utils.dump_errors(self.odrv, clear)

    def liveplotter(self, plot):

        if plot == 0:
            odrive.utils.start_liveplotter(self.axis0.encoder.pos_estimate)

        if plot == 1:
            odrive.utils.start_liveplotter(self.axis0.encoder.vel_estimate)

        if plot == 2:
            odrive.utils.start_liveplotter(self.axis0.motor.current_control.Iq_setpoint)

        if plot == 3:
            odrive.utils.start_liveplotter(self.axis0.motor.current_control.Iq_measured)

        if plot == 4:
            # INCOMPLETE: Figure out the KV for our motor
            odrive.utils.start_liveplotter(self.axis0.motor.current_control.Iq_measured*8.27/self.encoder_kv)

    def dump_motor_config(self):
        """
        TODO
        """
        print(self.axis0.motor.config)

    def dump_encoder_config(self):
        """
        TODO
        """
        print(self.axis0.encoder.config)

    def set_global_current_limit(self, new_current_lim):
        print("Setting global current limit to " + str(new_current_lim))
        self.axis0.motor.config.current_lim = new_current_lim

    def get_global_current_limit(self):
        return self.axis0.motor.config.current_lim

    def set_global_velocity_limit(self, new_velocity_lim):
        print("Setting global velocity limit to " + str(new_velocity,_lim))
        self.axis0.controller.config.vel_limit = new_velocity_lim

    def get_global_velocity_limit(self):
        return self.axis0.controller.config.vel_limit

    def set_control_mode(self, axis, mode):
        """
        CTRL_MODE_VOLTAGE_CONTROL = 0
        CTRL_MODE_CURRENT_CONTROL = 1
        CTRL_MODE_VELOCITY_CONTROL = 2
        CTRL_MODE_POSITION_CONTROL = 3
        CTRL_MODE_TRAJECTORY_CONTROL = 4
        """
        if mode in range(6):
            axis.controller.config.control_mode = mode
        else:
            print("Error: Invalid control mode")

    def set_requested_state(self, axis, state):
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
            axis.requested_state = state
        else:
            print("Error: Invalid requested state")

    def sinusoidal_test_move(self, verbose):
        """Do a sinusoidal test movement for 10 seconds

        See https://github.com/madcowswe/ODrive/blob/master/tools/odrive_demo.py

        Args:
            verbose: True to display position each time

        """
        self.requested_state(self.axis0, AXIS_STATE_CLOSED_LOOP_CONTROL)
        self.axis0.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
        print("Executing sinusoidal test move...")
        t0 = time.monotonic()
        while (time.monotonic() - t0) < 10:
            setpoint = 10000.0 * math.sin((time.monotonic() - t0)*2)
            if verbose:
                print("goto " + str(int(setpoint)))
            self.axis0.controller.pos_setpoint = setpoint
            time.sleep(0.01)

    def set_position(self, position):
        odrv.set_requested_state(self.axis0, AXIS_STATE_CLOSED_LOOP_CONTROL)
        odrv.axis0.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
        odrv.axis0.controller.pos_setpoint = position
        print("Set position to: " + str(position) + " encoder units")

    def set_position_trajectory_control(target_vel, accel_limit, decel_limit):
        pass

    def set_position_circular_control():
        pass

    def set_velocity(self, velocity):
        self.set_requested_state(self.axis0, AXIS_STATE_CLOSED_LOOP_CONTROL)
        self.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
        # Set the velocity [counts/s]
        self.axis0.controller.vel_setpoint = velocity
        print("Set Velocity to: " + velocity + "counts/s (Non-Ramped)")

    def set_velocity_ramped(self, velocity, ramp_rate):
        self.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
        # Set the velocity ramp rate (acceleration) [counts/s^2]
        self.axis0.controller.config.vel_ramp_rate = ramp_rate
        # Activate ramped velocty mode
        self.axis0.controller.vel_ramp_enable = True
        # Set the target velocity to be ramped to
        self.axis0.controller.vel_ramp_target = velocity
        print("Set Velocity to: " + velocity + " counts/s , Ramp-Rate: " + ramp_rate + " counts/s^2")

    def set_current_control(self, current):
        self.set_requested_state(self.axis0, AXIS_STATE_CLOSED_LOOP_CONTROL)
        self.axis0.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL
        # Set the current, proportional to torque
        # NOTE: There is no velocity limiting in current control mode, make
        # sure to not overrev the motor or exceed max speed for encode.
        # Uncomment below if you dare :)
        if (current >= 0 and current <= 23):
            print("Setting Current")
            self.axis0.controller.current_setpoint = current
        else:
            print("Error: Current value of outside of range")

    def set_voltage_control(self, current):
        self.axis0.controller.config.control_mode = CTRL_MODE_VOLTAGE_CONTROL

    def get_position(self):
        return self.axis0.encoder.pos_estimate

    def get_velocity(self):
        return self.axis0.encoder.vel_estimate

    ################################ ROS - Main ################################

    def set_motor_position(self, cmd):
        odrv.set_position(int(cmd))
        self.TERMpub.publish("cp" + str(odrv.get_position()))
        self.TERMpub.publish("tp" + str(odrv.axis0.controller.pos_setpoint))


    def start_calibration(self, arg):
        # TODO: only do full calibration if not pre_calibrated
        if (arg == "f"):
            odrv.auto_calibrate(True)
        if (arg == "u"):
            odrv.user_calibrate()

    def set_limit(self, cmd):
        cmd_code = cmd[0]
        cmd_data = cmd[1:]
        if (cmd_code == "v"):
            odrv.set_global_velocity_limit(int(cmd_data))
            # TODO: why is this being set twice
            #rospy.loginfo(odrive.set_global_velocity_limit(int(cmd_data)))
            return 0
        elif (cmd_code == "c"):
            odrv.set_global_current_limit(int(cmd_data))
            # TODO: why is this being set twice
            #rospy.loginfo(odrive.set_global_current_limit(int(cmd_data)))
            return 0
        else:
            rospy.loginfo("Invalid input")
            return -1

    def print_motor_config(self, cmd):
        if (cmd == "m"):
            odrv.dump_motor_config()
            return 0
        elif (cmd == "e"):
            odrv.dump_encoder_config()
            return 0
        else:
            rospy.loginfo("Invalid input")
            return -1

    def print_error(self, cmd):
        odrv.dump_errors()
        return 0

    def loop(self):
        angle_position_val = convert_counts_to_angle(self.get_position())
        self.PIDpub.publish(angle_position_val)


if __name__ == "__main__":
    rospy.init_node("odriv_node", anonymous=True)
    odrv = odrive_exo()
    while (True):
        odrv.loop()
        rospy.spin()
    
