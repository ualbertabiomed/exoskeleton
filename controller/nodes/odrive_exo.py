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

class ODrive_ROS:
    """
    ODrive ROS Node
    """

    # Declare variables
    odrv_ctrl = None
    TERMpub = None
    TERMsub = None

    def __init__(self):

        # Initialize ROS Node
        rospy.init_node("odrive_node", anonymous=True)

        # Initialize ODrive Control Object
        self.odrv_ctrl = ODrive_ctrl()

        # Initialize Publishers and Subscribers
        self.TERMsub = rospy.Subscriber("term_channel", String, self.term_callback)
        self.TERMpub = rospy.Publisher("odrive_info", String, queue_size = 10)

        # Try to connect to ODrive
        self.odrv_ctrl.connect()

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

    def user_loop(self):
        """
        For user control via menu
        """
        while (True):
            key = raw_input("\n1. Calibrate\n2. Position Control Mode\n3. Velocity Control Mode\n4. Voltage Control Mode\n5. Set Limits\n6. Dump Configs + Errors\nChoose a command (Press a number): ")

            if key == '1':
            # Calibration
                key = raw_input("\n1. Auto (Don't do this while motor is connect to arm)\n2. User\nChoose a calibration type: ")
                if key == '1':
                    # Don't do this while connected to arm
                    #self.odrv_ctrl.auto_calibrate(True)
                    pass
                if key == '2':
                    self.odrv_ctrl.user_calibrate()
            elif key == '2':
            # Position Control Mode
                key = ''
                while key != "q":
                    print("Current Position [counts]: " + str(self.odrv_ctrl.get_position()))
                    key = raw_input("Enter a position or press q to quit: ")
                    if key != 'q': self.odrv_ctrl.set_position(int(key))
                    # TODO: Possibly add a delay here to give time for motor to spin so position is read correctly on next loop

            elif key == '3':
            # Veclocity Control Mode
                key = ''
                while key != 'q':
                    print("Current Position [counts]: " + str(self.odrv_ctrl.get_position()))
                    print("Current Velocity [counts/s]: " + str(self.odrv_ctrl.get_velocity()))
                    key = raw_input("Enter a velocity or press q to quit: ")
                    if key != 'q': self.odrv_ctrl.set_velocity(int(key))
                    # TODO: Possibly add a delay here to give time for motor to spin so position is read correctly on next loop

            elif key == '4':
            # Voltage Control Mode
                self.odrv_ctrl.set_voltage()

            elif key == '5':
            # Set Limits
                print("Current Limit: " + str(self.odrv_ctrl.get_current_limit()) + "\nVelocity Limit: " + str(self.odrv_ctrl.get_velocity_limit())),
                key = raw_input("\n1. Current Limit\n2. Velocity Limit\n3. Abort\nWhich limit would you like to change: ")
                if key == '1':
                    # Current
                    key = raw_input("Enter a new Current Limit: ")
                    self.odrv_ctrl.set_current_limit(int(key))
                    print("New Current Limit: " + str(self.odrv_ctrl.get_current_limit()))
                elif key == '2':
                    # Velocity
                    key = raw_input("Enter a new Velocity Limit: ")
                    self.odrv_ctrl.set_velocity_limit(int(key))
                    print("New Velocity Limit: " + str(self.odrv_ctrl.get_velocity_limit()))
                else:
                # Aborts with any other key
                    pass


            elif key == '6':
            # Dump Configs
                key = raw_input("Reset Errors? [y/n]: ")
                if (key == "y"):
                    self.odrv_ctrl.dump_errors(True)
                else:
                    self.odrv_ctrl.dump_errors()
                self.odrv_ctrl.dump_motor_config()
                self.odrv_ctrl.dump_encoder_config()
                self.odrv_ctrl.dump_errors()

    def exo_loop(self):
        """
        For use in full system
        """
        pass

class ODrive_ctrl:
    """
    ODrive control commands - This class should get moved to the "include" folder
    """

    # Declare ODrive Related Variables
    odrv = None
    axis0 = None # Motor 0
    axis1 = None # Motor 1
    encoder_cpr = 8192
    encoder_kv = 192

    # Connection

    def connect(self):
        print("Finding an ODrive...")
        self.odrv = odrive.find_any()
        if self.odrv:
            self.axis0 = self.odrv.axis0
            self.axis1 = self.odrv.axis1
            print("Found an ODrive!")
        else:
            print("Couldn't find an ODrive...")

    def reboot_odrive(self):
        print("Rebooting and Reconnecting")
        try:
            self.odrv.reboot()
        except fibre.ChannelBrokenException:
            print("Expected connection loss due to reboot")
        self.connect()

    # Calibration

    def auto_calibrate(self, debug):
        """
        Auto Calibration - should only be needed once per life of motor.
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
        #   https://docs.google.com/spreadsheets/d/12vzz7XVEK6YNIOqH0jAz51F5VUpc-lJEs3mmkWP1H4Y/edit#gid=0 --> Motor limits
        self.axis0.motor.config.calibration_current = 10
        self.axis0.motor.config.current_lim = 68 # From above link
        self.axis0.motor.config.resistance_calib_max_voltage = 3
        self.axis0.controller.config.vel_limit = 50000 # counts/s
        self.odrv.save_configuration()
        self.reboot_odrive()
        print("\nIndex Values Before: " + str(self.axis0.encoder.is_ready) + " " + str(self.axis0.encoder.index_found))
        print("Delaying 10 seconds for manual calibration movement")
        time.sleep(10)
        print("\nIndex Values After: " + str(self.axis0.encoder.is_ready) + " " + str(self.axis0.encoder.index_found))

    # Getters

    def get_position(self):
        return self.axis0.encoder.pos_estimate

    def get_velocity(self):
        return self.axis0.encoder.vel_estimate

    def get_current_limit(self):
        return self.axis0.motor.config.current_lim

    def get_velocity_limit(self):
        return self.axis0.controller.config.vel_limit

    # Setters

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

    def set_position(self, position):
        self.set_requested_state(self.axis0, AXIS_STATE_CLOSED_LOOP_CONTROL)
        self.axis0.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
        self.axis0.controller.pos_setpoint = position # [counts]
        print("Set position to: " + str(position) + " encoder units")

    def set_velocity(self, velocity):
        self.set_requested_state(self.axis0, AXIS_STATE_CLOSED_LOOP_CONTROL)
        self.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
        self.axis0.controller.vel_setpoint = velocity # [counts/sec]
        print("Set Velocity to: " + str(velocity) + "counts/s (Non-Ramped)")

    def set_current(self, current):
        # Set the current, proportional to torque
        # NOTE: There is no velocity limiting in current control mode, make
        # sure to not overrev the motor or exceed max speed for encode.
        if (current >= 0 and current <= 68):
            self.set_requested_state(self.axis0, AXIS_STATE_CLOSED_LOOP_CONTROL)
            self.axis0.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL
            self.axis0.controller.current_setpoint = current
        else:
            print("Current outside motor limits")

    def set_voltage(self):
        self.set_requested_state(self.axis0, AXIS_STATE_CLOSED_LOOP_CONTROL)
        self.axis0.controller.config.control_mode = CTRL_MODE_VOLTAGE_CONTROL

    def set_current_limit(self, new_current_lim):
        self.axis0.motor.config.current_lim = new_current_lim
        print("Set global current limit = " + str(new_current_lim))

    def set_velocity_limit(self, new_velocity_lim):
        self.axis0.controller.config.vel_limit = new_velocity_lim
        print("Set global velocity limit = " + str(new_velocity_lim))

    # Config and Error Dumps

    def dump_errors(self, clear=False):
        odrive.utils.dump_errors(self.odrv, clear)

    def dump_motor_config(self):
        print(self.axis0.motor.config)

    def dump_encoder_config(self):
        print(self.axis0.encoder.config)

if __name__ == "__main__":
    odrv_node = ODrive_ROS()
    while (True):
        odrv_node.user_loop()
        rospy.spin()