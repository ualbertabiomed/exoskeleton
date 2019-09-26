/*
    Copyright (c) 2019 UAlbertaBiomed Exoskeleton. All rights reserved.
 
    This software may be modified and distributed
    under the terms of the BSD 3-Clause license.

    Refer to the LICENSE file for details.
 
    Author: Taylor Zowtuk, Cyrus Diego, Laura Petrich
    Created on: August, 2019
*/

#include "exoskeleton.h"


// Printing with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }


/*
    @brief: Initializes Arduino and Odrive communications 
    RX (ODrive TX), TX (ODrive RX)
*/
Exoskeleton::Exoskeleton() :
    odrive_serial(10, 11), 
    odrive(odrive_serial)
{
    // Arduino header library 
    init();   
    
    Gains g;
    g.Kp = 10.0;
    g.Ki = 0.0;
    g.Kd = 0.0;
    this->gains = g;

    // Serial to PC
    Serial.begin(9600);
    while (!Serial) ; // wait for Arduino Serial Monitor to open

    // Serial to the ODrive
    // odrive_serial(10,11); //RX (ODrive TX), TX (ODrive RX)
    // Note: you must also connect GND on ODrive to GND on Arduino!
    // odrive(odrive_serial);


    // ODrive uses 115200 baud
    odrive_serial.begin(115200);

    Serial.println(HEADER);
}
/*
    @brief: state machine waits for input from user to run the exoskeleton
*/
void Exoskeleton::run() {
    Serial.println("Starting exoskeleton");
    Serial.println("Send the character 'c' to recalibrate the motor");
    Serial.println("Send the character 'q' to quit");
    odrive.SetVelocity(0,0);
    char ch;
    bool quit = 0;
    while(!quit) {
        ch = wait_for_input();

        switch(ch){
            case 'c': 
                calibration();
                break;
            case 'q':
                Serial.println("Quitting exoskeleton");
                quit = 1;
                break;            
        }
    }
}

/*
    @brief: runs initial calibration of the servo motor and odrive 
*/
void Exoskeleton::calibration() {
    Serial.println("ODriveArduino");

    // Reboot
    write_odrive("odrv0.reboot()");
    Serial.println("Rebooted...");
    Serial.println("Setting parameters...");

    int numMotors = 1; //set for the number of motors connected to the odrive to calibrate multiple motors
    // Note: save the configuration and reboot as the gains are written out to the DRV 
    // (MOSFET driver) only during startup
    for (int i = 0; i < numMotors; ++i) {

        Serial.println("setting initial values");
        // Set vel_limit; the motor will be limited to this speed [counts/s]
        write_axis(i, ".controller.config.vel_limit", 22000.0f);
        // Set current_lim; the motor will be limited to this current [A]
        write_axis(i, ".motor.config.current_lim", 11.0f);
        // This ends up writing something like "w axis0.motor.config.current_lim 11.0\n"

        // Set the encoder config; values will vary depending on the encoder used check datasheet
        write_axis(i, ".encoder.config.cpr", 8192);
        write_axis(i, ".encoder.config.mode ENCODER_MODE_INCREMENTAL");

        // Print vel_limit and current_lim to serial to verify they were set correctly
        // Might need to switch this to ( Serial  << "Axis" ) stream instead to have it printed to monitor

        check_odrive_calibration(i, ".controller.config.vel_limit", "22000");
        check_odrive_calibration(i, ".motor.config.current_lim","11.0");
        check_odrive_calibration(i, ".encoder.config.cpr","8192");
    }

    // Save configuration
    Serial.println("saving config and rebooting");
    write_odrive("odrv0.save_configuration()");
    write_odrive("odrv0.reboot()");

    // Delay to give time to reboot
    delay(5000);

    Serial.println("ODCalibration ; Ready!");

    int requested_state;
    // If your motor has problems reaching the index location due to the mechanical load, you can increase <axis>.motor.config.calibration_current

    /*  Measure phase resistance and phase inductance of the motor
        To store the results set <axis>.motor.config.pre_calibrated to True and save the configuration
        After that you don’t have to run the motor calibration on the next start up
        This modifies the variables <axis>.motor.config.phase_resistance and <axis>.motor.config.phase_inductance 
    */
    requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;

    for (int i = 0; i < numMotors; ++i) {
        call_run_state(i, requested_state, true);
        // Not in documentation but assume that a successful motor calibration will make the <axis>.motor.is_ready go to true
        // Otherwise, need to set here so that we can later enter offset_calibration

        write_axis(i,".encoder.config.use_index True");

        // Turn the motor in one direction until the encoder index is traversed
        // This state can only be entered if <axis>.encoder.config.use_index is True
        requested_state = ODriveArduino::AXIS_STATE_ENCODER_INDEX_SEARCH;
        call_run_state(i, requested_state, true);

        /*  Turn the motor in one direction for a few seconds and then back to measure the offset between the encoder position and the electrical phase
            Can only be entered if the motor is calibrated (<axis>.motor.is_calibrated)
            A successful encoder calibration will make the <axis>.encoder.is_ready go to true
        */
        requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
        call_run_state(i, requested_state, true);

        // Check that the encoder calibration was successful
        check_odrive_calibration(i, ".error","0");

        String command;
        String outputVal;

        command = "w axis" + i;
        command += ".encoder.config.offset";
        outputVal = read_odrive(command);
        Serial << command << " == " << outputVal << '\n'; // This should print a number, like -326 or 1364

        command = "w axis" + i;
        command += ".motor.config.direction";
        outputVal = read_odrive(command);
        if (outputVal != '1' || outputVal != "-1") {
            calibration_error(command, outputVal);
        }
        Serial << command << " == " << outputVal << '\n';

        // Set so that we dont need to manually call <axis>.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH  on every bootup
        // Instead will automatically search for the index at startup
        write_axis(i, ".encoder.config.pre_calibrated True");
        write_axis(i, ".config.startup_encoder_index_search True");
        write_axis(i, ".motor.config.pre_calibrated True");

        requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
        call_run_state(i, requested_state, false);
        // With closed loop control the motor will now try to maintain its target position
    }

    // Save configuration
    write_odrive("odrv0.save_configuration()");
    write_odrive("odrv0.reboot()");

    delay(5000);
}

/*
    @brief: writes a command to a specific odrive (will need improvements)

    @param: command to send to the connected odrive
*/
void Exoskeleton::write_odrive(String command) {
    odrive_serial << command << '\n';
}

/*
    @brief: writes to a specified motor a string command 

    @param: axis, motor 1 or 0 
    @param: command, string command being sent to odrive 
*/
void Exoskeleton::write_axis(int axis, String command) {
    odrive_serial << "w axis" << axis << command << '\n';
}

/*
    @brief: write to a specified motor a sting command and a value to pass

    @param: axis, motor 1 or 0
    @param: command, string command being sent to odrive
    @param: val, float value to send to odrive 
*/
void Exoskeleton::write_axis(int axis, String command, float val) {
    odrive_serial << "w axis" << axis << command << ' ' << val << '\n';
}

/*
    @brief: write to the terminal to provide system feedback 

    @param: command, the command passed to the odrive 
    @param: val, the value read from the odrive
*/ 
void Exoskeleton::write_terminal(String command, String val) {
    Serial << command << " == " << val << '\n';
}

/*
    @brief: calls the runstate method and prints feedback to the terminal

    @param: axis, the motor setting state to 
    @param: requested_state, the state to set the odrive to 
    @param: wait, from odrive library, set true to wait for response from odrive
*/
void Exoskeleton::call_run_state(int axis, int requested_state, bool wait) {
    Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
    odrive.run_state(axis, requested_state, wait);
}

/*
    @brief: reads input from the odrive given a specified command

    @param: input, the value to be checked from the odrive 
    @return: the value currently set in odrive
*/
String Exoskeleton::read_odrive(String input) {
    odrive_serial << input;

    String str = "";
    
    static const unsigned long timeout = 1000;
    unsigned long timeout_start = millis();
    while(true) {
        while(!odrive_serial.available()) {
            if(millis() - timeout_start >= timeout) {
                return str;
            }
        }
        char c;
        c = odrive_serial.read();
        if(c == '\n') {
            break;
        }
    }
    return str;
}

/*
    @brief: after setting parameters to the odrive, check if they are set properly

    @param: command, the command sent to the odrive 
    @param: expected_value, the value the parameter should be set to
    @return: calibrated, true if the value was set properly
*/
void Exoskeleton::check_odrive_calibration(int i, String command, String expected_value) {
    String input;
    String output;
    Serial.println(command);    
    input = "w axis" + i;
    input += command + '\n';
    output = read_odrive(input);  // reads the value from odrive 
    Serial.println(output);
    // Check if the return value is the expected 
    if(output != expected_value) {
        Serial.println("Error!!");
        calibration_error(command, output);
    }
    write_terminal(command, expected_value);
}

/*
    @brief: displays a calibration error 

    @param: command, the command being passed to the odrive
    @param: output, the value the parameter is currently set to
*/
void Exoskeleton::calibration_error(String command, String output) {
    Serial.println("Calibration error: ");
    Serial << command << " == " << output;
    Serial.println("Exiting...");
    exit(0);
}

/*
    @brief: reads user keyboard input into the terminal

    @return: the character read from terminal
*/
char Exoskeleton::wait_for_input() {
    while(!Serial.available()) ;
    return Serial.read();
}


