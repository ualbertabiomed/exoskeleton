/*
 * Copyright (c) 2019 UAlbertaBiomed Exoskeleton. All rights reserved.
 *
 * This software may be modified and distributed
 * under the terms of the BSD 3-Clause license.
 *
 * Refer to the LICENSE file for details.
 *
 * Author: Taylor Zowtuk, Cyrus Diego, Laura Petrich
 * Created on: August, 2019
 *
 */

#include "exoskeleton.h"


// Printing with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

static const char* const HEADER = "\nExoskeleton © 2019 UAlbertaBiomed\n\n";

Exoskeleton::Exoskeleton() {
    init();

    Gains g;
    g.Kp = 10.0;
    g.Ki = 0.0;
    g.Kd = 0.0;
    this->gains = g;

    // Serial to PC
    Serial.begin(9600);
    while (!Serial) ; // wait for Arduino Serial Monitor to open

    // ODrive uses 115200 baud
    odrive_serial.begin(115200);

    // Serial to the ODrive
    odrive_serial(10, 11); //RX (ODrive TX), TX (ODrive RX)
    // Note: you must also connect GND on ODrive to GND on Arduino!
    odrive(odrive_serial);

    Serial.println(HEADER);
}
void Exoskeleton::run() {
    Serial.println("Starting exoskeleton");
    Serial.println("Send the character 'c' to recalibrate the motor");
    Serial.println("Send the character 'q' to quit");

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
/**************************************************************************
 * Get serial input
 **************************************************************************/
char Exoskeleton::wait_for_input() {
    while(!Serial.available()) ;
    return Serial.read();
}
/**************************************************************************
 * Calibrate the ODrive
 **************************************************************************/
void Exoskeleton::calibration() {
    Serial.println("ODriveArduino");
    // Reboot
    odrive_serial << "odrv0.reboot()";
    Serial.println("Rebooted...");
    Serial.println("Setting parameters...");

    int numMotors = 1; //set for the number of motors connected to the odrive to calibrate multiple motors
    // Note: save the configuration and reboot as the gains are written out to the DRV 
    // (MOSFET driver) only during startup
    for (int i = 0; i < numMotors; ++i) {
        // Set vel_limit; the motor will be limited to this speed [counts/s]
        odrive_serial << "w axis" << i << ".controller.config.vel_limit " << 22000.0f << '\n';
        // Set current_lim; the motor will be limited to this current [A]
        odrive_serial << "w axis" << i << ".motor.config.current_lim " << 11.0f << '\n';
        // This ends up writing something like "w axis0.motor.config.current_lim 11.0\n"

        // Set the encoder config; values will vary depending on the encoder used check datasheet
        odrive_serial << "w axis" << i << ".encoder.config.cpr " << 8192 << '\n';
        odrive_serial << "w axis" << i << ".encoder.config.mode " << "ENCODER_MODE_INCREMENTAL" << '\n';

        // Print vel_limit and current_lim to serial to verify they were set correctly
        // Might need to switch this to ( Serial  << "Axis" ) stream instead to have it printed to monitor
        String command;
        String outputVal;

        command = "w axis" + i + ".controller.config.vel_limit";
        outputVal = readOdrive(command);
        if(outputVal != "22000.0") {
            calibrationError(command, outputVal);
        }
        Serial << command << " == " << outputVal << '\n';

        command = "w axis" + i + ".motor.config.current_lim";
        outputVal = readOdrive(command);
        if(outputVal != "11.0") {
            calibrationError(command, outputVal);
        }
        Serial << command << " == " << outputVal << '\n';

        command = "w axis" + i + ".encoder.config.cpr";
        outputVal = readOdrive(command);
        if(outputVal != "8192") {
            calibrationError(command, outputVal);
        }
        Serial << command << " == " << outputVal << '\n';

        command = "w axis" + i + ".encoder.config.cpr";
        outputVal = readOdrive(command);
        if(outputVal != "8192") {
            calibrationError(command , outputVal);
        }
        Serial << command << " == " << outputVal << '\n';
    }
    // Save configuration
    odrive_serial << "odrv0.save_configuration()";
    odrive_serial << "odrv0.reboot()";
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
        Serial << "Axis" << i << ": Requesting state " << requested_state << '\n';
        odrive.run_state(i, requested_state, true);
        // Not in documentation but assume that a successful motor calibration will make the <axis>.motor.is_ready go to true
        // Otherwise, need to set here so that we can later enter offset_calibration


        odrive_serial << "w axis" << i << ".encoder.config.use_index " << "True" << '\n';

        // Turn the motor in one direction until the encoder index is traversed
        // This state can only be entered if <axis>.encoder.config.use_index is True
        requested_state = ODriveArduino::AXIS_STATE_ENCODER_INDEX_SEARCH;
        Serial << "Axis" << i << ": Requesting state " << requested_state << '\n';
        odrive.run_state(i, requested_state, true);

        /*  Turn the motor in one direction for a few seconds and then back to measure the offset between the encoder position and the electrical phase
            Can only be entered if the motor is calibrated (<axis>.motor.is_calibrated)
            A successful encoder calibration will make the <axis>.encoder.is_ready go to true
        */
        requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
        Serial << "Axis" << i << ": Requesting state " << requested_state << '\n';
        odrive.run_state(i, requested_state, true);

        // Check that the encoder calibration was successful
        String command;
        String outputVal;

        command = "w axis" + i + ".error";
        outputVal = readOdrive(command);
        if (outputVal != 0) {
            calibrationError(command, outputVal);
        }
        Serial << command << " == " << outputVal << '\n';

        command = "w axis" + i + ".encoder.config.offset";
        outputVal = readOdrive(command);
        Serial << command << " == " << outputVal << '\n'; // This should print a number, like -326 or 1364

        command = "w axis" + i + ".motor.config.direction";
        outputVal = readOdrive(command);
        if (outputVal != 1 || outputVal != -1) {
            calibrationError(command, outputVal);
        }
        Serial << command << " == " << outputVal << '\n';

        // Set so that we dont need to manually call <axis>.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH  on every bootup
        // Instead will automatically search for the index at startup
        odrive_serial << "w axis" << i << ".encoder.config.pre_calibrated " << "True" << '\n';
        odrive_serial << "w axis" << i << ".config.startup_encoder_index_search " << "True" << '\n';
        // Similarily, save current motor calibration and avoid doing it again on bootup
        odrive_serial << "w axis" << i << ".motor.config.pre_calibrated " << "True" << '\n';



        requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
        Serial << "Axis" << i << ": Requesting state " << requested_state << '\n';
        odrive.run_state(i, requested_state, false); // don't wait
        // With closed loop control the motor will now try to maintain its target position
    }

    // Save configuration
    odrive_serial << "odrv0.save_configuration()";
    odrive_serial << "odrv0.reboot()";
    delay(5000);
}


/**************************************************************************
 * Prints an error message if calibration was unsuccessful 
 **************************************************************************/
void Exoskeleton::calibrationError(String command, String output) {
    Serial.println("Calibration error: ");
    Serial << command << " == " << output;
    Serial.println("Exiting...");
    exit(0);
}

/**************************************************************************
 * Read feedback from the Odrive 
 * Parameter: string command that will be sent to the Odrive
 **************************************************************************/
String Exoskeleton::readOdrive(String input) {
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
        char c = odrive_serial.read();
        if(c == '\n') {
            break;
        }
    }
    return str;
}

/**************************************************************************
 * Calculate PID output
 **************************************************************************/
double Exoskeleton::pid() {
	double result;
	double target;
	result = 0.0;
	target = set_point();
	double error;
	double current;
	//current = Get Current Value TAYLOR;
	// error = target - current;
	double P;
	double I;
	double D;
	P = gains.Kp * error;
	I = gains.Ki;

	D = gains.Kd;

	time_t now = time(0);

	// convert now to string form
	char* dt = ctime(&now);

	cout << "The local date and time is: " << dt << endl;

	// convert now to tm struct for UTC
	tm* gmtm = gmtime(&now);
	dt = asctime(gmtm);
	cout << "The UTC date and time is:" << dt << endl;

	// Remember to check that denominator is not zero, throw this error if it is
	//if (gains.Kp == 0L) throw DivisionByZero();
	//cout << "Calculating PID..." << endl;
	return result;
}

/**************************************************************************
 * Calculate target set point to move to
 **************************************************************************/
double Exoskeleton::set_point() {
	double sp = 0.0;
	cout << "Finding new set point..." << endl;
	return sp;
}
/**************************************************************************
 * Main loop after connected, call pid from here
 **************************************************************************/
void Exoskeleton::loop() {
	bool shutdown = false;
	while (!shutdown) {
		string s;
		cout << "Looooooooop\nDo you want to continue? >>" << endl;
		this_thread::sleep_for(std::chrono::milliseconds(500));
		// getline will wait for the user to enter info and press enter
		getline(cin, s);
		if (tolower(s[0]) == 'n') {
			shutdown = true;
			cout << "See you later alligator!" << endl;
		}
	}
	return;
}
