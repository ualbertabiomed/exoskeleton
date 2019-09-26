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

#ifndef EXOSKELETON_H
#define EXOSKELETON_H

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>

using namespace std;

struct Gains {
	double Kp;
	double Ki;
	double Kd;
};

class Exoskeleton {
public:
	Exoskeleton();
	~Exoskeleton();
	void run();
    void calibration();
	void write_odrive(int axis, String command);
	void write_odrive(int axis, String command, float val);
	void write_terminal(String command, String val);
	String read_odrive(String input);
	void call_run_state(int axis, int requested_state, bool wait);
	void check_odrive_calibration(String command, String expected_value);
	void calibration_error(String command, String output);
	char wait_for_input();
	double pid();
	double set_point();
	void loop();

protected:
	Gains gains;

private:
    SoftwareSerial odrive_serial;
    ODriveArduino odrive;


	const String HEADER = "\nExoskeleton © 2019 UAlbertaBiomed\n\n";
};

#endif // EXOSKELETON_H
