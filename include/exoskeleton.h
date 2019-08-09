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
    void calibration();
	double pid();
	double set_point();
	void loop();

protected:
	Gains gains;

private:
    SoftwareSerial odrive_serial;
    ODriveArduino odrive;

    char wait_for_input();
    String readOdrive(String input);
    void calibrationError(String command, String output);
};

#endif // EXOSKELETON_H
