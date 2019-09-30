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

#ifndef JOINT_H
#define JOINT_H

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>

using namespace std;

class Joint {

protected:

SoftwareSerial *odrive_serial;
ODriveArduino *odrive;

public:
	Joint();
	//void run();
  void calibrate(int motor_number);
	void update();
	void set_position();
	void get_position();
	//double pid();
	//double set_point();
	//void loop();

private:

	const String HEADER = "\nExoskeleton © 2019 UAlbertaBiomed\n\n";
};

#endif // JOINT_H
