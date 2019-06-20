/*
 * Copyright (c) 2019 UAlbertaBiomed Exoskeleton. All rights reserved.
 *
 * This software may be modified and distributed
 * under the terms of the BSD 3-Clause license.
 *
 * Refer to the LICENSE file for details.
 *
 * Author: Laura Petrich
 * Created on: June 6, 2019
 *
 */

#ifndef EXOSKELETON_H
#define EXOSKELETON_H

#include <iostream>
#include <stdexcept>
#include <chrono>
#include <thread>
#include <string>
#include <ctime>

#include "SerialPort.h"
#include "../Servo-1.1.4/src/Servo.h"

using namespace std;

// For serial communication with Arduino
char output[MAX_DATA_LENGTH];
char incoming[MAX_DATA_LENGTH];
char *port = "\\\\.\\COM3";

static const char* const DIVISION_BY_ZERO_MESSAGE = "Division by zero is illegal";

class DivisionByZero : public exception {
public:
	virtual const char* what() const throw() {
		return DIVISION_BY_ZERO_MESSAGE;
	}
};

struct Gains {
	double Kp;
	double Ki;
	double Kd;
};

class Exoskeleton {
public:
	explicit Exoskeleton(Gains gains) {
		this->gains = gains;
	}
	~Exoskeleton() {
	}
	void init();
	double pid();
	double set_point();
	void loop();

protected:
	Gains gains;
	Servo ESC;
};

#endif // EXOSKELETON_H
