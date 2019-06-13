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

#include "exoskeleton.h"

 /**************************************************************************
  * Connect to arduino and motors
  **************************************************************************/
void Exoskeleton::init() {
	cout << "Initializing exoskeleton..." << endl;
	return;
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
