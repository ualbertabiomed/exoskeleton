/********************************************************************
This is a template for further implementations to the sensors and it's 
raw data.

Data types for all sensors (including fuse algorithm) are initialized
to be float

Please provide sufficient documentation to code scripted in the sensor
functions

The Makefile for compiling and sending code to arduino is still in
development
********************************************************************/

// Standard arduino library
#include <Arduino.h>

// The SFE_LSM9DS1 library requires both Wire and SPI be
// included BEFORE including the 9DS1 library.
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

// Code added in for ros node
#include <ros.h>
#include <std_msgs/Float32.h>

#include "flex_sensor.h"

// Initialize pins (may change depending on amount of sensors)
#define IMU1 A0
#define IMU2 A1

#define FSR1 A2
#define FSR2 A3
#define FSR3 A4
#define FSR4 A5
#define FSR5 A6
#define FSR6 A7
#define FSR7 A8
#define FSR8 A9
#define FSR9 A10

#define FLEX1 A11
#define FLEX2 A12

/*	
	DECLARE ALL CONSTANTS FOR IMU HERE
*/

/*
	DECLARE ALL CONSTANTS FOR FLEX_SENSOR HERE
*/

/*
	DECLARE ALL CONSTANTS FOR FSR HERE
*/

float IMU_sensor() {
	/*
		Implement code to get data from 2 IMUs on each hand. Use the data 
		to determine the angle
	*/
}

float FSR_sensor() {
	/*
		Implement code to read data from all 9 FSR. Interpret and realize 
		the raw data to meaningful information
	*/
}

float flex_sensor() {
	/*
		Implement code to read data from flex sensors. Calculate the angle
		using the raw data.
	*/
	// Instantiate flex_sensor object
	flexSensor flexSensorA((float) STRAIGHT_RESISTANCE,(float) BEND_RESISTANCE,(float));
	// Setups the the serial communication with the the arduino
	flexSensorA.setup(int PIN);
	// Outputs the angle into the serial monitor for now (future: need to use ros node)
	flexSensorA.run();

}

float fuse(float IMU_data, float flex_data, float FSR_data) {
	/*
		Implement an algorithm to combine all data from the sensors to
		calculate a 'final' position value; which will be sent to odrive.
	*/
}

int setup() {
	init();

	// begins serial communication with arduino
	Serial.begin(9600);

	// Set analog pin mode for IMU as input
	pinMode(IMU1, INPUT);
	pinMode(IMU2, INPUT);
	
	// Set analog pin mode for FSR as input
	pinMode(FSR1, INPUT);
	pinMode(FSR2, INPUT);
	pinMode(FSR3, INPUT);
	pinMode(FSR4, INPUT);
	pinMode(FSR5, INPUT);
	pinMode(FSR6, INPUT);
	pinMode(FSR7, INPUT);
	pinMode(FSR8, INPUT);
	pinMode(FSR9, INPUT);

	// Set analog pin mode for flex sensor as input
	pinMode(FLEX1, INPUT);
	pinMode(FLEX2, INPUT);
}

int main() {
	while(true) {
		// fuses all sensors to get a single position
		goalPos = fuse(IMU_sensor(), flex_sensor(), FSR_sensor());
		ODriveSerial(goalPos);
	}
}