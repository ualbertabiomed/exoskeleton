//	#include	"ODrive.h"

class Joint {

private:

public:

	/*
		@param: ODrive oobject, boolean calibrate
	*/
		// NOTE: Should constructor has a motor_number param
	Joint() {
	/*
		NOTE: How to check wether calibration needs to happen?
  */
	SoftwareSerial odrive_serial = new odrive_serial(10, 11);
	ODriveArduino odrive(odrive_serial);

	Serial.begin(9600);
	while (!Serial) ; // wait for Arduino Serial Monitor to open

	odrive_serial.begin(115200); 	// ODrive uses 115200 baud
	// while (!odrive_serial);
	}
	/*
    	@brief: runs initial calibration of the DC motor and odrive
	*/
	joint::void calibrate(int motor_number) {
		odrive.calibrate(motor number);
	}

	/*
		 TODO: Write update, set_position, and get_position methods
	*/
	joint::void update() {
		// use set_position and get_position for pd control
	}

	/*
		@param: double pos
	*/
	joint::void set_position() {

	}

	/*
		@return: double pos
	*/
	joint::double get_position() {

	}

};
