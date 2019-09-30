#include "joint.h"
//	#include	"ODrive.h"


/*
	@param: odrive_serial, odrive
*/
	// NOTE: Should constructor has a motor_number param
Joint::Joint() {
	/*
		NOTE: How to check wether calibration needs to happen?
	*/
	odrive_serial = new SoftwareSerial(10, 11);
	odrive = new ODriveArduino(*odrive_serial);

	Serial.begin(9600);
	while (!Serial) ; // wait for Arduino Serial Monitor to open

	odrive_serial->begin(115200); 	// ODrive uses 115200 baud
	// while (!odrive_serial);
	Serial.println(HEADER);
}
/*
  	@brief: runs initial calibration of the DC motor and odrive
*/
void Joint::calibrate(int motor_number) {
	odrive->Calibrate(motor_number);
}

/*
	 TODO: Write update, set_position, and get_position methods
*/
void Joint::update() {
	// use set_position and get_position for pd control
}

/*
	@param: double pos
*/
void Joint::set_position() {

}

/*
	@return: double pos
*/
void Joint::get_position() {

}
