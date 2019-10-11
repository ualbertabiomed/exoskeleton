//	#include	"ODrive.h"

class Joint {

private:
	struct gains g;

public:

	/*
		@param: ODrive oobject, list gains, boolean calibrate
	*/
	Joint() {
	/*
        Attributes:
			ODrive = oobject;
			g.kp = gains[0];
			g.ki = gains[1];
			g.kd = gains[2];
			target_position;
			current_position;
		check whether calibration needs to happen during initialization
    */
	}

	/*
    	@brief: runs initial calibration of the DC motor and odrive
	*/
	void calibrate() {

	}


	void update() {
		// use set_position and get_position for pd control
	}

	/*
		@param: double pos
	*/
	void set_position() {

	}

	/*
		@return: double pos
	*/
	double get_position() {

	}

};