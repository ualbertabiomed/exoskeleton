#ifndef _FLEX_SENSOR_H_
#define _FLEX_SENSOR_H_
#include <Arduino.h>

class flexSensor {
public:
	// Measure the voltage at 5V
	const float VCC = 4.98;

	flex_sensor(const float STRAIGHT_RESISTANCE, const float BEND_RESISTANCE, const float R_DIV) {
		this->STRAIGHT_RESISTANCE = STRAIGHT_RESISTANCE;
		this->BEND_RESISTANCE = BEND_RESISTANCE;
		this->R_DIV = R_DIV;
	}

	void setup(const int FLEX_PIN) {
		this->FLEX_PIN = FLEX_PIN;
		init();
  		Serial.begin(9600);
  		pinMode(this->FLEX_PIN, INPUT);
	}

	void run() {
		int flexADC = analogRead(FLEX_PIN);
	    float flexV = flexADC * VCC / 1023.0;
	    float flexR = R_DIV * (VCC / flexV - 1.0);
	    Serial.println("Resistance: " + String(flexR) + " ohms");

	    // Use the calculated resistance to estimate the sensor's
	    // bend angle:
	    float angle = map(flexR, STRAIGHT_RESISTANCE, BEND_RESISTANCE,
	                     0, 90.0);
	    Serial.println("Bend: " + String(angle) + " degrees");
	    Serial.println();

	    delay(500);
	}

private:
	const float STRAIGHT_RESISTANCE;
	const float BEND_RESISTANCE;
	const float R_DIV
	const int FLEX_PIN;
}


#endif