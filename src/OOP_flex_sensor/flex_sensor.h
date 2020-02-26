#ifndef _FLEX_SENSOR_H_
#define _FLEX_SENSOR_H_
#include <Arduino.h>

class flexSensor {
public:
	// Measure the voltage at 5V
	flexSensor(float STRAIGHT_RESISTANCE, float BEND_RESISTANCE, float R_DIV);
	const float VCC = 4.98;

	void setup(int FLEX_PIN);
	void run();

private:
	float STRAIGHT_RESISTANCE;
	float BEND_RESISTANCE;
	float R_DIV;
	int FLEX_PIN;
};

flexSensor::flexSensor(float STRAIGHT_RESISTANCE, float BEND_RESISTANCE, float R_DIV) {
	this->STRAIGHT_RESISTANCE = STRAIGHT_RESISTANCE;
	this->BEND_RESISTANCE = BEND_RESISTANCE;
	this->R_DIV = R_DIV;
}

void flexSensor::setup(int FLEX_PIN) {
	this->FLEX_PIN = FLEX_PIN;
	init();
	Serial.begin(9600);
	pinMode(this->FLEX_PIN, INPUT);
}

void flexSensor::run() {
	int flexADC = analogRead(FLEX_PIN);
    float flexV = flexADC * VCC / 1023.0;
    float flexR = this->R_DIV * (VCC / flexV - 1.0);
    Serial.print("Resistance: " );
    Serial.print(flexR); 
    Serial.println(" ohms");

    // Use the calculated resistance to estimate the sensor's
    // bend angle:
    float angle = map(flexR, STRAIGHT_RESISTANCE, BEND_RESISTANCE, 0, 90.0);
    //Serial.println("Bend: " + String(angle) + " degrees");
    Serial.print("Bend: ");
    Serial.print(angle);
    Serial.println(" degrees");

    delay(500);
}

#endif