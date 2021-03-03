#include "sensor_reading.h"

// initialize SensorGroup instance for the FSR sensors
const int numFsrSensors = 9;
int fsrPins[numFsrSensors] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
SensorGroup fsrSensors = SensorGroup(numFsrSensors, fsrPins);
float fsrVoltageReadings[numFsrSensors];
// initialize SensorGroup instance for the flex sensors
const int numFlexSensors = 5;
int flexPins[numFlexSensors] = {9, 10, 11, 12, 13};
SensorGroup flexSensors = SensorGroup(numFlexSensors, flexPins);
float flexVoltageReadings[numFlexSensors];

void setup() {
  // put your setup code here, to run once:
  // initialize digital pin LED_BUILTIN as an output.
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  fsrSensors.getVoltageReadings(&fsrVoltageReadings[0]);
  Serial.println("FSR Sensor Voltage Readings:"); Serial.print("[ ");
  for (int i = 0; i < numFsrSensors; ++i) {
    Serial.print("pin "); Serial.print(fsrPins[i]); Serial.print(": "); Serial.print(fsrVoltageReadings[i]);
    if (i == numFsrSensors - 1) Serial.println(" ]"); else Serial.print(", ");
  }
  flexSensors.getVoltageReadings(&flexVoltageReadings[0]);
  Serial.println("Flex Sensor Voltage Readings:"); Serial.print("[ ");
  for (int i = 0; i < numFlexSensors; ++i) {
    Serial.print("pin "); Serial.print(flexPins[i]); Serial.print(": "); Serial.print(flexVoltageReadings[i]);
    if (i == numFlexSensors - 1) Serial.println(" ]"); else Serial.print(", ");
  }
  delay(SLEEP_TIME);
}
