#define SLEEP_TIME 3000
#define VOLTAGE_SCALE 5.0
#define ANALOG_SCALE 1230.0

// class respresenting a group of sensors
class SensorGroup {
  // class representing the sensor corresponding to a single pin
  class Sensor {
    int pin;
    
    public:
      Sensor(int p) {
        pin = p;
      }
      int getAnalogReading() {
        return analogRead(pin);
      }
      float getVoltageReading() {
        return analogRead(pin) * (VOLTAGE_SCALE / ANALOG_SCALE);
      }
      bool verifyConnection() {
        // TODO - perform some action to verify that the pins are properly connected and transmitting
        return true;
      }
  };
  Sensor *sensors;
  bool verifyPinConnections() {
    bool conn = true;
    for (int i = 0; i < numSensors; ++i) {
      conn = conn && sensors[i].verifyConnection();
    }
    if (!conn) Serial.println("Some or all pins are not connected or properly transmitting");
    return conn;
  }
  
  public:
    int numSensors;
    SensorGroup(int numSensorsInGroup, int *pins);
    void getVoltageReadings(float *readings);
    void getAnalogReadings(int *readings);
};

SensorGroup::SensorGroup(const int numSensorsInGroup, int *pins) {
  // the length of the pins array must equal numSensorsInGroup
  sensors[numSensorsInGroup];
  numSensors = numSensorsInGroup;
  for (int i = 0; i < numSensorsInGroup; ++i) {
    sensors[i] = Sensor(pins[i]);
  }
}

void SensorGroup::getVoltageReadings(float *readings) {
  // verifies pin connections
  verifyPinConnections();
  // the array pointed to by readings must have a length equal to numSensors
  for (int i = 0; i < numSensors; ++i) {
    readings[i] = sensors[i].getVoltageReading();
  }
}

void SensorGroup::getAnalogReadings(int *readings) {
  // verifies pin connections
  verifyPinConnections();
  // the array pointed to by readings must have a length equal to numSensors
  for (int i = 0; i < numSensors; ++i) {
    readings[i] = sensors[i].getAnalogReading();
  }
}
