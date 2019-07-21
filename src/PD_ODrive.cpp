#include <Arduino.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>

// Printing with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

// Serial to the ODrive
SoftwareSerial odrive_serial(8, 9); //RX (ODrive TX), TX (ODrive RX)
// Note: you must also connect GND on ODrive to GND on Arduino!

ODriveArduino odrive(odrive_serial);

//const int FLEX_PIN = A0; // Pin connected to voltage divider output

// Measure the voltage at 5V and the actual resistance of your
// 47k resistor, and enter them below:
const float VCC = 4.98; // Measured voltage of Ardunio 5V line
const float R_DIV = 47500.0; // Measured resistance of 3.3k resistor

// Upload the code, then try to adjust these values to more
// accurately calculate bend degree.
const float STRAIGHT_RESISTANCE = 37300.0; // resistance when straight
const float BEND_RESISTANCE = 90000.0; // resistance at 90 deg

const float minFlex = 12; 
const float maxFlex = 200;
float targetPosition = 90; 
float currentPosition;
float currentTime;
float prevTime;
float Kp = 1;
float Kd = 0;
float prevError = 0;

float PD()
{
  float currentError;
  float derivative;
  float output;
  float dTime;
  float dError;

  currentError = targetPosition - currentPosition;
  dTime = currentTime - prevTime;
  dError = currentError - prevError;
  derivative =  dError / dTime;
  output = (Kp * currentError) + (Kd * derivative);
  prevError = currentError;
  prevTime = currentTime;

  return output;
}

void setup() 
{
  // ODrive uses 115200 baud
  odrive_serial.begin(115200);

  // Serial to PC
  Serial.begin(115200);
  while (!Serial) ; // wait for Arduino Serial Monitor to open

  Serial.println("ODriveArduino");
  Serial.println("Setting parameters...");

  int numMotors = 1;
  for (int axis = 0; axis < numMotors; ++axis) {
    odrive_serial << "w axis" << axis << ".controller.config.vel_limit " << 22000.0f << '\n';
    odrive_serial << "w axis" << axis << ".motor.config.current_lim " << 11.0f << '\n';
    // This ends up writing something like "w axis0.motor.config.current_lim 11.0\n"
  }

  Serial.println("Ready!");
  Serial.println("Send the character '0' calibrate motor (you must do this before you can command movement)");
  Serial.println("Send the character 'b' to read bus voltage");
  Serial.println("Send the character 'p' to read motor positions in a 10s loop");
}

void loop() 
{
  if (Serial.available()) {
    char c = Serial.read();

    // Run calibration sequence
    if (c == '0') {
      int motornum = c-'0';
      int requested_state;

      requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
      Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
      odrive.run_state(motornum, requested_state, true);

      requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
      Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
      odrive.run_state(motornum, requested_state, true);

      requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
      odrive.run_state(motornum, requested_state, false); // don't wait
    }

    // Read bus voltage
    if (c == 'b') {
      odrive_serial << "r vbus_voltage\n";
      Serial << "Vbus voltage: " << odrive.readFloat() << '\n';
    }

    // print motor position in a 10s loop
    if (c == 'p') {
      static const unsigned long duration = 10000;
      unsigned long start = millis();
      while(millis() - start < duration) {
        int numMotors = 1;
        for (int motor = 0; motor < numMotors; ++motor) {
          odrive_serial << "r axis" << motor << ".encoder.pos_estimate\n";
          Serial << odrive.readFloat() << '\t';
        }
        Serial << '\n';
      }
    }
    // execute PD
    if (c == 'd') {
      // loop in here dont want to exit at this point
      for(;;) {
        float controlVal;

        currentPosition = NOTREALGETPOSITION();

        // PD stuff
        currentTime = millis();
        controlVal = PD();

        // Values to be recorded from Serial (time [space] error \n)
        Serial.print(currentTime);
        Serial.print(" ");
        Serial.println(controlVal);
        Serial.println();

        // Change delay as necessary
        delay(500);
      }
    }
  }
}

int main()
{
  init();
  setup();

  while(true)
  {
    loop();
  }

  Serial.flush();
  Serial.end();
  
  return 0;
}