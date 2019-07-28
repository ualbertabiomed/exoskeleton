#include <Arduino.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>

// Printing with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

// Serial to the ODrive
SoftwareSerial odrive_serial(10, 11); //RX (ODrive TX), TX (ODrive RX)
// Note: you must also connect GND on ODrive to GND on Arduino!

ODriveArduino odrive(odrive_serial);

void setup() 
{
  // ODrive uses 115200 baud
  odrive_serial.begin(115200);

  // Serial to PC
  Serial.begin(9600);
  while (!Serial) ; // wait for Arduino Serial Monitor to open

  Serial.println("ODriveArduino");
  Serial.println("Setting parameters...");

  int numMotors = 1;
  // Note: save the configuration and reboot as the gains are written out to the DRV 
  // (MOSFET driver) only during startup
  for (int axis = 0; axis < numMotors; ++axis) {
    // Set vel_limit; the motor will be limited to this speed [counts/s]
    odrive_serial << "w axis" << axis << ".controller.config.vel_limit " << 22000.0f << '\n';
    // Set current_lim; the motor will be limited to this current [A]
    odrive_serial << "w axis" << axis << ".motor.config.current_lim " << 11.0f << '\n';
    // This ends up writing something like "w axis0.motor.config.current_lim 11.0\n"

    // Set the encoder config; values will vary depending on the encoder used check datasheet
    odrive_serial << "w axis" << axis << ".encoder.config.cpr " << 8192 << '\n';
    odrive_serial << "w axis" << axis << ".encoder.config.mode " << ENCODER_MODE_INCREMENTAL << '\n';
  }

  // Save configuration
  odrive_serial << "odrv0.save_configuration()";
  odrive_serial << "odrv0.reboot()";
  // Delay to give time to reboot
  delay(5000);
  // Print vel_limit and current_lim to serial to verify they were set correctly
  // Might need to switch this to ( Serial  << "Axis" ) stream instead to have it printed to monitor
  odrive_serial << "w axis" << axis << ".controller.config.vel_limit";
  odrive_serial << "w axis" << axis << ".motor.config.current_lim";
  odrive_serial << "w axis" << axis << ".encoder.config.cpr";
  odrive_serial << "w axis" << axis << ".encoder.config.mode";


  Serial.println("Ready!");
  Serial.println("Send the character '0' calibrate motor (you must do this before you can command movement)");
  Serial.println("Send the character 's' to exectue test move");
  Serial.println("Send the character 'b' to read bus voltage");
  Serial.println("Send the character 'p' to read motor positions in a 10s loop");
}

void loop() 
{
  if (Serial.available()) {
    char c = Serial.read();

    // Run calibration sequence
    if (c == '0') {
      // If your motor has problems reaching the index location due to the mechanical load, you can increase <axis>.motor.config.calibration_current
      int motornum = c-'0';
      int requested_state;

      requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
      /* Measure phase resistance and phase inductance of the motor
         To store the results set <axis>.motor.config.pre_calibrated to True and save the configuration
         After that you don’t have to run the motor calibration on the next start up
         This modifies the variables <axis>.motor.config.phase_resistance and <axis>.motor.config.phase_inductance 
      */
      Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
      odrive.run_state(motornum, requested_state, true);
      // Not in documentation but assume that a successful motor calibration will make the <axis>.motor.is_ready go to true
      // Otherwise, need to set here so that we can later enter offset_calibration


      odrive_serial << "w axis" << axis << ".encoder.config.use_index " << "True" << '\n';
      requested_state = ODriveArduino::AXIS_STATE_ENCODER_INDEX_SEARCH;
      // Turn the motor in one direction until the encoder index is traversed
      // This state can only be entered if <axis>.encoder.config.use_index is True
      Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
      odrive.run_state(motornum, requested_state, true);

      requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
      /* Turn the motor in one direction for a few seconds and then back to measure the offset between the encoder position and the electrical phase
         Can only be entered if the motor is calibrated (<axis>.motor.is_calibrated)
         A successful encoder calibration will make the <axis>.encoder.is_ready go to true
      */
      Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
      odrive.run_state(motornum, requested_state, true);

      // Check that the encoder calibration was successful
      odrive_serial << "w axis" << axis << ".error";
      // This should print 0
      odrive_serial << "w axis" << axis << ".encoder.config.offset";
      // This should print a number, like -326 or 1364
      odrive_serial << "w axis" << axis << ".motor.config.direction";
      // This should print 1 or -1

      odrive_serial << "w axis" << axis << ".encoder.config.pre_calibrated " << "True" << '\n';
      // Set so that we dont need to manually call <axis>.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH  on every bootup
      // Instead will automatically search for the index at startup
      odrive_serial << "w axis" << axis << ".config.startup_encoder_index_search " << "True" << '\n';
      // Similarily, save current motor calibration and avoid doing it again on bootup
      odrive_serial << "w axis" << axis << ".motor.config.pre_calibrated " << "True" << '\n';



      requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
      odrive.run_state(motornum, requested_state, false); // don't wait

      // Save configuration
      odrive_serial << "odrv0.save_configuration()";
      odrive_serial << "odrv0.reboot()";
    }

    // Sinusoidal test move
    if (c == 's') {
      Serial.println("Executing test move");
      for (float ph = 0.0f; ph < 6.28318530718f; ph += 0.01f) {
        float pos_m0 = 20000.0f * cos(ph);
        float pos_m1 = 20000.0f * sin(ph);
        odrive.SetPosition(0, pos_m0);
        odrive.SetPosition(1, pos_m1);
        delay(5);
      }
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