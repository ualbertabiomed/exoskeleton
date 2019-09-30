
#include "Arduino.h"
#include "ODriveArduino.h"

static const int kMotorOffsetFloat = 2;
static const int kMotorStrideFloat = 28;
static const int kMotorOffsetInt32 = 0;
static const int kMotorStrideInt32 = 4;
static const int kMotorOffsetBool = 0;
static const int kMotorStrideBool = 4;
static const int kMotorOffsetUint16 = 0;
static const int kMotorStrideUint16 = 2;

// Print with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

ODriveArduino::ODriveArduino(Stream& serial)
    : serial_(serial) {}

/*
    @brief: Follows the "Encoder with Index Signal" from https://docs.odriverobotics.com/encoders#encoder-without-index-signal

    @param: motor_number, motor 1 or 0
*/
void ODriveArduino::Calibrate(int motor_number) {
  Serial.println("ODriveArduino");

  // Reboot
  write_odrive("odrv0.reboot()");
  Serial.println("Rebooted...");
  Serial.println("Setting parameters...");

  // Note: save the configuration and reboot as the gains are written out to the DRV
  // (MOSFET driver) only during startup

  Serial.println("setting initial values");
  // Set vel_limit; the motor will be limited to this speed [counts/s]
  write_axis(motor_number, ".controller.config.vel_limit", 22000.0f);
  // Set current_lim; the motor will be limited to this current [A]
  write_axis(motor_number, ".motor.config.current_lim", 11.0f);
  // This ends up writing something like "w axis0.motor.config.current_lim 11.0\n"

  // Set the encoder config; values will vary depending on the encoder used check datasheet
  write_axis(motor_number, ".encoder.config.cpr", 8192);
  write_axis(motor_number, ".encoder.config.mode ENCODER_MODE_INCREMENTAL");

  // Print vel_limit and current_lim to serial to verify they were set correctly
  // Might need to switch this to ( Serial  << "Axis" ) stream instead to have it printed to monitor

  check_odrive_calibration(motor_number, ".controller.config.vel_limit", "22000");
  check_odrive_calibration(motor_number, ".motor.config.current_lim", "11.0");
  check_odrive_calibration(motor_number, ".encoder.config.cpr", "8192");

  // Save configuration
  Serial.println("saving config and rebooting");
  write_odrive("odrv0.save_configuration()");
  write_odrive("odrv0.reboot()");

  // Delay to give time to reboot
  delay(5000);

  Serial.println("ODCalibration ; Ready!");

  int requested_state;
  // If your motor has problems reaching the index location due to the mechanical load, you can increase <axis>.motor.config.calibration_current

  /*  Measure phase resistance and phase inductance of the motor
      To store the results set <axis>.motor.config.pre_calibrated to True and save the configuration
      After that you don’t have to run the motor calibration on the next start up
      This modifies the variables <axis>.motor.config.phase_resistance and <axis>.motor.config.phase_inductance
  */
  requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;

  call_run_state(motor_number, requested_state, true);
  // Not in documentation but assume that a successful motor calibration will make the <axis>.motor.is_ready go to true
  // Otherwise, need to set here so that we can later enter offset_calibration

  write_axis(motor_number,".encoder.config.use_index True");

  // Turn the motor in one direction until the encoder index is traversed
  // This state can only be entered if <axis>.encoder.config.use_index is True
  requested_state = ODriveArduino::AXIS_STATE_ENCODER_INDEX_SEARCH;
  call_run_state(motor_number, requested_state, true);

  /*  Turn the motor in one direction for a few seconds and then back to measure the offset between the encoder position and the electrical phase
      Can only be entered if the motor is calibrated (<axis>.motor.is_calibrated)
      A successful encoder calibration will make the <axis>.encoder.is_ready go to true
  */
  requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
  call_run_state(motor_number, requested_state, true);

  // Check that the encoder calibration was successful
  check_odrive_calibration(motor_number, ".error", "0");

  String command;
  String outputVal;

  command = "w axis" + motor_number;
  command += ".encoder.config.offset";
  outputVal = read_odrive(command);
  Serial << command << " == " << outputVal << '\n'; // This should print a number, like -326 or 1364

  command = "w axis" + motor_number;
  command += ".motor.config.direction";
  outputVal = read_odrive(command);
  if (outputVal != '1' || outputVal != "-1") {
      calibration_error(command, outputVal);
  }
  Serial << command << " == " << outputVal << '\n';

  // Set so that we dont need to manually call <axis>.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH  on every bootup
  // Instead will automatically search for the index at startup
  write_axis(motor_number, ".encoder.config.pre_calibrated True");
  write_axis(motor_number, ".config.startup_encoder_index_search True");
  write_axis(motor_number, ".motor.config.pre_calibrated True");

  requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
  call_run_state(motor_number, requested_state, false);
  // With closed loop control the motor will now try to maintain its target position

  // Save configuration
  write_odrive("odrv0.save_configuration()");
  write_odrive("odrv0.reboot()");

  delay(5000);
}
/*
    @brief: Reset all configs
*/
void ODriveArduino::Reset() {
    write_odrive("odrv0.reset_configuration");
}

void ODriveArduino::SetPosition(int motor_number, float position) {
    SetPosition(motor_number, position, 0.0f, 0.0f);
}

void ODriveArduino::SetPosition(int motor_number, float position, float velocity_feedforward) {
    SetPosition(motor_number, position, velocity_feedforward, 0.0f);
}

void ODriveArduino::SetPosition(int motor_number, float position, float velocity_feedforward, float current_feedforward) {
    serial_ << "p " << motor_number  << " " << position << " " << velocity_feedforward << " " << current_feedforward << "\n";
}

void ODriveArduino::SetVelocity(int motor_number, float velocity) {
    SetVelocity(motor_number, velocity, 0.0f);
}

void ODriveArduino::SetVelocity(int motor_number, float velocity, float current_feedforward) {
    serial_ << "v " << motor_number  << " " << velocity << " " << current_feedforward << "\n";
}

void ODriveArduino::SetCurrent(int motor_number, float current) {
    serial_ << "c " << motor_number << " " << current << "\n";
}

void ODriveArduino::TrapezoidalMove(int motor_number, float position){
    serial_ << "t " << motor_number << " " << position << "\n";
}

float ODriveArduino::readFloat() {
    return readString().toFloat();
}

int32_t ODriveArduino::readInt() {
    return readString().toInt();
}

bool ODriveArduino::run_state(int axis, int requested_state, bool wait) {
    int timeout_ctr = 100;
    serial_ << "w axis" << axis << ".requested_state " << requested_state << '\n';
    if (wait) {
        do {
            delay(100);
            serial_ << "r axis" << axis << ".current_state\n";
        } while (readInt() != AXIS_STATE_IDLE && --timeout_ctr > 0);
    }

    return timeout_ctr > 0;
}

String ODriveArduino::readString() {
    String str = "";
    static const unsigned long timeout = 1000;
    unsigned long timeout_start = millis();
    for (;;) {
        while (!serial_.available()) {
            if (millis() - timeout_start >= timeout) {
                return str;
            }
        }
        char c = serial_.read();
        if (c == '\n')
            break;
        str += c;
    }
    return str;
}
