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

//const int FLEX_PIN = A0; // Pin connected to voltage divider output

// Measure the voltage at 5V and the actual resistance of your
// 47k resistor, and enter them below:
// const float VCC = 4.98; // Measured voltage of Ardunio 5V line
// const float R_DIV = 47500.0; // Measured resistance of 3.3k resistor

// // Upload the code, then try to adjust these values to more
// // accurately calculate bend degree.
// const float STRAIGHT_RESISTANCE = 37300.0; // resistance when straight
// const float BEND_RESISTANCE = 90000.0; // resistance at 90 deg

// const float minFlex = 12; 
// const float maxFlex = 200;
float targetPosition = 90; 
float currentPosition;
float currentTime;
float prevTime;
float Kp = 1;
float Kd = 0;
float prevError = 0;

float PD() {
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


String readODrive(String input) {
    odrive_serial << input;

    String str = "";
    static const unsigned long timeout = 1000;
    unsigned long timeout_start = millis();
    for (;;) {
        while (!odrive_serial.available()) {
            if (millis() - timeout_start >= timeout) {
                return str;
            }
        }
        char c = odrive_serial.read();
        if (c == '\n')
            break;
        str += c;
    }
    return str;
}

bool errorCheck() {
    bool check;
    String result;

    check = true;

    result = readODrive("r axis0.error\n");
    if(result != '0') {
        Serial.print("axis0.error == ");
        Serial.println(result.toInt(), HEX);
        check = false;
    }

    result = readODrive("r axis0.motor.config.direction\n");
    if(result != "1" || result != "-1") {
        Serial.print("axis0.motor.config.direction == ");
        Serial.println(result);
        check = false;
    }

    return check;
}

void calibration() {
    odrive_serial << "w axis0.encoder.config.use_index True" << '\n';
    int motornum = '0'-'0';
    int requested_state;

    requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
    Serial << "Axis" << '0' << ": Requesting state " << requested_state << '\n';
    odrive.run_state(motornum, requested_state, true);

    requested_state = ODriveArduino::AXIS_STATE_ENCODER_INDEX_SEARCH;
    Serial << "Axis" << '0' << ": Requesting state " << requested_state << '\n';
    odrive.run_state(motornum, requested_state, true);

    // odrive.run_state(0, ODriveArduino::AXIS_STATE_ENCODER_INDEX_SEARCH, true);
    // odrive.run_state(0, ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION, true);
    // odrive_serial << "w axis0.requested_state " << ODriveArduino::AXIS_STATE_ENCODER_INDEX_SEARCH << '\n';
    // odrive_serial << "w axis0.requested_state " << ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION << '\n';

    if(errorCheck()) {
        Serial.print("axis0.encoder.config.offset == ");
        Serial.println(readODrive("axis0.encoder.config.offset\n"));

        odrive_serial << "w axis0.encoder.config.pre_calibrated True\n";
        odrive_serial << "w axis0.config.startup_encoder_index_search True\n";
        odrive_serial << "w axis0.motor.config.pre_calibrated True\n";
        odrive_serial << "axis0.save_configuration()\n";
        odrive_serial << "odrv0.reboot()\n";

    } else {
        Serial.println("There was an error!!");
    }
}

void setup() {
    // ODrive uses 115200 baud
    odrive_serial.begin(115200);

    // Serial to PC
    Serial.begin(9600);
    while (!Serial) ; // wait for Arduino Serial Monitor to open

    Serial.println("ODriveArduino");
    Serial.println("Setting parameters...");

    int numMotors = 1;
    for (int axis = 0; axis < numMotors; ++axis) {
        odrive_serial << "w axis" << axis << ".controller.config.vel_limit " << 22000.0f << '\n';
        odrive_serial << "w axis" << axis << ".motor.config.current_lim " << 11.0f << '\n';
        // This ends up writing something like "w axis0.motor.config.current_lim 11.0\n"
    }

    // calibration();

    Serial.println("Ready!");
    Serial.println("Send the character '0' calibrate motor (you must do this before you can command movement)");
    Serial.println("Send the character 's' to exectue test move");
    Serial.println("Send the character 't' to execute the incremental movement");
    Serial.println("Send the character 'g' to check if the ODrive saved the calibration of the motor and encoder");
    Serial.println("Send the character 'b' to read bus voltage");
    Serial.println("Send the character 'p' to read motor positions in a 10s loop");
}


void loop() {
    if (Serial.available()) {
        char c = Serial.read();

        // if(c == 'g') {
        //     odrive_serial << "r axis0.motor.config.pre_calibrated" << '\n';
        //     String str = "";
        //     static const unsigned long timeout = 1000;
        //     unsigned long timeout_start = millis();
        //     for(;;) {
        //         while(!odrive_serial.available()) {
        //             if(millis() - timeout_start >= timeout) {
        //                 return str;
        //             }
        //         }
        //         char c = odrive_serial.read();
        //         if(c == '\n') {
        //             break;
        //         }
        //         str += c;
        //     }
        //     Serial << "Is the motor calibrated? " << str << '\n';

        //     odrive_serial << "r axis0.encoder.config.pre_calibrated" << '\n';
        //     timeout_start = millis();
        //     for(;;) {
        //         while(!odrive_serial.available()) {
        //             if(millis() - timeout_start >= timeout) {
        //                 return str;
        //             }
        //         }
        //         char c = odrive_serial.read();
        //         if(c == '\n') {
        //             break;
        //         }
        //         str += c;
        //     }
        //     Serial << "Is the encoder calibrated? " << str << '\n';   
        // }

        if (c == 's') {
            Serial.println("Executing test move");
            for (float ph = 0.0f; ph < 0.0872665f; ph += 0.01f) {
                float pos_m0 = 20000.0f * cos(ph);
                odrive.SetPosition(0, pos_m0);
                delay(5);
            }
            Serial.println("Finished test move");
        }

        // if (c == 'n') {
        //   Serial.println("Executing test move");
        //   odrive.SetPosition(0, 0.0872665f);
        //   delay(5);
        // }

        if(c == 't') {
            Serial.println("Running Increment loop");
            Serial.println("Setting to zero");

            float testCurrentPosition = 0;
            odrive.SetPosition(0, testCurrentPosition);

            Serial.println("Press n to increment by 5 degrees");

            while(1) {
                char c = Serial.read();

                if(c == 'n') {
                    Serial.println("pressed n");
                    testCurrentPosition += 0.0872665f;
                    odrive.SetPosition(0,testCurrentPosition);
                } else if(c == 'q') {
                    Serial.println("pressed q");
                    break;
                }
                if(testCurrentPosition >= 45) {
                    Serial.println("passed 45");
                    float testCurrentPosition = 0;
                    odrive.SetPosition(0, testCurrentPosition);
                }
            }
        }

        // Run calibration sequence
        if (c == '0') {
            odrive_serial << "w axis0.encoder.config.use_index True" << '\n';

            int motornum = c-'0';
            int requested_state;

            requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
            Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
            odrive.run_state(motornum, requested_state, true);

            // This should store the calibration
            // Not sure if the syntax is correct, goal was to send the String command to the serial -cyrus 
            // odrive_serial << "w axis0.motor.config.pre_calibrated = True" << '\n';

            requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
            Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
            odrive.run_state(motornum, requested_state, true);

            // This should store the calibration
            // Not sure if the syntax is correct, goal was to send the String command to the serial -cyrus
            // odrive_serial << "w axis0.encoder.pre_calibrated = True" << '\n';

            requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
            Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
            odrive.run_state(motornum, requested_state, false); // don't wait
            odrive_serial << "w axis0.controller.set_vel_setpoint(3000,0)" << '\n';
            delay(3000);
            requested_state = ODriveArduino::AXIS_STATE_IDLE;
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

                // currentPosition = NOTREALGETPOSITION();

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

int main() {
    init();
    setup();

    while(true) {
        loop();
    }

    Serial.flush();
    Serial.end();

    return 0;
}