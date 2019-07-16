#include <Arduino.h>

const int FLEX_PIN = A0; // Pin connected to voltage divider output

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
float targetAngle = 90; 
float currentAngle;
float prevAngle;
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

  currentError = targetAngle - currentAngle;
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
  Serial.begin(9600);
  pinMode(FLEX_PIN, INPUT);
}

void loop() 
{
  float controlVal;

  // Read the ADC, and calculate voltage and resistance from it
  int flexADC = analogRead(FLEX_PIN);
  float flexV = flexADC * VCC / 1023.0;
  float flexR = R_DIV * (VCC / flexV - 1.0);
  Serial.println("Resistance: " + String(flexR) + " ohms");

  // Use the calculated resistance to estimate the sensor's
  // bend angle:
  currentAngle = map(flexR, STRAIGHT_RESISTANCE, BEND_RESISTANCE,
                   0, 90.0);
  currentTime = millis();
  controlVal = PD();
  Serial.println("Bend: " + String(currentAngle) + " degrees");
  Serial.println("Current control value: " + String(controlVal));
  Serial.println();
  delay(500);
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