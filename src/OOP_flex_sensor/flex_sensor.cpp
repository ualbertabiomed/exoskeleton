/******************************************************************************
Flex_Sensor_Example.ino
Example sketch for SparkFun's flex sensors
  (https://www.sparkfun.com/products/10264)
Jim Lindblom @ SparkFun Electronics
April 28, 2016

Create a voltage divider circuit combining a flex sensor with a 47k resistor.
- The resistor should connect from A0 to GND.
- The flex sensor should connect from A0 to 3.3V
As the resistance of the flex sensor increases (meaning it's being bent), the
voltage at A0 should decrease.

Development environment specifics:
Arduino 1.6.7
******************************************************************************/
#include <Arduino.h>
#include "flex_sensor.h"

int main() 
{
  // Read the ADC, and calculate voltage and resistance from it
  flexSensor a((float)13000.0,(float)26500.0,(float)15000.0);
  a.setup(A0);

  while(true) {
    a.run();
  }
} 
