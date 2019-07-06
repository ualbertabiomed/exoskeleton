// Use the following link for info on how to overwrite main
// http://gammon.com.au/forum/?id=12625

#include <Arduino.h>
#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

int main(void)
{
  init();

  // Dont know what this does but if uncommented results in error saying multiple mains
  //initVariant();
  // "The problem is that initVariant is in the same module as main.
  // Calling initVariant causes the linker to include all of main.cpp."

#if defined(USBCON)
  USBDevice.attach();
#endif

  setup();

  for (;;) {
	  // flash the onboard led on and off btwn rotations so we know that we successfully used our main
	digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
	delay(1000);                       // wait for a second
	digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
	delay(1000);                       // wait for a second
    loop();
    if (serialEventRun) serialEventRun();
  }

  return 0;
}

void setup() {
  myservo.attach(3);  // attaches the servo on pin 9 to the servo object

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }

  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}
