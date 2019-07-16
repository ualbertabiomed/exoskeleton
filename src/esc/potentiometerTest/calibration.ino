#include <Servo.h>

Servo myservo;

const byte pot_Pin = A0;
const byte servo_Pin = 3;

void setup()
{
  Serial.begin(9600);
  myservo.attach(servo_Pin);
  myservo.write(110);
}

void loop()
{
  potentiometerStuff();
}

void potentiometerStuff()
{
  int data = 0;
  data = map(analogRead(pot_Pin),0,1023,0,180);
  Serial.println(data);

  myservo.write(data);
  
  return;
}
