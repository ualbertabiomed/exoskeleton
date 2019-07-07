#include <Arduino.h>

const int flexsensor = A0;
const int led = 3;
int value; 

void setup()
{
    Serial.begin(9600);
    Serial.flush();
}

void loop()
{
    value = analogRead(flexsensor);
    Serial.println(value);
    value = map(value,500,700,0,255);
    analogWrite(led,value);
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