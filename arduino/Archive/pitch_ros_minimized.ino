// The SFE_LSM9DS1 library requires both Wire and SPI be
// included BEFORE including the 9DS1 library.
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>



// code added in for ros node
#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;

std_msgs::Float32 str_msg;
ros::Publisher chatter("chatter", &str_msg);



// Use the LSM9DS1 class to create an object. [imu] can be
// named anything, we'll refer to that throught the sketch.
LSM9DS1 imu;



// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

////////////////////////////
// Sketch Output Settings //
////////////////////////////
#define PRINT_CALCULATED
//#define PRINT_RAW
#define PRINT_SPEED 250 // 250 ms between prints
static unsigned long lastPrint = 0; // Keep track of print time


void printPitch(float ax, float ay, float az);




void setup() {
  //begin serial communication
  Serial.begin(115200);

  // code added in for ros node to work
  nh.initNode();
  nh.advertise(chatter);


  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    while (1)
      ;
  }

}

void loop() {
  if ( imu.accelAvailable() )
  {
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    imu.readAccel();
  }
  printPitch(imu.ax,imu.ay,imu.az);

}
  void printPitch(float ax, float ay, float az)
{
  
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));
  
  // Convert everything from radians to degrees:
  pitch *= 180.0 / PI;
  /*
  Serial.print("Pitch: ");
  Serial.flush();
  Serial.print(pitch, 2);
  Serial.flush();
  */

  // code added in for ros node
  str_msg.data = pitch;
  chatter.publish( &str_msg );
  nh.spinOnce();
  

  
 
 
}// end of void printPitch

  
