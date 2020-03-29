/* This is a sketch to run the motors using Adafruit assembled Motor Shield for Ardunio v2
 * the purpose of this code is to have the motor run in a straight line and a backwards line
 *
 */



#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
  Adafruit_MotorShield AFMS = Adafruit_MotorShield();

   // if I ever need to stack the motor shields I can create a different I2C address
  // Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

  //Select the 'port' M1, M2, M3, M4
  Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
  Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
  Adafruit_DCMotor *myMotor3 = AFMS.getMotor(3);
  Adafruit_DCMotor *myMotor4 = AFMS.getMotor(4);


// Forward Drive Function Definition
int motorsReverse(int i, int n);

// Function definition (header and body)
// This function drives all four motors forward at a user set speed (i) for a set delay time (n)
void setup()
{

}
void loop()
{

  uint8_t i;

  int n = 10;
  motorsReverse(i,n);

}


int motorsReverse(int i, int n)
{

  Serial.begin(9600);

  AFMS.begin(); //create with default frequency of 1.6KHZ
  // can also add different frequency of 1KHz AFMS.begin(1000)

  //set the speed to start, from 0 (off) to 255 (max speed)

for (i=0; i<255; i++) {
  myMotor1->setSpeed(i);
  myMotor2->setSpeed(i);
  myMotor3->setSpeed(i);
  myMotor4->setSpeed(i);
  delay(n);

  myMotor1->run(BACKWARD);
  myMotor2->run(BACKWARD);
  myMotor3->run(FORWARD);
  myMotor4->run(FORWARD);


  }
  // turn on motor
  myMotor1->run(RELEASE);
  myMotor2->run(RELEASE);
  myMotor3->run(RELEASE);
  myMotor4->run(RELEASE);

}