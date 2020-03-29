motorsRight.h

/* This is a sketch to run the motors using Adafruit assembled Motor Shield for Ardunio v2
 * the purpose of this code is to have the motor run in a straight line on left side and a backwards line on right side.
 *
 */



#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
  Adafruit_MotorShield AFMS = Adafruit_MotorShield();

   // if I ever need to stack the motor shields I can create a different I2C address
  // Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

  //Select the 'port' M1, M2, M3, M4
  Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1); //Front Left
  Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2); //Front Right
  Adafruit_DCMotor *myMotor3 = AFMS.getMotor(3); //Back Right
  Adafruit_DCMotor *myMotor4 = AFMS.getMotor(4); //Back Left


// Forward Drive Function Definition
int motorsRight(int i, int n);

// Function definition (header and body)
// This function drives all four motors forward at a user set speed (i) for a set delay time (n)
void setup()
{

}
void loop()
{

  uint8_t i;

  int n = 10;
  motorsRight(i,n);

}


int motorsRight(int i, int n)
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

  myMotor1->run(FORWARD); //Front Left
  myMotor2->run(BACKWARD); //Front Right
  myMotor3->run(FORWARD); //Back Left
  myMotor4->run(BACKWARD); //Back Right


  }
  // turn on motor
  myMotor1->run(RELEASE);
  myMotor2->run(RELEASE);
  myMotor3->run(RELEASE);
  myMotor4->run(RELEASE);

}
