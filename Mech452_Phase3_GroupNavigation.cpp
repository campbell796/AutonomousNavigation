/*
Group 6 Phase 3 Lab Code
Created by Fiona Ross and Hee Roh, 10/18/2018
Program written for single sharp on the left front corner of the robot.
Robot will try to remain a set distance from the wall (TARGET)
Gain was set to reach a desired speed for turning and wall following and account for deadband
Main program was put in a do-while loop so that it could be repeated with push of PBA button
*/

// ==========================
// Initialization
//==========================
#include <SoftwareSerial.h>
// LED Pin Assignments
int RED = 4;
int YLW = 5;
int GRN = 6;

/*
Button Pin Assignments used to start program, calibrate CMUcam sensor and initialize subroutines
*/
int BUTTON_A = 7;
int BUTTON_B = 8;
int BUTTON_C = 9;

int MOTOR_L = 10;  // left motor signal
int MOTOR_R = 11;  // right motor signal
int BUMPER = 13; // bumpers at front of robot 
int SHARP_1 = A1;  //initializing sharp 1 placed at side of robot for all following 
int SHARP_2 = A2;  //initializing sharp 2 placed at front of robot for collision avoidance

//=========================
// global constants 
//==========================
const int STOP_SPEED = 145;  //pulse length for stopping motors
const int DELTA = 300;      //forward speed.   //may need to slow robot down 
const int TARGET1 = 1200;   //sharp for target distance,
const int TARGET2 = 1000; //sharp for target distance may be lower for front sharp //1200 is approximate (in mvolts)

// Navigation with proportional speed control
float GAIN = 0.1;

// CMUcam variable declaration for calibration 
int green_pixels; // declaring for green traffic light 
int green_confidence; //declaring for green traffic light 
int red_pixels; //declaring for red traffic light 
int red_confidence; //declaring for red traffic light 

//cmu cam serial set-up
SoftwareSerial cmucam(2, 3);  //pin 2: white, pin 3: blue

//==============================
//function declaration
//non-VOID sub functions need to be declared here.
//=========================================
boolean cmucam2_set(char* cmd);
boolean cmucam2_get(char* cmd);

//global variables
unsigned char rcv_data[8] = "";
/*
array of 8 chars (updated everytime cmucam2_get() is called)
Reminder that to see how rcv_data is arranged -- look at the cmucam2_get() function
*/
byte pixels;
byte confidence;

// ==========================================
//Set-UP
// the setup routine runs once when we press reset:
//===========================================
void setup() {
	// initialize the digital led pins as outputs.
	pinMode(RED, OUTPUT);
	pinMode(YLW, OUTPUT);
	pinMode(GRN, OUTPUT);

	//initialize buttons and bumper pins as inputs
	pinMode(BUMPER, INPUT);
	pinMode(BUTTON_A, INPUT);
	pinMode(BUTTON_B, INPUT);
	pinMode(BUTTON_C, INPUT);

	//initialize motor control pins as outputs
	pinMode(MOTOR_L, OUTPUT);
	pinMode(MOTOR_R, OUTPUT);

	// setting serial transfer rate at 9600 bits per second
	Serial.begin(9600);

	// initializing CMUcam sensor
	Serial.println("Initializing camera, one moment please.");
	cmucam.begin(9600);
	cmucam.print("RS");
	cmucam.print("\r");
	cmucam.print("RS");
	cmucam.print("\r");
	cmucam.listen();
	while (cmucam.available() > 0)
	{
		cmucam.read();
	}
	delay(100);
	while (!cmucam2_set("RS"));

	//configure camera LEDs  
	cmucam2_set("L0 1"); // green LED (on camera) on
	cmucam2_set("L1 0"); // red LED (on camera) off

	while (!cmucam2_set("CR 18 36"));
	/* 
	this is YCrCb mode so that the TV works before we press the button during debugging (switches back to RGB mode when we white balance and prepare vehicle for testing)
 */

	Serial.println("Initialization complete. Please wait while white balance");
	do {
		toggleLED(GRN);
		// Green LED flashing showing setup is complete
	} while (digitalRead(BUTTON_A) == HIGH);

	delay(500);

	//white balance CMUcam ie calibrating 
	Serial.println("Please wait while camera autoadjusts (yellow LED Solid)");
	while (!cmucam2_set("CR 18 44 19 33"));// RGB white balance on, autogain on
	for (int i = 0; i < 10; i++) // let the camera adjust for 5 seconds
	{
		turnOnLED(YLW);
		delay(500);
	}
	while (!cmucam2_set("CR 18 40 19 32"));
	/*
	RGB white balance off, auto gain off
	18 40 19 32 values were found using separate calibration code used to find RGB mean values, 
	RGB deviation values, pixel values and confidence values
	*/

	// set up the camera for color tracking
	while (!cmucam2_set("PM 1"));// set poll mode
	while (!cmucam2_set("RM 1"));// set raw mode on

	Serial.println("White balance complete. Program running (yellow LED flashing)");
	Serial.println("Press a button to start the program");
	do
	{
		toggleLED(YLW);
	} while (digitalRead(BUTTON_A) == HIGH);
	//setup serial debug
	Serial.begin(9600);

	delay(200);
}
//===========================================
// Main Program Loop
// the loop routine runs over and over again forever
//============================================
void loop() {
	do
	{
		Serial.println("Program Ready, Press PBA");
		toggleLED(GRN); // informs user program is ready     
	} while (digitalRead(BUTTON_A) == HIGH);
	/*
	this loop pauses program until PBA (push button A) button is pressed and allows main program to be repeated
	*/
	do
	{
		//---------------------------------------------------------
		//Get red traffic light reading 
		// ----------------------------------------------------------
		cmucam2_get("TC 210 240 97 157 16 46", 'T');
		// tracking Red pixels
		red_pixels = rcv_data[6]; //get pixel count
		red_confidence = rcv_data[7];   // get confidence count
		runMotors(0, 0); //vehicle stops if reading red light
		digitalWrite(RED, HIGH);  //put on red Led to show reading red light 
	} while (red_pixels > 3 && red_confidence > 2);
	// stops vehicle until CMUcam no longer reads red light
	do
	{

		// ---------------------------------------------
		//Get sharp reading from Sharp 1
		// -----------------------------------------------------
		int sensor1 = map(analogRead(SHARP_1), 0, 1023, 0, 5000); //map digital input to analog output 
		Serial.println("reading values");
		// account for error
		int error1 = TARGET1 - sensor1;
		// proportional control 
		int dummy1 = min(abs(error1)*GAIN, 100);
		int delta_v1 = DELTA * ((100 - dummy1) / 100);
		// Map analog input from 1023 counts to 5000 mvolts

		//---------------------------------------------
		//Get Sharp reading from Sharp 2
		//---------------------------------------------
		int sensor2 = map(analogRead(SHARP_2), 0, 1023, 0, 5000);
		//  Serial.println(sensor2); 
		//  int error2 = TARGET2-sensor2;
		//  int dummy2 = min(abs(error2)*GAIN,100);
		//  int delta_v2 = DELTA*((100-dummy2)/100);
		//  Map analog input from 1023 counts to 5000 mvolts 
		//  Controls for reading of Sharp 1

//==================================
// Logic Loop for wall following
//=================================
//---------------------------------------------------
// Check for pedestrians or other vehicles in front 
//---------------------------------------------------------
		if (sensor2 > 1800) //1800 is analog output
		{ //make sure nothing in front of vehicle 
			runMotors(0, 0); // stop if something in front of vehicle
			sensor2 = map(analogRead(SHARP_2), 0, 1023, 0, 5000); //take another distance reading 
			Serial.println(sensor2);
		}
		else
		{// use sharp 1 on side of robot for wall following
//-------------------------------------------------------------
// Adjust vehicle if too far from wall
//-------------------------------------------------------------
			if (error1 > -50 && error1 < 800) // too far from wall 
			{
				Serial.println("Too far from wall"); // debugging line  
				 //Serial.println(error1); 
	//output what the error is to see if where it is in set conditions…add to code when debugging 
			//digitalWrite(YLW,HIGH); 
	//yellow LED indicates too far from wall..put in when testing
				digitalWrite(RED, LOW);
				//turns off red Led if was stopped at traffic light before
				runMotors(delta_v1, DELTA);
				//run left and right motors so turns closer to wall
			}
			//-------------------------------------------------------
			// Adjust vehicle if too close to wall
			//------------------------------------------------------      
			else if (error1 < -600) // too close to wall
			{
				//    sensor1 = map(analogRead(SHARP_1), 0, 1023, 0, 5000);
				//    error1 = TARGET1-sensor1;
	/*
	 taking another sensor and error reading made vehicle too slow so taking this out of code for now
	*/
				Serial.println("too close to wall");
				// add this line when debugging
							//    Serial.println(error1);
				// add this line when debugging 
				digitalWrite(GRN, HIGH);
				//add this line when testing to visually see if too close to wall
				digitalWrite(RED, LOW);
				//turn off red led if previously was stopped at red light
				runMotors(DELTA, delta_v1);
				// adjust L and R motor to turn away from wall
			}
			//------------------------------------------------------
			// Adjust vehicle position if loses wall
			// ie does the vehicle need to turn corner 
			// and go down another street or over bridge
			//--------------------------------------------------      
			else if (error1 >= 600) //loses the wall
			{
				Serial.println("Corner");
				// This is a debugging line 
				do
				{
					/*
					This cornering loop continuously takes sharp on side of vehicle reading and turns vehicle until sharp can regain the wall
					*/
					sensor1 = map(analogRead(SHARP_1), 0, 1023, 0, 5000); //sensor reading 
					error1 = TARGET1 - sensor1; // sensor calc
					dummy1 = min(abs(error1)*GAIN, 100); // sensor calc
					delta_v1 = DELTA * ((100 - dummy1) / 100); //sensor calc
					runMotors(delta_v1, DELTA); // this turns vehicle
				} while (error1 >= -100);    //until wall reacquired
//-------------------------------------------------
// After turn corner check for red stop light
//-----------------------------------------------------     
				cmucam2_get("TC 210 240 97 157 16 46", 'T');  // check for red light    
// need to recalibrate 
				red_pixels = rcv_data[6];
				red_confidence = rcv_data[7]
					if (red_pixels >= 5 && red_confidence >= 4)      // make this non zero by getting pixel and confidence numbers 
					{
						do {
							Serial.println("RED LIGHT");
							// this is a debugging line 
							cmucam2_get("TC 210 240 97 157 16 46", 'T');
							//update camera reading 
							red_pixels = rcv_data[6];
							red_confidence = rcv_data[7];
							// stop vehicle if read red
							runMotors(0, 0);
						} while (red_pixels >= 5 && red_confidence >= 4);
						// continue getting camera inputs until no longer read red
					}
				//--------------------------------------------------
				// Read for green light 
				//--------------------------------------------------
				//update camera reading 
				cmucam2_get("TC 56 116 207 240 206 240", 'T');//check for green
				green_pixels = rcv_data[6];
				green_confidence = rcv_data[7];
				// stop robot so can get clear camera reading
				runMotors(0, 0);
				delay(1000);
				// -------------------------------------------------
				// Green Light Routine 1 – need to test which works best 
				//with other vehicles/groups 
				//For this task Green light = parking routine / get out of way for another vehicle
				// Car will remain parked until button A pressed 
				// Uses delay() 
				//-------------------------------------------------
				// set green light conditions 
				if (green_pixels > 3 && green_confidence > 100)
				{
					// vehicle move forward 
					runMotors(DELTA, DELTA);
					delay(200);
					// debugging line 
					Serial.println("Green LIGHT");
					//vehicle turns out of way 
					runMotors(-DELTA, DELTA);
					delay(1600);
					//this loop stops the vehicle until button is pressed
					do {
						runMotors(0, 0);
					} while (digitalRead(BUTTON_A) == HIGH);
					//------------------------------------------------
					// Green Light Routine 2 – no parking 
					// moves backwards while taking green light reading 
					// too slow and doesn’t work w/ approaching vehicles
					// no delay() so faster…will this work with other vehicles?
					//---------------------------------------------------------
					/*
											do{
						//get camera reading for green light
					cmucam2_get("TC 28 88 201 240 210 240", 'T');
					green_pixels = rcv_data[6];
					green_confidence = rcv_data[7];
					//vehicle moves backwards but still reads green value
										runMotors(-DELTA,-DELTA);
					//       }while(green_pixels < 3 && green_pixels >0 && green_confidence > 0 );
									 }
								}
								else
								{
					//debugging tool when testing
									 digitalWrite(GRN,HIGH);
					//robot moves forward again
									 runMotors(DELTA,DELTA);
								}
							}
					// ===========================================
					/*
					 the entire program will run until bumper is hit indicating we have messed up by either hitting pedestrian (Yikes! Casualty Alert!!) or another Car (Goodbye low insurance Costs!!)
					*/
					//==============================================
				}while (digitalRead(BUMPER) == LOW);
			}



			//**********SUBROUTINES******************
			//=======================================
			//CMUCam Functions
			// Refer back to resources for help
			//======================================
			boolean cmucam2_set(char* cmd)
			{
				// send the command
				cmucam.print(cmd);
				cmucam.print("\r");
				cmucam.listen();
				delay(10);  //give the camera time to respond
				boolean ack = false;
				// get the response
				while (cmucam.available() > 0)
				{
					char inbyte = cmucam.read();
					if (inbyte == ':')
						ack = true;
				}
				// flush
				while (cmucam.available() > 0)
					cmucam.read();
				return ack;
			}

			/*
			 * Function for sending commands to the CMU Cam2
			 * where return data is expected.
			 */
			boolean cmucam2_get(char* cmd, char packet)
			{
				// send the command
				cmucam.print(cmd);
				cmucam.print("\r");
				cmucam.listen();

				delay(5);  //give camera time to respond
				// S-Packet
				if (packet == 'S')
				{
					while (cmucam.read() != 255); // wait for signal byte
					while (cmucam.read() != 83);
					while (cmucam.available() < 6); // wait for data
					for (int i = 0; i < 6; i++) // read the packet
					{
						rcv_data[i] = cmucam.read();
					}
				}
				// T-Packet
				if (packet == 'T')
				{
					while (cmucam.read() != 255); // wait for signal byte
					while (cmucam.read() != 84);
					while (cmucam.available() < 8); // wait for data
					for (int i = 0; i < 8; i++) // read the packet
					{
						rcv_data[i] = cmucam.read();
					}
				}

				/*
				How rcv_data will be organized depends on the type of packet received (i.e. S or T)

				rcv_data is an array of 8 unsigned chars (index from 0 to 7):

				rcv_data index: |  0  |  1  |  2  |  3  |  4  |  5  |  6  |  7  |
				if S packet:    |Rmean|Gmean|Bmean|Rdevi|Gdevi|Bdevi| \r  |rand |
				if T packet:    |  mx |  my |  x1 |  y1 |  x2 |  y2 |pixel|confd|
				*/

				// flush
				while (cmucam.available() > 0)
					cmucam.read();
				return true;
			}
			//================================
			// Single LED on Function
			//================================
			//Turn on a single LED, and all other off
			void turnOnLED(int colour) {
				digitalWrite(GRN, LOW);
				digitalWrite(YLW, LOW);
				digitalWrite(RED, LOW);
				digitalWrite(colour, HIGH);
			}
			//=================================
			//LED Toggle Function
			//==================================
			//Toggle an LED on/off
			void toggleLED(int colour) {
				digitalWrite(colour, HIGH);
				delay(250);
				digitalWrite(colour, LOW);
				delay(250);
			}
			//======================================
			//LED Flash Funciton
			//======================================
			//flash all LEDs
			void flashAllLEDs() {
				digitalWrite(GRN, LOW);
				digitalWrite(YLW, LOW);
				digitalWrite(RED, LOW);
				delay(250);
				digitalWrite(GRN, HIGH);
				digitalWrite(YLW, HIGH);
				digitalWrite(RED, HIGH);
				delay(250);
			}
			//========================================
			// Run Motors Function 
			//========================================
			void runMotors(int delta_L, int delta_R)
			{
				int pulse_L = (STOP_SPEED + delta_L) * 10;  //determines length of pulse in microsec
				int pulse_R = (STOP_SPEED + delta_R) * 10;
				for (int i = 0; i < 3; i++) {
					pulseOut(MOTOR_L, pulse_L);    //send pulse to left motors
					pulseOut(MOTOR_R, pulse_R);    //send pulse to right motors
				}
			}
			void pulseOut(int motor, int pulsewidth)
			{
				digitalWrite(motor, HIGH);
				delayMicroseconds(pulsewidth);  //send pulse of desired pulsewidth      
				digitalWrite(motor, LOW);
			}

			//=========================================
			// PlotLite Function Used to graph Sharp voltage output
			// Saved this for debugging tool but not included in routine
			//=============================================
			//void plotLite(int value)  {
			//    while((millis() - plot_time) <= 50);  //wait until 50 msecs has passed (since the last point was plotted)
			//    Serial.print(value);     //output to plotlite         
			//    Serial.write(13);        //carriage return
			//    plot_time = millis();    //update the last plot time (global variable)
			//}



