/*	MExo Digi Controller
	[INFO]
	This BT Serial communication program is designed for
	Arduino Mega2560 boards and Bluetooth modules by 
	Serial and Serial1.

	[Hardware Setup]
	BT Serial: Serial1
	BT BAUD: 115200
	BT RX 		=> 		18 TX
	BT TX 		=> 		19 RX
	BT STATE 	=>		47 I/O

	Nano Serial: Serial2
	Nano BAUD: 115200
	Nano TX 	=> 		19 RX

	2.4G Serial: Serial3
	2.4G BAUD: 9600
	2.4G RX 	=> 		14 TX
	2.4G TX 	=> 		15 RX
	2.4G STATE 	=>		32 I/O

	MPU-0 AD0 	=> 		2  Left  side
	MPU-1 AD0 	=> 		3  Right side
	MPU-2 AD0 	=> 		45 Back  side
	
	LEDs Sig    =>		8  Pin
	Buzzer Sig  =>		11 PWM

	[BT Connection Solution]
	Keep sending the command "AT+SENM=1,0/r/n" at very begining
	until the BT_STATE_PIN goes HIGH and then run others.
	(AT button on BT module should be pressed for at least 5s untill be released)
	Finally, power on the robot, the connection will setup automatically.

	[Software Setup]
	Seprated Timing Structure is applied to this programme by including PollTimer.h
	It helps to scan different loops with different frequencies.
	[Do Not] use "Delay(ms);" to change the frequency which will prolong the loop
	and missing High Frequent Signals in other loops (e.g. Serial receive).
	If 1Hz is needed, please add counters in the loop and check the counters.

	[Updates]
	[2017-02-28] MExo_10.ino	Add WatchDog & ASM process to prevent block and restart automatically	
	[2017-03-16] MExo_12.ino	Change AD0 Pins to 2 [Left-Arm] and 3 [Right-Arm] to accommadate Arduino UNO 
	[2017-03-17] MExo_13.ino	Add Serial3 to work with 2.4G Serial module
								Add LobotServoController_1.h to work with Lobot Servo Board
	[2017-03-17] MExo_14.ino	Add XiaoWei Fuction for XiaoWei Robot
	[2017-03-26] MExo_15.ino	Add the 3rd MPU6050's AD0 to Pin 45
								Move BT_State from Pin 25 to Pin 47
	[2017-03-27] MExo_16.ino	Change I2C Clock from 400kHz to 380kHz for three DMPs, but the 3rd dosen't work
								Change Serial3 Baud from 9600 to 115200bps for better performance
	[2017-03-28] MExo_17.ino	Modify parameters for XiaoWei robot control
	[2017-03-29] MExo_18.ino	Remove the 3rd MPU6050, changed I2C Clock back to 400kHz, and inproved XiaoWei parameters
	[2017-03-31] MExo_19.ino	Update XiaoWei output values

	Last updated by Henry Chen on March 29th 2017.
	============================================
	MExo code is placed under the MIT license
	Copyright (c) 2017 MOOCXING Shanghai

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
	===============================================
*/

#include <Wire.h>		// Library for I2C communication
#include <I2Cdev.h>		// Library for I2C communication
#include <math.h>		// Library for Maths calculation
#include <avr/wdt.h>	// Library for WatchDog
#include "PollTimer.h"	// Library for Multi-processing
#include "WiiChuck_2.h"	// Library for WiiChuck
#include "MPU6050_6Axis_MotionApps20_10.h"	// Library for MPU6050 DMP
#include "MPU6050_Wrapper_3.h"				// Library for MPU6050 functions
#include "LobotServoController_1.h"			// Library for Lobot Servo Board control
#include <Adafruit_NeoPixel.h>				// Library for LED strip-WS2812

//------------------------------------------ [Motion] MPU6050
#define AD0_PIN_0 2  	// 2  Left side.  Connect to AD0 pin on MPU #0 
#define AD0_PIN_1 3  		// 3  Right side. Connect to AD0 pin on MPU #1
// #define AD0_PIN_2 45  	// 45 Connect to AD0 pin on MPU #2	[Only 2 MPUs can work well with WiiChuck. Problem unknow!]
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69 (Selected for output values)
//------------------------------------------ [WatchDog]	Mega2560	
#define TIMEOUT WDTO_2S // Wait for about 2000ms, it depends on working voltage
//------------------------------------------ [Bluetooth] HC05
#define BT_STATE_PIN 47 // Connect BT_STATE
#define LED_PIN 	 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
//------------------------------------------ [WiiChuck] NunChuck
#define MAXANGLE 90
#define MINANGLE -90
//------------------------------------------ [LED Strip] WS2812
#define LED_PIN 8


PollTimer SerialLoop(100UL); 	// 100Hz to refresh Serial Crossing Communication
PollTimer BT_StateLoop(1UL); 	// 1Hz to refresh, actually 1/BT_Counter_Num Hz to reconnect BT
PollTimer WiiLoop(30UL); 		// 30Hz to read WiiChuck data
PollTimer MPU6050Loop(30UL); 	// 180Hz to read MUP6050 data (Qualified to stay FIFO clean & avoid death)
PollTimer MaintenanceLoop(10UL);// 30Hz to send the Serial Output data

LobotServoController myse;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(5, LED_PIN, NEO_GRB + NEO_KHZ800);
// Main Vars Setup
// Output Control vars
bool Alpha_Output = true;
// bool Unity_Output = true;
// bool XiaoWei_Output = true;
// bool Alpha_Output = false;
bool Unity_Output = false;
bool XiaoWei_Output = false;
// BT & Serial
bool BT_State = false;
// bool BT_Flag = true;
bool BT_Flag = false; 
// MPU control/Status vars
const bool useSecondMpu = true;
MPU6050_Array mpus(useSecondMpu ? 2 : 1);
// MPU6050_Array mpus(useSecondMpu ? 3 : 1);	// Need to replace the Right Arm MPU
	bool blinkState = false;// Blink after each DMP processing
	bool dmpReady = false;  // set true if DMP init was successful
	uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
	uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
	uint16_t fifoCount;     // count of all bytes currently in FIFO
	uint8_t fifoBuffer[64]; // FIFO storage buffer
	uint8_t Commander_Serial[64]; 
	uint8_t Commander_Serial1[64]; 	
	uint8_t Commander_Serial3[64];
	uint8_t Commander_Check = 0;
	// Bluetooth control/Status vars
	int BT_Counter_Num = 3;
	int BT_Counter = BT_Counter_Num;
	String Command_Setup = ("AT+SENM=1,0\r\n");
	String inString_0 = "";         // a string to hold incoming data
	String inString_1 = ""; 
	String inString_2 = ""; 
	String inString_3 = ""; 
	bool stringComplete_0 = false;  // whether the string is complete
	bool stringComplete_1 = false;  // whether the string is complete
	// WiiChuck vars
	WiiChuck chuck = WiiChuck();
	int Joy_DeadZone = 30;
	int Joy_X = 0;
	int Joy_Y = 0;
	int Joy_C = 0;
	int Joy_Z = 0;
	int Wii_Status = 0;
	int Wii_Joystick = 0;
	int Wii_Button = 0;
	// Buzzer vars
	int pinBuzzer = 11;
	long Buzzer_fq = 3000;
	// Resisitor vars
	const int analogInPin_L = A14;  // Analog input pin that the potentiometer is attached to
	const int analogInPin_R = A13;  // Analog input pin that the potentiometer is attached to
	float ResisitorValue_L = 0;       // value read from A14 pot
	float ResisitorValue_R = 0;       // value read from A13 pot
	float Angle_L_temp = 0;
	float Angle_R_temp = 0;
	float Angle_L_Unity = 0;			// Angle of left  elbow for Unity
	float Angle_R_Unity = 0;			// Angle of right elbow for Unity
	float Angle_L_Alpha = 0;			// Angle of left  elbow for Alpha
	float Angle_R_Alpha = 0;			// Angle of right elbow for Alpha
	float Angle_R_XiaoWei = 0;				// Angle of left  elbow for XiaoWei
	float Angle_L_XiaoWei = 0;				// Angle of right elbow for XiaoWei
	const int analogInPin_LT = A12; // Analog input pin that the potentiometer is attached to
	const int analogInPin_RT = A11; // Analog input pin that the potentiometer is attached to
	float ResisitorValue_LT = 0;      // value read from A14 pot
	float ResisitorValue_RT = 0;      // value read from A13 pot
	float Angle_LT_temp = 0;
	float Angle_RT_temp = 0;
	float Angle_LT_Unity = 0;			// Angle of left  elbow for Unity
	float Angle_RT_Unity = 0;			// Angle of right elbow for Unity
	float Angle_LT_Alpha = 0;			// Angle of left  elbow for Alpha
	float Angle_RT_Alpha = 0;			// Angle of right elbow for Alpha
	float Angle_RT_XiaoWei = 0;			// Angle of left  elbow for XiaoWei
	float Angle_LT_XiaoWei = 0;			// Angle of right elbow for XiaoWei
	// Orientation/Motion vars
	Quaternion q;           // [w, x, y, z]         quaternion container
	VectorInt16 aa;         // [x, y, z]            accel sensor measurements
	VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
	VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
	VectorInt16 gyro;    // [x, y, z]            world-frame accel sensor measurements
	VectorFloat gravity;    // [x, y, z]            gravity vector
	float euler[3];         // [psi, theta, phi]    Euler angle container
	float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
	float jiaodu[12];           // gravity, angles of different surfaces & quaternion
	uint16_t FIFO_Count = 0;
	uint16_t FIFO_0 = 0;
	uint16_t FIFO_1 = 0;
	uint16_t FIFO_2 = 0;
	//----------------[Motion_0-Left]
	float q_w_0 = 0;
	float q_x_0 = 0;
	float q_y_0 = 0;
	float q_z_0 = 0;
	float g_x_0 = 0;
	float g_y_0 = 0;
	float g_z_0 = 0;
	float a_x_0 = 0;
	float a_y_0 = 0;
	float a_z_0 = 0;
	float a_yz_0 = 0;
	float a_s_cos_0 = 0;
	float a_s_tan_0 = 0;
	float a_s_delta_0 = 0;
	float a_s_0 = 0;
	int a_s_sign_0 = 0;
	float a_w_cos_0 = 0;
	float a_w_tan_0 = 0;
	float a_w_delta_0 = 0;
	float a_w_product_0 = 0;
	float a_w_0 = 0;
	float e_0_0 = 0;
	float e_1_0 = 0;
	float e_2_0 = 0;
	float e_s_0 = 0;
	float e_t_0 = 0;
	int e_t_sign_0 = 0;
	float e_w_0 = 0;
	float qa_a_0 = 0;
	float qa_x_0 = 0;
	float qa_y_0 = 0;
	float qa_z_0 = 0;
	float qa_sqrt_0 = 0;
	//----------------[Motion_1-Right]
	float q_w_1 = 0;
	float q_x_1 = 0;
	float q_y_1 = 0;
	float q_z_1 = 0;
	float g_x_1 = 0;
	float g_y_1 = 0;
	float g_z_1 = 0;
	float a_x_1 = 0;
	float a_y_1 = 0;
	float a_z_1 = 0;
	float a_yz_1 = 0;
	float a_s_cos_1 = 0;
	float a_s_tan_1 = 0;
	float a_s_delta_1 = 0;
	float a_s_1 = 0;
	int a_s_sign_1 = 0;
	float a_w_cos_1 = 0;
	float a_w_tan_1 = 0;
	float a_w_delta_1 = 0;
	float a_w_product_1 = 0;
	float a_w_1 = 0;
	float e_0_1 = 0;
	float e_1_1 = 0;
	float e_2_1 = 0;
	float e_s_1 = 0;
	float e_t_1 = 0;
	int e_t_sign_1 = 0;
	float e_w_1 = 0;
	//----------------[Motion_2-Back]
	float a_t_2 = 0;

	//----------------[Motion_vars]
	float Motion_var_0 = 0;
	float Motion_var_1 = 0;
	float Motion_var_2 = 0;
	float Motion_var_3 = 0;
	float Motion_var_4 = 0;
	float Motion_var_5 = 0;
	float Motion_var_6 = 0;
	float Motion_var_7 = 0;
	float Motion_var_8 = 0;


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // configure Buzzer and LED for BT & MPU6050 status 
    pinMode(pinBuzzer, OUTPUT);
    pinMode(BT_STATE_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Helps to fix the I2C self-block a lot for at most 2 DMPs.
    // Wire.setClock(300000); // 300kHz I2C clock fixs the I2C self-block a lot for 3 DMPs.
    // Wire.setClock(380000); // Tested for 2 to work among 3 DMPs



    // initialize serial communication
	Serial.begin(115200);	// 0(RX)and  1(TX)
    Serial1.begin(115200); 	//19(RX)and 18(TX)
    Serial3.begin(115200); 	//15(RX)and 14(TX)
    Serial.println(	"Hello World! --From Serial");

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    tone(pinBuzzer, Buzzer_fq);

    mpus.add(AD0_PIN_0);
    mpus.add(AD0_PIN_1);
    // mpus.add(AD0_PIN_2);

    wdt_enable(TIMEOUT);  	// WatchDog starts here to detact initializing failures

    mpus.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    if (mpus.testConnection()) {
		Serial.println(F("MPU6050 connection successful"));
	} 
	else {
		mpus.halt(F("MPU6050 connection failed, halting"));
	}

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    mpus.dmpInitialize();
    mpus.programDmp(0);	/////////////////////////////// Something goes wrong here!
    mpus.programDmp(1);
    // mpus.programDmp(2); 
   /////////////////////////////// Detach befor MPUs begin & Attach the CLK Pin of WiiChuck
   	// delay(10);
    chuck.begin();
 //    delay(10);
	// chuck.update();
	// delay(10);
	// chuck.calibrateJoy();

    // Start every loop's timer
    SerialLoop.start();
    BT_StateLoop.start();
    WiiLoop.start();
    MPU6050Loop.start();
    MaintenanceLoop.start();
    noTone(pinBuzzer);

   //  strip.begin();
  	// strip.show(); // Initialize all pixels to 'off'
}

//------------------------------------------7
void handleMPUevent(uint8_t mpu) {

	MPU6050_Wrapper* currentMPU = mpus.select(mpu);
    // reset interrupt flag and get INT_STATUS byte
    currentMPU->getIntStatus();
    FIFO_Count = currentMPU->_fifoCount;

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((currentMPU->_mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT))
    	|| currentMPU->_fifoCount >= 512) {
    	// || currentMPU->_fifoCount >= 512) {
        // reset so we can continue cleanly
        currentMPU->resetFIFO();
        Serial.println(F("FIFO overflow!"));
        // return;
    }
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    if (currentMPU->_mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        // read and dump a packet if the queue contains more than one
        while (currentMPU->_fifoCount >= 2 * currentMPU->_packetSize){
            // read and dump one sample
            // Serial.print("DUMP"); // this trace will be removed soon
            currentMPU->getFIFOBytes(fifoBuffer);
        }
        // read a packet from FIFO and process the data
        currentMPU->getFIFOBytes(fifoBuffer);
        currentMPU->_mpu.dmpGetQuaternion(&q, fifoBuffer);
        currentMPU->_mpu.dmpGetEuler(euler, &q);
        currentMPU->_mpu.dmpGetGravity(&gravity, &q);
        currentMPU->_mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        currentMPU->_mpu.dmpGetAngle(jiaodu, &q, &gravity);
        currentMPU->_mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        currentMPU->_mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
        currentMPU->_mpu.dmpGetGyro(&gyro, fifoBuffer);
        // Show the processing frequency by blinking
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
//------------------------------------------

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

	if(SerialLoop.check()){  // check if there are Serial messages, 400Hz
		// Check if Serial message comes
		if(BT_Flag != true){
			if (Serial.available() > 0) {
				int inChar_0 = Serial.read();
				inString_0 += (char)inChar_0;
				if (inChar_0 == '\n') {
					stringComplete_0 = true;	// set a flag for main loop
				}
			}

			if (Serial1.available() > 0) {
				int inChar_1 = Serial1.read();
				inString_1 += (char)inChar_1;
				if (inChar_1 == '\n') {
					stringComplete_1 = true;	// set a flag for main loop
				}			
			}
		}

	}

	if(BT_StateLoop.check()) {  // Keep connecting BT, 1/3Hz (BT_StateLoop/T_Counter_Num)Hz
		if(Alpha_Output == true){
			    BT_State = digitalRead(BT_STATE_PIN);	// If BT is connected, BT_STATE is HIGH and LED is on
			if (BT_State == HIGH) {
			    digitalWrite(LED_PIN, HIGH);
				BT_Flag = true;	// Start the MPU6050 output
			} 
			else {
				digitalWrite(LED_PIN, LOW);
				// BT_Flag = true;
				if(Unity_Output != true){
				  BT_Flag = false;  
				}
		    	BT_Counter --;
		    	if(BT_Counter < 1){
		    		Serial.println(F("Trying BT connection...Check BT_AT, BT_State & Robot_Power..."));
					Serial.print(Command_Setup);
					Serial1.print(Command_Setup); 
		    	    BT_Counter = BT_Counter_Num;	// reset counter
		    	}
	    	}
		}
		else{
			BT_Flag = true;
		}
		
	}

	if(WiiLoop.check()){  // check if there are WiiChuck, 80Hz
		chuck.update(); 
		if(chuck.readJoyX() > Joy_DeadZone){Joy_X = 7;}		// X axle right
		else if(chuck.readJoyX() < -Joy_DeadZone){Joy_X = 4;}	// X axle reft
		else{Joy_X = 0;}										// X axle middle
		if(chuck.readJoyY() > Joy_DeadZone){Joy_Y = 1;}		// Y axle forwards
		else if(chuck.readJoyY() < -Joy_DeadZone){Joy_Y = 2;}	// Y axle backwards
		else{Joy_Y = 0;}										// Y axle middle

		if(chuck.buttonC > 0){Joy_C = 10;}						// C button pressed
		else{Joy_C = 0;}										// C button released
		if(chuck.buttonZ > 0){Joy_Z = 20;}						// Z button pressed
		else{Joy_Z = 0;}										// Z button released

		Wii_Status = (Joy_X + Joy_Y + Joy_C + Joy_Z);			// Sum all Wii vars

		Wii_Joystick = Wii_Status % 10;
		Wii_Button = Wii_Status / 10;

		// [WiiChuck Value Map]	
		//  	
		// >>Joystick
		// 	       +Y
		//    -------------
		//    | 5   1   8 |
		// -X | 4  (0)  7 | +X
		//    | 6   2   9 |
		//    -------------
		// 	       -Y
		//
		// >>Button
		// 	      C    Z
		//    -------------
		//    |  (0)  [0] | Released
		//    |  10   20  |	Pressed
		//    -------------

	}

	if(MPU6050Loop.check()){  // check if it is time for next sensor sample, 120Hz
		if(BT_Flag == true){
			static uint8_t mpu = 0;
			static MPU6050_Wrapper* currentMPU = NULL;
			// for (int i=0;i<1;i++) {	// For 1 MPU
			for (int i=0;i<2;i++) {	// For 2 MPUs
			// for (int i=0;i<3;i++) { // For 3 MPUs
	            mpu = i;
	            currentMPU = mpus.select(mpu);
	            if (currentMPU->isDue()) {
	            	handleMPUevent(mpu);
	            	if(mpu==0){	// The Left Side
	            		g_x_0 = gyro.x;
	            		g_y_0 = gyro.y;
	            		g_z_0 = gyro.z;

	            		a_x_0 = jiaodu[0];
	            		a_y_0 = jiaodu[1];
	            		a_z_0 = jiaodu[2];
	            		a_yz_0 = jiaodu[7];

	            		a_s_cos_0 = jiaodu[3];
	            		a_s_tan_0 = jiaodu[4];
	            		a_s_delta_0 = a_s_cos_0 - a_s_tan_0;

	            		a_w_cos_0 = jiaodu[5];
	            		a_w_tan_0 = jiaodu[6];
	            		a_w_delta_0 = a_w_cos_0 - a_w_tan_0;
	            		a_w_product_0 = a_w_delta_0*a_z_0;

	            		q_w_0 = jiaodu[8];
	            		q_x_0 = jiaodu[9];
	            		q_y_0 = jiaodu[10];
	            		q_z_0 = jiaodu[11];

	            		if(fabs(a_w_product_0) >= 40){
	            		    asm volatile ("jmp 0");		// <ASM Process> lump to the start of programme
	            		}

	            		if(1){
	                    	//--------------------------------------ShenZhan
	                    	if(a_x_0 <= 0){
	                    		if( fabs(a_z_0) <= 0.2){
	                    			if(a_s_delta_0 <= 10){
	                    				a_s_0 = a_s_cos_0;
	                    			}
	                    			else{
	                    				a_s_0 = a_s_tan_0;
	                    			}
	                    		}
	                    		else{
	                    			a_s_0 = a_s_tan_0;
	                    		}
	                    		if(a_y_0 >=0){
	                    			if(a_z_0 > 0){
	                    				a_s_0 = -1*a_s_0;

	                    			}
	                    		}

	                    	}
	                    	else{
	                    		if( fabs(a_z_0) <= 0.2){
	                    			if(a_s_delta_0 <= 10){
	                    				a_s_0 = 180 - a_s_cos_0;
	                    			}
	                    			else{
	                    				a_s_0 = 180 - a_s_tan_0;
	                    			}
	                    		}
	                    		else{
	                    			a_s_0 = 180 - a_s_tan_0;
	                    		}

	                    	}
	                    	if(a_y_0 >= 0){
	                    		a_s_0 = -1*a_s_0;
	                    	}
	                    	if(a_s_0 < 0){
	                    		a_s_sign_0 = 1;
	                    	}
	                    	else{
	                    		a_s_sign_0 = 0;
	                    	}

							//--------------------------------------WaiZhan

		                    	if( fabs(a_z_0) <= 0.3){

		                    		if(a_yz_0 <= 0.25){
		                    			a_w_0 = a_w_0 + (g_x_0 / 13);	
		                    			a_w_0 = fmax(a_w_0 , a_w_tan_0);
		                    			a_w_0 = fmin(a_w_0 , a_w_cos_0);
		                    		}
		                    		else{
		                    			a_w_0 = a_w_tan_0;
		                    		}
		                    	}
		                    	else{
		                    		a_w_0 = a_w_cos_0;
		                    	}
		                    }
	                    FIFO_0 = FIFO_Count;
	                }

	                if(mpu==1){	// The Right Side

	              		e_0_1 = euler[0];
	            		e_1_1 = euler[1];
	            		e_2_1 = euler[2];

	                	g_x_1 = gyro.x;
	                	g_y_1 = gyro.y;
	                	g_z_1 = gyro.z;

	                	a_x_1 = jiaodu[0];
	                	a_y_1 = jiaodu[1];
	                	a_z_1 = jiaodu[2];
	                	a_yz_1 = jiaodu[7];

	                	a_s_cos_1 = jiaodu[3];
	                	a_s_tan_1 = jiaodu[4];
	                	a_s_delta_1 = a_s_cos_1 - a_s_tan_1;

	                	a_w_cos_1 = jiaodu[5];
	                	a_w_tan_1 = jiaodu[6];
	                	a_w_delta_1 = a_w_cos_1 - a_w_tan_1;
	                	a_w_product_1 = a_w_delta_1*a_z_1;

	                	q_w_1 = jiaodu[8];
	            		q_x_1 = jiaodu[9];
	            		q_y_1 = jiaodu[10];
	            		q_z_1 = jiaodu[11];

	            		if(fabs(a_w_product_1) >= 40){
	            		    asm volatile ("jmp 0");		// <ASM Process> lump to the start of programme
	            		}
	                	if(1){
	                    	//--------------------------------------ShenZhan
	                    	if(a_x_1 <= 0){
	                    		if( fabs(a_z_1) <= 0.2){
	                    			if(a_s_delta_1 <= 10){
	                    				a_s_1 = a_s_cos_1;
	                    			}
	                    			else{
	                    				a_s_1 = a_s_tan_1;
	                    			}
	                    		}
	                    		else{
	                    			a_s_1 = a_s_tan_1;
	                    		}
	                    		if(a_y_1 <0){
	                    			if(a_z_1 > 0){
	                    				a_s_1 = -1*a_s_1;

	                    			}
	                    		}

	                    	}
	                    	else{
	                    		if( fabs(a_z_1) <= 0.2){
	                    			if(a_s_delta_1 <= 10){
	                    				a_s_1 = 180 - a_s_cos_1;
	                    			}
	                    			else{
	                    				a_s_1 = 180 - a_s_tan_1;
	                    			}
	                    		}
	                    		else{
	                    			a_s_1 = 180 - a_s_tan_1;
	                    		}

	                    	}
	                    	if(a_y_1 < 0){
	                    		a_s_1 = -1*a_s_1;
	                    	}
	                    	if(a_s_1 < 0){
	                    		a_s_sign_1 = 1;
	                    	}
	                    	else{
	                    		a_s_sign_1 = 0;
	                    	}

							//--------------------------------------WaiZhan
		                    // if(a_y_1 <= 0){

		                    	if( fabs(a_z_1) <= 0.3){

		                    		if(a_yz_1 <= 0.25){
		                    			a_w_1 = a_w_1 + (g_x_1 / 13);	
		                    			a_w_1 = fmax(a_w_1 , a_w_tan_1);
		                    			a_w_1 = fmin(a_w_1 , a_w_cos_1);
		                    		}
		                    		else{
		                    			a_w_1 = a_w_tan_1;
		                    		}
		                    	}
		                    	else{
		                    		a_w_1 = a_w_cos_1;
		                    	}
		                    }
	                    FIFO_1 = FIFO_Count;
	                }
	            }
	        }			
		}
    }

	if(MaintenanceLoop.check()) {   // only display data 40Hz per second
			// Send Serial message if string is complete
		if(BT_Flag != true){	// Exchange Serial data
			if (stringComplete_0 == true) {	
				Serial1.print(inString_0);
				// Serial.print(inString_0);
				stringComplete_0 = false;
				inString_0 = "";
			}
			if (stringComplete_1 == true) {	
				Serial.print(inString_1);
				stringComplete_1 = false;
				inString_1 = "";
			}   
		}

		if(BT_Flag == true){
			// Read Elbow Angles from resisitors
			ResisitorValue_L = analogRead(analogInPin_L);
			ResisitorValue_R = analogRead(analogInPin_R);

			Angle_L_Unity = map(ResisitorValue_L, 0, 1023, 0, 180);
			Angle_R_Unity = map(ResisitorValue_R, 0, 1023, 0, 180);


			if(fabs(Angle_L_temp - Angle_L_Unity)>1) 
			{
			    Angle_L_Alpha = 180 - Angle_L_Unity -30;
			    Angle_L_temp = Angle_L_Unity;
			}

			if(fabs(Angle_R_temp - Angle_R_Unity)>1) 
			{
			    Angle_R_Alpha = 180 - Angle_R_Unity -70;
			    Angle_R_temp = Angle_R_Unity;
			}

			Angle_L_Alpha = constrain(Angle_L_Alpha, 0, 180);
			Angle_R_Alpha = constrain(Angle_R_Alpha, 0, 180);

			// Angle_L_XiaoWei = Angle_L_Alpha;
			// Angle_R_XiaoWei = Angle_R_Alpha;
		
			// Read Arm Turning Angles from resisitors
			ResisitorValue_LT = analogRead(analogInPin_LT);
			ResisitorValue_RT = analogRead(analogInPin_RT);

			Angle_LT_Unity = map(ResisitorValue_LT, 0, 1023, 0, 180);
			Angle_RT_Unity = map(ResisitorValue_RT, 0, 1023, 0, 180);

			// Angle_LT_temp = map(ResisitorValue_LT, 0, 1023, 0, 900);
			// Angle_LT_temp = map(ResisitorValue_RT, 0, 1023, 0, 900);

			// if(fabs(Angle_LT_temp - Angle_LT_XiaoWei)>2) 
			// {
			//     Angle_LT_XiaoWei = Angle_LT_temp;
			// }

			// if(fabs(Angle_RT_temp - Angle_RT_XiaoWei)>2) 
			// {
			//     Angle_RT_XiaoWei = Angle_RT_temp;
			// }

			// if(fabs(Angle_LT_temp - Angle_LT_Unity)>1) 
			// {
			//     Angle_LT_Alpha = 180 - Angle_LT_Unity -30;
			//     Angle_LT_temp = Angle_LT_Unity;
			// }

			// if(fabs(Angle_RT_temp - Angle_RT_Unity)>1) 
			// {
			//     Angle_RT_Alpha = 180 - Angle_RT_Unity -70;
			//     Angle_RT_temp = Angle_RT_Unity;
			// }

			// Angle_LT_Alpha = constrain(Angle_LT_Alpha, 0, 180);
			// Angle_RT_Alpha = constrain(Angle_RT_Alpha, 0, 180);

			// Angle_LT_XiaoWei = Angle_LT_Alpha;
			// Angle_RT_XiaoWei = Angle_RT_Alpha;

			if(Alpha_Output == true){
				switch (Wii_Button) {
				    case 0:	// Both C & Z button are released
						switch (Wii_Joystick) {
						    case 1:	// Up
						      command_Aplah(109,102);
						      break;
						    case 2: // Down
						      command_Aplah(109,98);
						      break;
						    case 4:	// Left
						      command_Aplah(109,108);
						      break;
						    case 7: // Right
						      command_Aplah(109,114);
						      break;
						}

				      break;
				    case 1:	// Only C button is pressed
						switch (Wii_Joystick) {
						    case 1:	// Up
						      command_Aplah(102,102);
						      break;
						    case 2: // Down
						      command_Aplah(102,98);
						      break;
						    case 4:	// Left
						      command_Aplah(116,108);
						      break;
						    case 7: // Right
						      command_Aplah(116,114);
						      break;
						}
				      break;
				    case 2:	// Only Z button is pressed     //Val    Tag    Note
						Commander_Serial1[0] = 0xFB;    	//251	 H-1	Header-1
						Commander_Serial1[1] = 0xBF;    	//191	 H-2	Header-2
						Commander_Serial1[2] = 0x18;    	// 24	 Len	Sum of Command byte quantity without Ender
						Commander_Serial1[3] = 0x23;    	// 35  	 Com    [Multi Servos Control]
						Commander_Serial1[4] = fabs(a_s_1);	// 90	[ 1]	Right-Back & forth
						Commander_Serial1[5] = a_w_1;		//  0	[ 2]	Right-Up & down
						Commander_Serial1[6] = Angle_R_Alpha;		// 90	[ 3]	Right-Elbow
						Commander_Serial1[7] = 180 - fabs(a_s_0);	// 90	[ 4]	 Left-Back & forth
						Commander_Serial1[8] = 180 - a_w_0;			//180	[ 5]	 Left-Up & down
						Commander_Serial1[9] = Angle_L_Alpha;		// 90	[ 6]	 Left-Elbow
						Commander_Serial1[10] = 0x5A;  		// 90	[ 7]	Right-Leg-U
						Commander_Serial1[11] = 0x3C;   	// 60	[ 8]	Right-Leg-MU
						Commander_Serial1[12] = 0x4C;   	// 76	[ 9]	Right-Leg-M
						Commander_Serial1[13] = 0x6E;   	//110	[10]	Right-Leg-ML
						Commander_Serial1[14] = 0x5A;   	// 90	[11]	Right-Leg-L
						Commander_Serial1[15] = 0x5A;   	// 90	[12]	 Left-Leg-U
						Commander_Serial1[16] = 0x78;   	//120	[13]	 Left-Leg-MU
						Commander_Serial1[17] = 0x68;   	//104	[14]	 Left-Leg-M
						Commander_Serial1[18] = 0x46;   	// 70	[15]	 Left-Leg-ML
						Commander_Serial1[19] = 0x5A;   	// 90	[16]	 Left-Leg-L
						Commander_Serial1[20] = 0x05;   	// 30	Time	Time for servo movement
						Commander_Serial1[21] = 0x01;   	//  0 	 W-1    Wait period-High for next command
						Commander_Serial1[22] = 0x20;   	// 20 	 W-2    Wait period-Low for next command
						Commander_Serial1[23] = check_Alpha(Commander_Serial1, 2, 22);	//  ~  Check    (Len+Com+[1]+..+[n])%256 - 1
						Commander_Serial1[24] = 0xED;   	//237	 End    Ender	

						// Serial.write(Commander_Serial1, 25); // Send data to Alpha via Serial1 [BT]
						Serial1.write(Commander_Serial1, 25); // Send data to Alpha via Serial1 [BT]  
				      break;
				    case 3:	// Both C & Z button are pressed//Val    Tag    Note						
						Commander_Serial1[0] = 0xFB;    	//251	 H-1	Header-1
						Commander_Serial1[1] = 0xBF;    	//191	 H-2	Header-2
						Commander_Serial1[2] = 0x06;    	//  6	 Len	Sum of Command byte quantity without Ender
						Commander_Serial1[3] = 0x01;    	//  1  	 Com    Default Servo Position and Sound
						Commander_Serial1[4] = 0x00;		//  0	[ 1]	Undefined
						Commander_Serial1[5] = 0x07;   		//  ~  Check    (Len+Com+[1]+..+[n])%255
						Commander_Serial1[6] = 0xED;   		//237	 End    Ender	

						// Serial.write(Commander_Serial1, 7); // Send data to Alpha via Serial1 [BT]
						Serial1.write(Commander_Serial1, 7); // Send data to Alpha via Serial1 [BT]	
				      break;
				    // default:
				      // do something
				}
			}

			if(Unity_Output == true){
		        Commander_Serial[0]  = 0xFB;    //251
		        Commander_Serial[1]  = 0xBF;    //191
		        Commander_Serial[2]  = Angle_L_Unity;    //24
		        Commander_Serial[3]  = Angle_R_Unity;    //35
		        Commander_Serial[4]  = 0x00;    //00
		        Commander_Serial[5]  = a_w_0;   		// WaiZhan  L:0~90~180:H
		        Commander_Serial[6]  = a_s_sign_0;  	// ShenZhan +:0 / -:1
		        Commander_Serial[7]  = fabs(a_s_0);	// ShenZhan F:180~90~0:B~-90:T
		        Commander_Serial[8]  = a_w_1;
		        Commander_Serial[9]  = a_s_sign_1;
		        Commander_Serial[10] = fabs(a_s_1);   //90
		        Commander_Serial[11] = 0x00;   //60
		        Commander_Serial[12] = Wii_Status;   //110
		        Commander_Serial[13] = 0x00;   //110
		        Commander_Serial[14] = 0x00;    //90
		        Commander_Serial[15] = 0x00;  //90
		        Commander_Serial[16] = 0x00;  //120
		        Commander_Serial[17] = 0x00;   //104
		        Commander_Serial[18] = 0x00;   //70
		        Commander_Serial[19] = Angle_LT_Unity;   //90
		        Commander_Serial[20] = Angle_RT_Unity;   //30
		        Commander_Serial[21] = 0x00;   //0
		        Commander_Serial[22] = 0x0A;   //10
		        Commander_Serial[23] = Commander_Check;   //Sum(Commander_Serial[2]+...+[22])%255
		        Commander_Serial[24] = 0xED;   //237

		        Serial.write(Commander_Serial, 25); // Send data to Unity via Serial [USB]	
			     
			    // Commander_Serial[0]  = 0xFB;    //251
		     //    Commander_Serial[1]  = 0xBF;    //191
		     //    Commander_Serial[2]  = Angle_L_temp;    //24
		     //    Commander_Serial[3]  = Angle_R_temp;    //35
		     //    Commander_Serial[4]  = 0x00;    //00
		     //    Commander_Serial[5]  = a_w_0;   		// WaiZhan  L:0~90~180:H
		     //    Commander_Serial[6]  = a_s_sign_0;  	// ShenZhan +:0 / -:1
		     //    Commander_Serial[7]  = fabs(a_s_0);	// ShenZhan F:180~90~0:B~-90:T
		     //    Commander_Serial[8]  = a_w_1;
		     //    Commander_Serial[9]  = a_s_sign_1;
		     //    Commander_Serial[10] = fabs(a_s_1);   //90
		     //    Commander_Serial[11] = 0x00;   //60
		     //    Commander_Serial[12] = Wii_Status;   //110
		     //    Commander_Serial[13] = 0x00;   //110
		     //    Commander_Serial[14] = a_x_2;    //90
		     //    Commander_Serial[15] = a_y_2;  //90
		     //    Commander_Serial[16] = a_z_2;  //120
		     //    Commander_Serial[17] = a_yz_2;   //104
		     //    Commander_Serial[18] = 0x00;   //70
		     //    Commander_Serial[19] = Angle_LT_temp;   //90
		     //    Commander_Serial[20] = Angle_RT_temp;   //30
		     //    Commander_Serial[21] = 0x00;   //0
		     //    Commander_Serial[22] = 0x00;   //10
		     //    Commander_Serial[23] = 0x00;   //Sum(Commander_Serial[2]+...+[22])%255
		     //    Commander_Serial[24] = 0xED;   //237

		     //    Serial.write(Commander_Serial, 25); // Send data 
			}

			if(XiaoWei_Output == true){
				//////////////////////////////////////////////// For Lobot Servo Board V2.0
				// Commander_Serial3[ 0] = 0xFB;    			//251	H-1		Header-1
				// Commander_Serial3[ 1] = 0xBF;    			//191	H-2		Header-2
				// Commander_Serial3[ 2] = a_t_2;   			// 90	[0]		Turn of back
				// Commander_Serial3[ 3] = Angle_R_XiaoWei;	// 80	[1]		Right-Elbow
				// Commander_Serial3[ 4] = Angle_RT_XiaoWei;	// 90	[2]		Right-Leg turn
				// Commander_Serial3[ 5] = a_w_1;				//  0	[3]		Right-Up & down
				// Commander_Serial3[ 6] = fabs(a_s_1);		// 90	[4]		Right-Back & forth
				// Commander_Serial3[ 7] = Angle_L_XiaoWei; 	// 60	[5]		Left-Elbow
				// Commander_Serial3[ 8] = Angle_LT_XiaoWei;	// 70	[6]		Left-Leg turn
				// Commander_Serial3[ 9] = 180 - a_w_0;		//180	[7]		Left-Up & down
				// Commander_Serial3[10] = 180 - fabs(a_s_0);	// 90	[8]		Left-Back & forth
				// Commander_Serial3[11] = 0x02;				//200 			Time for Servos
				// Commander_Serial3[12] = Wii_Status;   		//237	Wii 	Commander
				// Commander_Serial3[13] = 0xED;   			//237   End 	Ender	

		        // Serial3.write(Commander_Serial3, 14); // Send data to Unity via Serial [USB]
		        // Serial.write(Commander_Serial3, 14); // Send data to Unity via Serial [USB]	

				// Commander_Serial3[ 0] = 0xFB;   //251	H-1		Header-1
				// Commander_Serial3[ 1] = 0xBF;   //191	H-2		Header-2
				// Commander_Serial3[ 2] = 0x69;   // 105°  0x5F	[0]		Turn of back
				// Commander_Serial3[ 3] = 0x24;	// 36°  0x24	[1]		Right-Elbow
				// Commander_Serial3[ 4] = 0xA2;	// 162° 0xA2	[2]		Right-Leg turn
				// Commander_Serial3[ 5] = 0x51;	//  81° 0x51	[3]		Right-Up & down
				// Commander_Serial3[ 6] = 0x75;	// 117° 0x75	[4]		Right-Back & forth
				// Commander_Serial3[ 7] = 0x17; 	// 23° 	0x17	[5]		Left-Elbow
				// Commander_Serial3[ 8] = 0x12;	// 18° 	0x12	[6]		Left-Leg turn
				// Commander_Serial3[ 9] = 0x6C;	// 108° 0x6C	[7]		Left-Up & down
				// Commander_Serial3[10] = 0x4D;	// 77° 	0x4D	[8]		Left-Back & forth
				// Commander_Serial3[11] = 0x02;	//200 			Time for Servos
				// Commander_Serial3[12] = Wii_Status;   	//237	Wii 	Commander
				// Commander_Serial3[13] = 0xED;   		//237   End 	Ender	

		  //       Serial3.write(Commander_Serial3, 14); // Send data to Unity via Serial [USB]
		  //       Serial.write(Commander_Serial3, 14); // Send data to Unity via Serial [USB]	

				Angle_L_temp = map(ResisitorValue_L, 0, 1023, 0, 500);
				Angle_R_temp = map(ResisitorValue_R, 0, 1023, 0, 267);
				Angle_LT_temp = map(ResisitorValue_LT, 0, 1023, 0, 250);
				Angle_RT_temp = map(ResisitorValue_RT, 0, 1023, 0, 643);

				Angle_L_XiaoWei = Angle_L_temp;
				Angle_R_XiaoWei = Angle_R_temp;
				Angle_LT_XiaoWei = Angle_LT_temp;
				Angle_RT_XiaoWei = Angle_RT_temp;

				Motion_var_1 = Angle_R_XiaoWei;
				Motion_var_1 = constrain(Motion_var_1, 37, (37+80));	// Servo_1 Calibrated
				Motion_var_1 = Motion_var_1-17;

				Motion_var_2 = Angle_RT_XiaoWei;	
				Motion_var_2 = constrain(Motion_var_2, 185, 185+50);
				Motion_var_2 = Motion_var_2-185;	
				Motion_var_2 = map(Motion_var_2,0, 50, 0, 180);	

				// Motion_var_2 = constrain(Motion_var_2, 50, 50+180);		// Servo_2 Calibrated
				// Motion_var_2 = Motion_var_2-(50+20);					// 20 is offset
				// Motion_var_2 = constrain(Motion_var_2, 0, 180);

				Motion_var_3 = 90 - a_w_1;								// Servo_3 Calibrated
				Motion_var_3 = map(Motion_var_3,0, 90, 35, 90);
				// Motion_var_3 = constrain(Motion_var_3, 35, 90);
				Motion_var_4 = 180 - a_s_1;								// Servo_4 Calibrated
				Motion_var_4 = constrain(Motion_var_4, 30, 180);

				Motion_var_5 = Angle_L_XiaoWei;							// Servo_5 Calibrated
				// Motion_var_5 = constrain(Motion_var_5, 0, 255);
				Motion_var_5 = constrain(Motion_var_5, 115, (115+80));	
				Motion_var_5 = 100-(Motion_var_5-105);				

				Motion_var_6 = Angle_LT_XiaoWei;						// Servo_6 Calibrated
				// Motion_var_6 = constrain(Motion_var_6, 0, 255);
				Motion_var_6 = constrain(Motion_var_6, 35, (35+60));
				Motion_var_6 = 2*(Motion_var_6-35);
				Motion_var_6 = constrain(Motion_var_6, 0, 115);

				Motion_var_7 = a_w_0;
				Motion_var_7 = map(Motion_var_7,0, 90, 100, 160);		// Servo_7 Calibrated
				Motion_var_8 = a_s_0;
				Motion_var_8 = constrain(Motion_var_8, 0, 150);			// Servo_8 Calibrated

				Commander_Serial3[ 0] = 0xFB;   //251	H-1		Header-1
				Commander_Serial3[ 1] = 0xBF;   //191	H-2		Header-2
				Commander_Serial3[ 2] = 0x69;   // 105° = 95° 0x5F	[0]		Turn of back
				
				Commander_Serial3[ 3] = 0x14;	// 20° 0x14		[1]		Right-Elbow
				// Commander_Serial3[ 4] = 0x5A;	// 180° 0xB4	[2]		Right-Leg turn
				Commander_Serial3[ 5] = 0x23;	// 90° 0x5A		[3]		Right-Up & Down 	35° 0x23
				Commander_Serial3[ 6] = 0x5A;	// 90° 0x5A		[4]		Right-Back & forth
				// Commander_Serial3[ 3] = Motion_var_1;	// 36°  0x24	[1]		Right-Elbow
				Commander_Serial3[ 4] = Motion_var_2;	// 162° 0xA2	[2]		Right-Leg turn
				// Commander_Serial3[ 5] = Motion_var_3;	//  81° 0x51	[3]		Right-Up & down
				// Commander_Serial3[ 6] = Motion_var_4;	// 130° = 117° 0x75	[4]		Right-Back & forth

				Commander_Serial3[ 7] = 0x0A; 	// 10° 	0x0A	[5]		Left-Elbow	
				Commander_Serial3[ 8] = 0x5A;	// 90° 0x5A		[6]		Left-Leg turn
				Commander_Serial3[ 9] = 0xA0;	// 100° 0x64 	[7]		Left-Up & down 	160° 0xA0
				Commander_Serial3[10] = 0x5A;	// 90° 0x5A		[8]		Left-Back & forth
				
				// Commander_Serial3[ 7] = Motion_var_5; 	// 10° 	0x0A	[5]		Left-Elbow	
				// Commander_Serial3[ 8] = Motion_var_6;	// 90° 0x5A		[6]		Left-Leg turn
				// Commander_Serial3[ 9] = Motion_var_7;	// 108° 0x6C	[7]		Left-Up & down
				// Commander_Serial3[10] = Motion_var_8;	// 77° 	0x4D	[8]		Left-Back & forth
				Commander_Serial3[11] = 0x02;	// 200 			Time for Servos
				Commander_Serial3[12] = Wii_Status;   	//237	Wii 	Commander
				Commander_Serial3[13] = 0xED;   		//237   End 	Ender	

		        Serial3.write(Commander_Serial3, 14); // Send data to Unity via Serial [USB]
		        Serial.write(Commander_Serial3, 14); // Send data to Unity via Serial [USB]	

			}
		}
    }
    wdt_reset();
    // colorWipe(strip.Color(255, 0, 0), 50); // Red
}

//////////////////////////////////////////
uint8_t check_Alpha(uint8_t data[], int arry_head, int arry_end) {
	uint8_t Commander_Sum = 0;
	for(int i=arry_head; i<=arry_end; i++){
		Commander_Sum = Commander_Sum + data[i];
	}
	if(Commander_Check >= 256){
	  	return Commander_Check = (Commander_Sum % 256) - 1;  
	}
	else{
		return Commander_Check = Commander_Sum; 
	}
}


void command_Aplah(int a, int b) {
	Commander_Serial1[0] = 0xFB;    	//251	 H-1	Header-1
	Commander_Serial1[1] = 0xBF;    	//191	 H-2	Header-2
	Commander_Serial1[2] = 0x07;    	// 10	 Len	Sum of Command byte quantity without Ender
	Commander_Serial1[3] = 0x03;    	// 34  	 Com    [Single Servo Control]
	Commander_Serial1[4] = a;		//  3	[ 3]	Servo: Right-Elbow
	Commander_Serial1[5] = b;		//120  Angle	Servo Angle
	Commander_Serial1[6] = check_Alpha(Commander_Serial1, 2, 5);	//  ~  Check    (Len+Com+[1]+..+[n])%256 - 1
	Commander_Serial1[7] = 0xED;   	//237	 End    Ender	

	Serial1.write(Commander_Serial1, 8); // Send data to Alpha via Serial1 [BT] 
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
  }
}
