////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//File: Chronos_Flightboard.ino
//Date: Jan 02, 2014
// NASA Student Launch
////Arduino sketch for the Chronos Arduino flight computer to process flight parameters and deploy airbrake system.
//University: California State Polytechnic University, Pomona
//Author: Ryan Gunawan
//Contact: gunawan.ryan@gmail.com
#define chronos_version "1.5"
//1.1: XBee Testing and Debug: WORKING
//1.2: Accelerometer Debug Status: WORKING
//1.3: Wireless SD Shield: WORKING
//1.4: Velocity Calculations: NEEDS VERIFICATION
//1.5: Motor Shield with Linear Actuator: DEBUGGING IN PROGRESS
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Libraries Included //////////////////////////////////////////////////////////////////////////////////////////////
//#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <stdlib.h>
#include <math.h>
#include <SD.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include "MPL3115A2.h"

// Pin Definitions /////////////////////////////////////////////////////////////////////////////////////////////////
//SoftwareSerial XBee(2, 3); //Creates XBee object //RENDERED USELESS POST V1.3
const int CS = 10; // Chip Select Signal -> Pin 10
const int sdcs = 4;

//ADXL345 Registers
char POWER_CTL = 0x2D;	//Power Control Register
char DATA_FORMAT = 0x31;
char DATAX0 = 0x32;	//X-Axis Data 0
char DATAX1 = 0x33;	//X-Axis Data 1
char DATAY0 = 0x34;	//Y-Axis Data 0
char DATAY1 = 0x35;	//Y-Axis Data 1
char DATAZ0 = 0x36;	//Z-Axis Data 0
char DATAZ1 = 0x37;	//Z-Axis Data 1

//This buffer will hold values read from the ADXL345 registers.
char values[10];
//These variables will be used to hold the x,y and z axis accelerometer values.

int sx, sy, sz;
float vix = 0, viy = 0, viz = 0, vfx, vfy, vfz;

// Dummy Parameters/////////////////////////////////////////////////////////////////////
int packet = 0;  // Packet Number
int x = 1;  // Simulated X-Accel
int y = 2;  // Simulated Y-Accel
int z = 3;  // Simulated Z-Accel

const int xpin = A0;                  // x-axis of the accelerometer
const int ypin = A1;                  // y-axis
const int zpin = A2;                  // z-axis (only on 3-axis models)

// Make sure these two variables are correct for your setup
int scale = 16; // 3 (±3g) for ADXL337, 200 (±200g) for ADXL377
boolean micro_is_5V = true; // Set to true if using a 5V microcontroller such as the Arduino Uno, false if using a 3.3V microcontroller, this affects the interpretation of the sensor data

double iTime, dTime, aTime, pTime;
float pitch, roll;

// Altimeter Definitions
MPL3115A2 myPressure;
float altitude, temperature;

// Transmission Parameters
File dataFile;
String dataString = "Initializing...";
char charVal[10];
static char dtostrfbuffer[15];

// Linear Actuator Motor Shield Definition
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor = AFMS.getMotor(4);
boolean activate = false, retract = false, burn = false, burnout = false;

void setup()
{
	//Wireless Shield - XBee Transciever
	Serial.begin(57600);
	//Wireless SD Shield - MicroSD

	if (!SD.begin(sdcs)) {
		Serial.println("Card failed, or not present");
		// don't do anything more:
		return;
	}
	Serial.println("card initialized.");
	//MotorShield
	AFMS.begin();
	myMotor->setSpeed(0);

	//Initiate an SPI communication instance. & Configure the SPI connection for the ADXL345
	SPI.begin();
	SPI.setDataMode(SPI_MODE3);
	//Set up the Chip Select pin to be an output from the Arduino.
	pinMode(CS, OUTPUT);
	//Before communication starts, the Chip Select pin needs to be set high.
	digitalWrite(CS, HIGH);
	//Put the ADXL345 into +/- 16G range by writing the value 0x0B to the DATA_FORMAT register.
	writeRegister(DATA_FORMAT, 0x0B);
	//Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
	writeRegister(POWER_CTL, 0x08);  //Measurement mode   
	// Calibration Code Here:
	// https://learn.adafruit.com/adxl345-digital-accelerometer/programming

	// Previous Acceleration values for calculations
	sx = ((int)values[1] << 8) | (int)values[0];
	//The Y value is stored in values[2] and values[3].
	sy = ((int)values[3] << 8) | (int)values[2];
	//The Z value is stored in values[4] and values[5].
	sz = ((int)values[5] << 8) | (int)values[4];

	myPressure.setModeAltimeter();  // Measures altitude above sea level (in meters)
	myPressure.setOversampleRate(7); // Set Oversample to the recommended 128
	myPressure.enableEventFlags(); // Enable all three pressure and temp event flags   

	iTime = millis();
	pTime = iTime;
}

// Notations: Z Axis is vertical axis
void loop()
{
	File dataFile = SD.open("chronos.txt", FILE_WRITE);
	dataString = "";
	//DATA TRANSMISSION
	packet = packet + 1;
	dataString += packet;
	// Altitude Measurements
	altitude = myPressure.readAltitudeFt();
	temperature = myPressure.readTempF();
	// Accelerometer Reading 
	//Reading 6 bytes of data starting at register DATAX0 will retrieve the x,y and z acceleration values from the ADXL345.
	//The results of the read operation will get stored to the values[] buffer.
	readRegister(DATAX0, 6, values);
	aTime = millis();
	dTime = (aTime - pTime) / 1000;

	//The ADXL345 gives 10-bit acceleration values, but they are stored as bytes (8-bits). To get the full value, two bytes must be combined for each axis.
	//The X value is stored in values[0] and values[1].
	x = ((int)values[1] << 8) | (int)values[0];
	//The Y value is stored in values[2] and values[3].
	y = ((int)values[3] << 8) | (int)values[2];
	//The Z value is stored in values[4] and values[5].
	z = ((int)values[5] << 8) | (int)values[4];

	float xg = mapf(x, -4095, 4095, -16, 16);  // Might have to include offsets to account for gravity
	float yg = mapf(y, -4095, 4095, -16, 16);  // Have to double check to verify direction of offsets
	float zg = mapf(z, -4095, 4095, -16, 16);  // Subtract 1g from each axis?


	// Velocity Calculations
	vfx = vix + (xg*dTime*32.2);
	vfy = viy + (yg*dTime*32.2);
	vfz = viz + (zg*dTime*32.2);

	// Pitch and Roll Calculations
	pitch = atan2(xg, sqrt(zg*zg + yg*yg)) * 180.0 / PI;
	roll = atan2(yg, sqrt(zg*zg + xg*xg)) * 180.0 / PI;
	// Burnout Calculations
	// Calculate whether burnout has occured or not
	//do a calculation whether the acceleration has gone from positive to negative
	//confirmation by multiple values
	if (xg > 5 || yg > 5 || zg > 5){
		burn = true;
	}
	//

	// Linear Actuator Controls
	// Calculations
	double totalVelocity = sqrt(vfx*vfx + vfy*vfy + vfz*vfz);
	if (totalVelocity < 150 && burnout){
		activate = true;
	}
	if (burnout && totalVelocity == 0)
		// Decision making to activate
		if (activate){
		activateAB(100);
		dataFile.println("AIRBRAKES ACTIVATION INITIATED");
		}
	if (retract){
		deactivateAB(100);
		dataFile.println("AIRBRAKES RETRACTION INITIATED");
	}
	activateAB(100);
	deactivateAB(100);


	//   String dataStr = 42,time,xg,yg,zg,checksum
	// appending -> dataStr += 
	dataString += 42;
	dataString += "," + packet;
	dataString += ",";
	dataString += dtostrf(xg, 5, 1, dtostrfbuffer);
	dataString += ",xg,";
	dataString += dtostrf(yg, 5, 1, dtostrfbuffer);
	dataString += ",yg,";
	dataString += dtostrf(zg, 5, 1, dtostrfbuffer);
	dataString += ",zg,";
	dataString += dtostrf(altitude, 5, 1, dtostrfbuffer);
	dataString += ",alt,%GUN";


	Serial.print("42:");  //Data Packet Identifier
	Serial.print(packet);
	Serial.print(",");
	Serial.print(xg);
	Serial.print("gx,");
	Serial.print(yg);
	Serial.print("gy,");
	Serial.print(zg);
	Serial.print("gz,");
	Serial.print("%");
	Serial.print("\t");
	Serial.print(vfx);
	Serial.print("vx\t");
	Serial.print(vfy);
	Serial.print("vy\t");
	Serial.print(vfz);
	Serial.println("vz\t");
	delay(50);

	// If the file is available, write to it.
	if (dataFile) {
		dataFile.println(dataString);
		dataFile.close();
		// print to the serial port too:.
		Serial.println(dataString);
	}
	// if the file isn't open, pop up an error:
	else {
		Serial.println("error opening chronosdatalog.txt");
	}


	// Update variables for consecutive calculations
	pTime = aTime;
	sx = x;
	sy = y;
	sz = z;
	vix = vfx;
	viy = vfy;
	viz = vfz;
}

// Same functionality as Arduino's standard map function, except using floats
float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


//This function will write a value to a register on the ADXL345.
//Parameters:
//  char registerAddress - The register to write a value to
//  char value - The value to be written to the specified register.
void writeRegister(char registerAddress, char value){
	//Set Chip Select pin low to signal the beginning of an SPI packet.
	digitalWrite(CS, LOW);
	//Transfer the register address over SPI.
	SPI.transfer(registerAddress);
	//Transfer the desired register value over SPI.
	SPI.transfer(value);
	//Set the Chip Select pin high to signal the end of an SPI packet.
	digitalWrite(CS, HIGH);
}

//This function will read a certain number of registers starting from a specified address and store their values in a buffer.
//Parameters:
//  char registerAddress - The register addresse to start the read sequence from.
//  int numBytes - The number of registers that should be read.
//  char * values - A pointer to a buffer where the results of the operation should be stored.
void readRegister(char registerAddress, int numBytes, char * values){
	//Since we're performing a read operation, the most significant bit of the register address should be set.
	char address = 0x80 | registerAddress;
	//If we're doing a multi-byte read, bit 6 needs to be set as well.
	if (numBytes > 1)address = address | 0x40;

	//Set the Chip select pin low to start an SPI packet.
	digitalWrite(CS, LOW);
	//Transfer the starting register address that needs to be read.
	SPI.transfer(address);
	//Continue to read registers until we've read the number specified, storing the results to the input buffer.
	for (int i = 0; i<numBytes; i++){
		values[i] = SPI.transfer(0x00);
	}
	//Set the Chips Select pin high to end the SPI packet.
	digitalWrite(CS, HIGH);
}

void activateAB(int x){
	myMotor->setSpeed(255);
	myMotor->run(FORWARD);
	delay(x);
	myMotor->setSpeed(0);
}

void deactivateAB(int x){
	myMotor->setSpeed(255);
	myMotor->run(BACKWARD);
	delay(x);
	myMotor->setSpeed(0);
}

void errorMsg(File datafile, String error){
	datafile.println("Error: " + error);
}