/* Test program for the Automatic Car. 
Copyright (C) 2013  Joe Almanza, Rob Perez, David Sherline, Kang Liu, Shuo Zhang.
Sensor readings via Bruce Allen, 2009.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details. */

#include "DualVNH5019MotorShield.h"
#include "QTRSensors.h"

#define NUM_SENSORS 8
#define TIMEOUT 3000  // Time before sensor turns off.
#define KP 0.2
#define KD 5
#define M_DEFAULT 75  // Default motor speed.
#define M_MAX 400     // Max motor speed.

const int anPin      = 1;  // Analog pin 1 for reading in the analog voltage from the MaxSonar device.
const int buttonPin  = 53; // The number of the pushbutton pin.
const long int limit = 20; // The distance in which the vehicle will stop moving forward.

// Variables needed to store values.
long anVolt, inches, cm, i;
int sum         = 0;
int avgrange    = 60;
int lastError   = 0;
int buttonState = 0; // Variable for reading the pushbutton status.

// Initializes the motor module.
DualVNH5019MotorShield md;  

// Initializes the sensor array
QTRSensorsRC qtrrc((unsigned char[]) {26, 27, 28, 29, 30, 31, 32, 33}, NUM_SENSORS, TIMEOUT, QTR_NO_EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

// Stops the motors if there is a fault.
void stopIfFault() {
	if (md.getM1Fault()) {
		Serial.println("M1 fault");
    		while(1);
  	}

  	if (md.getM2Fault()) {
    		Serial.println("M2 fault");
    		while(1);
  	}
    
	if (md.getM2Fault()) {
    		Serial.println("Stop");
    		while(1);
  	}
}

void setup() {
  	pinMode(anPin, INPUT);
	Serial.begin(9600);  // This opens up a serial connection to shoot the results back to the PC console
	md.init();           // Initializes the motor shield.

	// Turns on LED to indicate calibration mode.
        pinMode(23, OUTPUT);
        digitalWrite(23, HIGH);
        
	// Calibration lasts about 10 seconds.
        for (i = 0; i < 250; i++) {
    		qtrrc.calibrate(); // Reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call).
    		delay(20);
  	}
  	digitalWrite(23, LOW);     // Turn off LED to indicate we are through with calibration.

  	// Print the calibration minimum values measured when emitters were on.
  	Serial.begin(9600);
  	Serial.print("Minimum values: ");
  	for (i = 0; i < NUM_SENSORS; i++) {
    		Serial.print(qtrrc.calibratedMinimumOn[i]);
    		Serial.print(' ');
  	}
  	Serial.println();
  
  	// print the calibration maximum values measured when emitters were on
  	Serial.print("Maximum Values: ");
  	for (i = 0; i < NUM_SENSORS; i++) {
    		Serial.print(qtrrc.calibratedMaximumOn[i]);
    		Serial.print(' ');
 	}
  	Serial.println();
  	Serial.println();
  	delay(1000);
}

void loop() {

	// Serial.println("button check");
	// Read the state of the pushbutton value:
	buttonState = digitalRead(buttonPin);
	// Check if the pushbutton is pressed.
	// If it is, the buttonState is HIGH:
	if (buttonState == HIGH) {     
		// Do nothing.
	}else{
		md.setSpeeds(0, 0);
		return;
	} 

  	unsigned int sensors[7];
  	// Get calibrated sensor values returned in the sensors array, along with the line position.
  	// Position will range from 0 to 7000, with 4000 corresponding to the line over the middle sensor.
  	int position = qtrrc.readLine(sensors);
  	unsigned char i;
  	for (i = 0; i < NUM_SENSORS; i++) {
    		Serial.print(sensors[i] * 10 / 1001);
    		Serial.print(' ');
  	}
  	Serial.print("    ");
  	Serial.println(position);
 
  	int error      = position - 3000;
  	int motorSpeed = KP * error + KD * (error - lastError);  // PID equation.
        lastError      = error;
 
	// Sets the motor speeds based on the PID equation.
  	int m1Speed = M_DEFAULT + motorSpeed;
  	int m2Speed = M_DEFAULT - motorSpeed;
 	
	// Makes sure the motor speeds stay at or above zero.
  	if (m1Speed < 0)
    		m1Speed = 0;
  	if (m2Speed < 0)
   		m2Speed = 0;
    
	// Makes sure the motor speeds stay at or below the maximum.
  	if(m1Speed > M_MAX)
    		m1Speed = M_MAX;
  	if(m2Speed > M_MAX)
   		 m2Speed = M_MAX;
 
  	// Sets the motor speeds using the two motor speed variables above
  	md.setSpeeds(m1Speed, m2Speed);
  	stopIfFault();
 }
