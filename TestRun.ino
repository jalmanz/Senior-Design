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

// Analog pin 1 for reading in the analog voltage from the MaxSonar device.
const int anPin = 1;
// The distance in which the vehicle will stop moving forward.
const long int limit = 20;

// Variables needed to store values
long anVolt, inches, cm;
int sum      = 0;
int avgrange = 60; // Quantity of values to average (sample size)

const int buttonPin = 53;     // The number of the pushbutton pin
int buttonState = 0;          // Variable for reading the pushbutton status

DualVNH5019MotorShield md;

// Stops the motors if there is a fault.
void stopIfFault()
{
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
}

void loop() {
	
	Serial.println("button check");
	// Read the state of the pushbutton value:
	buttonState = digitalRead(buttonPin);
	// Check if the pushbutton is pressed.
	// If it is, the buttonState is HIGH:
	if (buttonState == HIGH) {     
		// Start test:    
		Serial.println("button on");
	}else{
		// Stop program:
		Serial.println("button off");
		md.setSpeeds(0, 0);
		return;
	}
  
	// MaxSonar Analog reads are known to be very sensitive. See the Arduino forum for more information.
	// A simple fix is to average out a sample of n readings to get a more consistant reading.
	// Even with averaging I still find it to be less accurate than the pw method.
	// This loop gets 60 reads and averages them
	for(int i = 0; i < avgrange ; i++) {
		//Used to read in the analog voltage output that is being sent by the MaxSonar device.
		//Scale factor is (Vcc/512) per inch. A 5V supply yields ~9.8mV/in
		//Arduino analog pin goes from 0 to 1024, so the value has to be divided by 2 to get the actual inches
		anVolt = analogRead(anPin)/2;
		sum += anVolt;
	}  
	inches = sum/avgrange;
	Serial.println(inches);
	
	// If an object gets within 20 inches, the car will stop and rotate left till nothing is in the way.
	if(inches < limit) {
		md.setBrakes(400,400);   // Stops.
		md.setSpeeds(100, 100);  // Rotates.
		stopIfFault();
	}else{
		md.setSpeeds(100, -100); // Continue forward.
		stopIfFault();
	}
	sum = 0;
 }

  
  
  
