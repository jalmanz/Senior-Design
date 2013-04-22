/* Line following program for the Automatic Car. 
Copyright (C) 2013  Joe Almanza, Rob Perez, David Sherline, Kang Liu, Shuo Zhang.
Sensor readings via Bruce Allen, 2009.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>. */

#include "DualVNH5019MotorShield.h"
#include <matrix_lcd_commands.h>
#include <LiquidCrystal.h>
#include "QTRSensors.h"
#include <TKLCD.h>
#include <Wire.h>

#define NUM_SENSORS 7 // Although we have 8 sensors, we used an odd number to make finding the middle easier.
#define M_DEFAULT 200 // Default motor speed.
#define TIMEOUT 3000  // Time before sensor turns off.
#define M_MAX 150     // Max motor speed.
#define KP 0.2        // Proportial gain.
#define KD 15  	  // Differential gain.


// Variables for pins on the arduino board.
const int sonar          = 1;  // Analog pin 1 for reading in the analog voltage from the MaxSonar device.
const int stopSwitchPin  = 53; // Digital pin 53 used the stop switch pin.
const int collisionPin   = 49; // Digital pin 49 used for the collision detection.
const int overridePin    = 51; // Digital pin 48 used for manual override.

// Variables used for the sonar and collision sensor function.
long anVolt, inches, cm, i;
int collisionDetectState = 0;
const long int limit     = 10;
int overrideState        = 0;
int avgrange             = 60;
int sum                  = 0;

// Variable for reading the stop and collision system switch status.
int collisionSwitchState = 0;
int stopSwitchState      = 0;  

// Variables used in the PID equation.
int lastError = 0;
int position  = 0;
int error     = 0;

// Variables for motor speeds.
int motorSpeed = 0;
int m1Speed    = 0;
int m2Speed    = 0;

// Initializes the LCD module in serial mode
TKLCD_Serial lcd = TKLCD_Serial();

// Initializes the motor module.
DualVNH5019MotorShield md;  

// Initializes the sensor array, using pin numbers for the emitters.
QTRSensorsRC qtrrc((unsigned char[]) {26, 27, 28, 29, 30, 31, 32, 33}, NUM_SENSORS, TIMEOUT, QTR_NO_EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

// Setup ran before the main loop.
void setup() {
	md.init();                     // Initializes the motor shield.
  	pinMode(sonar, INPUT);         // Initializes the analog pin for the sonar sensor.
	pinMode(stopSwitchPin, INPUT); // Initializes the digital pin for the stop switch.
	pinMode(collisionPin, INPUT);  // Initializes the digital pin for the collision detector.
	pinMode(overridePin, INPUT);   // Initializes the digital pin for the manual override.
    lcd.begin();                   // Initializes the LCD Module.
    lcd.setContrast(255);
	lcd.setBrightness(255);

	// Checks if the stop switch is toggled.
	checkStopSwitch();
	
	// Runs the calibration.
	calibrate();
}

// Main program that is looped.
void loop() {  
	
	// Checks if the stop switch is toggled.
	checkStopSwitch();

    // Checks if there are any obstacles in the way of the car.
	checkObstacle();

	// Checks if the sonar sensor has missed an obstacle and a collision has been detected.
	checkCollision();

  	unsigned int sensors[7];
  	// Get calibrated sensor values returned in the sensors array, along with the line position.
  	// Position will range from 0 to 7000, with 4000 corresponding to the line over the middle sensor.
  	int position = qtrrc.readLine(sensors);
  	unsigned char i;
 
  	error      = position - 3000;
  	motorSpeed = KP * error + KD * (error - lastError);  // PID equation.
    lastError  = error;
 
	// Sets the motor speeds based on the PID equation.
  	m1Speed = M_DEFAULT + motorSpeed;
  	m2Speed = M_DEFAULT - motorSpeed;
 	
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

// Runs the calibration mode.
void calibrate() {
	// Indicates on the LCD screen that the car is in calibration mode.
    lcd.clear();
    lcd.print("Calibrating...");
	delay(1000);
	
	// The car spins in a circle to automatically calibrate.
	md.setSpeeds(200, -200);
	// Calibration lasts about 10 seconds.
    for (i = 0; i < 50; i++) {
		qtrrc.calibrate(); // Reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call).
    	delay(5);
  	}
	md.setBrakes(400, 400);

  	lcd.clear();
    lcd.print("Ready!"); // Shows ready when calibration is done.
	delay(2000);         // Waits 2 seconds before starting.
	lcd.clear();
	lcd.print("Running...");
	return;
}

// Checks if the stop switch has been activated.
void checkStopSwitch() {
	// Read the state of the stop switch value.
	stopSwitchState = digitalRead(stopSwitchPin);
	// Checks if the switch is toggled.
	// If it is, the state is LOW.
	if(stopSwitchState != HIGH) {
		lcd.clear();
		lcd.print("Stopped...");
		while(stopSwitchState != HIGH) {
			md.setBrakes(400,400);
			stopSwitchState = digitalRead(stopSwitchPin);
		}
		lcd.clear();
		lcd.print("Running...");
		return;
	}
}
 
// Checks for an obstacle in the way of the car using the sonar sensor.
void checkObstacle() {          
	// This loop gets 60 reads and averages them
	for(int i = 0; i < avgrange ; i++) {
		// Used to read in the analog voltage output that is being sent by the MaxSonar device.
		// Scale factor is (Vcc/512) per inch. A 5V supply yields ~9.8mV/in
		// Arduino analog pin goes from 0 to 1024, so the value has to be divided by 2 to get the actual inches
		anVolt = analogRead(sonar)/2;
		sum   += anVolt;
	}  
	inches = sum/avgrange;
    	
	// If an object gets within 10 inches, the car stops.
	if(inches < limit) {
		lcd.clear();
		lcd.print("Obstacle!");
		while(inches < limit) {		
			md.setBrakes(400,400);   // Stops the car.		
			// Same averaging for sonar sensor as before.
			for(int i = 0; i < avgrange ; i++) {
				anVolt = analogRead(sonar)/2;
				sum   += anVolt;
			}  
			inches = sum/avgrange;
			sum    = 0;
		}
		lcd.clear();
		lcd.print("Running...");
	}
    return;
}

// Checks if the collision sensors have been activated.
void checkCollision() {
	// Read the state of the collision detectors.
	collisionDetectState = digitalRead(collisionPin);
	// Checks if the sensors have been activated.
	// If they are, the state is LOW.
	if(collisionDetectState != HIGH) {
		md.setBrakes(400,400);
		lcd.clear();
        lcd.print("Collision!");
		md.setSpeeds(-200, -200);
		delay(200);
		md.setBrakes(400, 400);
		
		// This section is used for the hypothetical driver to wait for the obstacle to be moved,
		// then provide input for the car to being moving again.
		overrideState = digitalRead(overridePin);
		if(overrideState != HIGH) {
			lcd.clear();
			lcd.print("Waiting...");
			while(overrideState != HIGH) 
				overrideState = digitalRead(overridePin);			
		}
		lcd.clear();
		lcd.print("Running...");
	}
	return;
}

// Stops the motors if every other sensor has failed and there is a fault.
void stopIfFault() {
	if (md.getM1Fault()) {
		lcd.clear();
		lcd.print("Motor 1 Fault");
		md.setBrakes(400,400);
    	while(1);
  	}

  	if (md.getM2Fault()) {
    	lcd.clear();
		lcd.print("Motor 2 Fault");
		md.setBrakes(400,400);
    	while(1);
  	}
}
