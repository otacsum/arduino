#include <AccelStepper.h>

// Pins used for control signals
#define ENABLE 2
#define dirPin 3
#define stepPin 4

#define REVERSE

#define stepsPerSec 41 // Steps (or microsteps) per second

AccelStepper rAsc(1, stepPin, dirPin); //1 = bipolar motor on dedicated driver

#ifdef REVERSE
	int direction = -1;
#else
	int direction = 1;
#endif	

void setup() {
	// Pull the enable pin low to enable the driver
	pinMode(ENABLE, OUTPUT);
	digitalWrite(ENABLE, LOW);

	// Set the upper limit and then define the constant run speed
	rAsc.setMaxSpeed(direction * stepsPerSec);
	rAsc.setSpeed(direction * stepsPerSec);
}

void loop() {  
	rAsc.runSpeed(); // Run the motor
}