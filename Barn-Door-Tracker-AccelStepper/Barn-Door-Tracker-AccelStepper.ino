#include <CheapStepper.h>	//Include the 28BYJ-48 driver library
#include <OneButton.h>	//Include the button monitoring library

//Turn on serial monitoring for debugging.
#define debug 	false

/**Declare our stepper and set the digital output pins:
 *	Arduino pin <--> pins on ULN2003 board:
 *			3 	<--> IN1
 *			4 	<--> IN2
 *			5 	<--> IN3
 *			6 	<--> IN4
 */
CheapStepper stepper (3,4,5,6); 


/**Speed definition:
 *4075 - 5000 steps per full revolution (approximately).
 *Using the calibration dial, determine your exact steps, https://www.thingiverse.com/thing:2007795
 *There are three different versions of the little stepper of gear ratio so just test and time this.
 *Set the number of steps for your version of the motor by uncommenting the appropriate line, below.
 */
#define stepsPerRevolution		4096	//internal gear ratio is 64:1 (advertised)
//#define stepsPerRevolution	4075	//internal gear ratio is 63.68395:1 (measured on some units)


/**Delay Calculation:
 *We want 1.09 turns of the big gear in 60 seconds.
 *Gear ratio is 4.3:1
 *Delay in microseconds between steps with CheapStepper is 8.8838
 *So...
 */
float stepsDelayMicroscondsDecimal = ((stepsPerRevolution * 4.3 * 1.09) / 8.8838);
//Drop the decimal, any remainder is going to be insignificant for the small variations on these motors.
int   stepsDelayMicrosconds = stepsDelayMicroscondsDecimal;  


/**On/Off/Reverse Switching:
 *A button will change the state of the tracker:
 *Short Click 	--> 	Turn the machine On, Clockwise by default.
 *Double Click 	-->		Clockwise, Counter-Clockwise
 *Long Press 	-->		Turn the machine Off.
 *Let's create a boolean array to save the power and direction of our rotation
 *We want the large gear to turn clockwise to raise the platform by default...
 *So the motor gear should turn counter-clockwise by default.
 *If it goes wrong way on startup, make the second default value "true" instead
 */
bool machineState[] = {false, false};	//machineState[power, direction] --> [Off, Counter-Clockwise]

// Setup a new OneButton on pin A1.  
OneButton button1 (A1, true);

void setup() {
	if (debug) {
		//Print stepper calculation to the monitor for testing and validation.
		Serial.begin(9600);
		Serial.print("Delay of ");
		Serial.print(stepsDelayMicrosconds); // get delay between steps for set RPM
		Serial.print(" microseconds between steps");
		Serial.println();	
	}

	// link the button1 functions.
	button1.attachClick(button1Click);
	button1.attachDoubleClick(button1DoubleClick);
	button1.attachLongPressStart(button1LongPress); //Runs once when a long press is detected.
}

void loop() {
	//Keep watching the push buttons.
	button1.tick();

	if (machineState[0]) { //First parameter is true, drive the motor.
		stepper.step(machineState[1]);
		delayMicroseconds(stepsDelayMicrosconds); 
	}
}


/**
 * Callback functions for the OneButton monitoring.
 */

void button1Click() {
	if (debug) {
		Serial.println("Button 1 Clicked - Turning machine on.");
	}

	machineState[0] = true;  //turn the machine on.
}

void button1DoubleClick() {
	if (debug) {
		Serial.println("Button 1 Double Clicked - Reversing direction.");
	}

	machineState[1] = !machineState[1];  //invert second parameter, clockwise, counter-clockwise boolean.
}

void button1LongPress() {
	if (debug) {
		Serial.println("Button 1 Long Pressed - Turning machine off.");
	}

	machineState[0] = false;  //turn the machine off.
	machineState[1] = false;  //reset default rotation to counterclockwise.
}

