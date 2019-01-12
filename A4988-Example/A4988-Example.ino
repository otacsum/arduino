/* FILE:    A4988_Example.cpp
   DATE:    01/03/17
   VERSION: 0.1
   AUTHOR:  Andrew Davies
   
01/03/17 version 0.1: Original version
   
A simple example to generate the required control signals for the A4988 stepper 
motor controller module. 

The module should be connected to your Arduino as follows:

Arduino..........MAX6675 (HCSENS0038)
GND..............GND
+5V..............+5V
8................Enable (EN)
9................Direction (DIR)
10...............STEP (CLK)


You may copy, alter and reuse this code in any way you like, but please leave
reference to HobbyComponents.com in your comments if you redistribute this code.
This software may not be used directly for the purpose of selling products that
directly compete with Hobby Components Ltd's own range of products.

THIS SOFTWARE IS PROVIDED "AS IS". HOBBY COMPONENTS MAKES NO WARRANTIES, WHETHER
EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, ACCURACY OR LACK OF NEGLIGENCE.
HOBBY COMPONENTS SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR ANY DAMAGES,
INCLUDING, BUT NOT LIMITED TO, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY
REASON WHATSOEVER.
*/


/* Pins used for control signals */
#define ENABLE 8
#define DIRECTION 9
#define STEP 10

#define FORWARD HIGH
#define REVERSE LOW

/* Change this values to alter the clock speed */
#define SPEED 1

void setup() 
{
  pinMode(ENABLE, OUTPUT);
  pinMode(DIRECTION, OUTPUT);
  pinMode(STEP, OUTPUT);

  /* Pull the enable pin low to enable the driver */
  digitalWrite(ENABLE, LOW);
}


void loop() 
{
  /* The the rotational direction to the forward direction */
  digitalWrite(DIRECTION, FORWARD);

  /* Keep stepping the motor in an infinite loop */
  while(1)
  {
    digitalWrite(STEP, HIGH);   
    delay(SPEED);              
    digitalWrite(STEP, LOW);    
    delay(SPEED);            
  }
}