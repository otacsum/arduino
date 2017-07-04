/**
 * ardu-predator.ino
 *
 * @author Mike Muscato
 * @date   2017-07-04
 * 
 * Refactored to handle two GY-521 MPU6050 Breakout Boards.
 * One fixed to the body, the other dynamic on the head.
 * By calculating the difference between tilt and yaw angles...
 * we can keep the shoulder cannon pointing along our relative...
 * line of vision, regardless of our absolute body position.
 *
 * =============  Attribution ==========================
 * Based on MPU6050_Latest_code.ino
 * Originally provided by "HC" aka "zhomeslice" on forum.arduino.cc
 * at https://forum.arduino.cc/index.php?PHPSESSID=h4c6487i42hbb7uh6rjk0eadp1&topic=446713.msg3073854#msg3073854
 * 
 * Original source committed in my repo for reference at: arduino/MPU6050/MPU6050_Latest_code/
 * =====================================================
 * 
 */

// Arduino Library must include the following library file collections.  
// These repos are forked on my gitHub or you can find them at...
// https://github.com/jrowberg/i2cdevlib
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

// Standard included arduino servo library
#include "Servo.h"

// Comment to turn off Serial printing
#define DEBUG  

// DEBUG serial printing macros
#ifdef DEBUG
	//#define DPRINT(args...)  Serial.print(args)             //OR use the following syntax:
	#define DPRINTSTIMER(t)    for (static unsigned long SpamTimer; (unsigned long)(millis() - SpamTimer) >= (t); SpamTimer = millis())
	#define  DPRINTSFN(StrSize,Name,...) {char S[StrSize];Serial.print("\t");Serial.print(Name);Serial.print(" "); Serial.print(dtostrf((float)__VA_ARGS__ ,S));}//StringSize,Name,Variable,Spaces,Percision
	#define DPRINTLN(...)      Serial.println(__VA_ARGS__)
#else
	#define DPRINTSTIMER(t)    if(false)
	#define DPRINTSFN(...)     //blank line
	#define DPRINTLN(...)      //blank line
#endif

// Initialize gyro instances for both MPUs
MPU6050 mpus[2] = {MPU6050(0x68), MPU6050(0x69)};

// Initialize servo instances
Servo yawServo;
Servo pitchServo;

//LED will give us status of the loop, if it turns off something failed.
const int LED_PIN = 13;

// Prevent the servos from rotating beyond mechanical limits 
const int maxPitch = 135;
const int minPitch = 45;
// NOTE: Yaw direction is inverted via map() method in moveServos()
const int maxYaw = 135;
const int minYaw = 45;

// You must supply your gyro offsets here,
// Use MPU6050_calibration.ino found at:
//     https://forum.arduino.cc/index.php?PHPSESSID=h4c6487i42hbb7uh6rjk0eadp1&topic=446713.msg3073854#msg3073854
//     
// Calibration code also copied in my repo:  arduino/MPU6050/
// 
// Gyro 0 calibrated at:  {-3074,  -2036,  1236,    63,    -16,     78}
// Gyro 1 calibrated at:  {-1143,  -1197,   989,   147,    -78,    -10}
// 
//                           XA      YA      ZA     XG      YG      ZG
int mpuOffsets[2][6] =  {
                          {-3074,  -2036,  1236,    63,    -16,     78},
                          {-1143,  -1197,   989,   147,    -78,    -10}
                        };

// var to store the servo position when testing
int pos = 0;    

// vars for angle values.
float Yaw, Pitch, Roll;

// Will be used for timing events.
unsigned long currentMillis = 0;    // stores the value of millis() in each iteration of loop()
unsigned long prevMillis = 0;    // stores the value of millis() in each iteration of loop()



// ================================================================
// ===                      i2c SETUP Items                     ===
// ================================================================
void i2cSetup() {
  // Join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
}



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
// indicates whether MPU interrupt pin has gone high
volatile bool mpuInterrupt[2] = {false, false};     

//Intentional duplication. Interrupt routine cannot pass args. 
void dmpDataReady0() {
    mpuInterrupt[0] = true;
}
void dmpDataReady1() {
    mpuInterrupt[1] = true;
}



// ================================================================
// ===                      MPU DMP SETUP                       ===
// ================================================================
uint8_t devStatus; // return status after each initialization operation (0 = success, !0 = error)

//Containers for each MPU's communication
int FifoAlive[2] = {0, 0}; // tests if the interrupt is triggering
uint16_t packetSize[2];    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount[2];     // count of all bytes currently in FIFO
uint8_t fifoBuffer[2][64]; // FIFO storage buffer

// orientation/motion vars 
// (overwritten during each read/calculate cycle so they do not need to be duplicated for each MPU)
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector



/**
 * MPU6050 Connection / Initialization routine
 * =============================================================
 *
 * @author Modified by Mike Muscato to enable dual MPU configuration 
 * @date   2017-07-04
 *
 */
void MPU6050Connect() {

  for (int i = 0; i < 2; i++) {
    Serial.print(F("Initializing MPU ")); Serial.println(i);
    
    static int MPUInitCntr = 0;  //Counter for looping and eventual initialization failure.
    
    // initialize device
    mpus[i].initialize();
    // load and configure the DMP
    devStatus = mpus[i].dmpInitialize();
    
    if (devStatus != 0) {
      
      // ERROR Checking!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      
      char *StatStr[5] { "No Error", "initial memory load failed", "DMP configuration updates failed", "3", "4"};

      MPUInitCntr++;

      Serial.print(F("MPU")); Serial.print(i); Serial.print(F(" connection Try #")); Serial.println(MPUInitCntr);
      Serial.print(F("DMP Initialization failed (code ")); Serial.print(StatStr[devStatus]); Serial.println(F(")"));
      
      if (MPUInitCntr >= 10) return; //only try 10 times
      delay(1000);
      MPU6050Connect(); // Lets try again
      return;
    }

    //Set offset values for each of our MPUs from our array.
    mpus[i].setXAccelOffset(mpuOffsets[i][0]);
    mpus[i].setYAccelOffset(mpuOffsets[i][1]);
    mpus[i].setZAccelOffset(mpuOffsets[i][2]);
    mpus[i].setXGyroOffset(mpuOffsets[i][3]);
    mpus[i].setYGyroOffset(mpuOffsets[i][4]);
    mpus[i].setZGyroOffset(mpuOffsets[i][5]);

    Serial.print(F("Enabling DMP on MPU")); Serial.println(i);
    mpus[i].setDMPEnabled(true);

    // enable Arduino interrupt detection
    int intPin = i + 2;
    Serial.print(F("Enabling interrupt detection on MPU")); Serial.print(i); Serial.print(F("(Arduino external interrupt pin ")); Serial.print(intPin); Serial.println(F(" on the Uno)..."));
    Serial.print(F("mpu")); Serial.print(i); Serial.print(F(".getInterruptDrive=  ")); Serial.println(mpus[i].getInterruptDrive());  //Current interrupt drive mode (0=push-pull, 1=open-drain)
    
    //Intentional duplication. Interrupt routine cannot pass args. 
    if (i == 0) {
      attachInterrupt(i, dmpDataReady0, RISING);
    }
    else {
      attachInterrupt(i, dmpDataReady1, RISING);
    }
    
    packetSize[i] = mpus[i].dmpGetFIFOPacketSize();  // get expected DMP packet size for later comparison
    delay(1000); // Let it Stabalize
    mpus[i].resetFIFO(); // Clear fifo buffer    
    mpuInterrupt[i] = false; // wait for next interrupt
    
  }

}



/**
 * GetDMP Reads the data from our MPUs and stores it in each mpus instance.
 * =============================================================
 *
 * @author Modified by Mike Muscato to enable dual MPU configuration 
 * @date   2017-07-04
 *
 */
void GetDMP() { 

  for (int i = 0; i < 2; i++) { 
    mpuInterrupt[i] = false;
    FifoAlive[i] = 1;
    fifoCount[i] = mpus[i].getFIFOCount();
    
    if ((!fifoCount[i]) || (fifoCount[i] % packetSize[i])) { // we have failed Reset and wait till next time!
      digitalWrite(LED_PIN, LOW); // lets turn off the blinking LED so we can see we are failing.
        mpus[i].resetFIFO();// clear the buffer and start over
    } 
    else {
      while (fifoCount[i]  >= packetSize[i]) { // Get the packets until we have the latest!
          mpus[i].getFIFOBytes(fifoBuffer[i], packetSize[i]); // lets do the magic and get the data
          fifoCount[i] -= packetSize[i];
      }
    }
  } 
  
  MPUMath(); // Successful!  Do the math and show angles from both MPUs
  digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Blink the LED on each cycle
  
}



/**
 * MPUMath Calculates angles of MPUs from raw sensor data
 * =============================================================
 *
 * @author Modified by Mike Muscato to enable dual MPU configuration 
 * @date   2017-07-04
 *
 */
void MPUMath() {
  for (int i = 0; i < 2; i++) {
    // Get the raw data from the sensor
    mpus[i].dmpGetQuaternion(&q, fifoBuffer[i]);
    mpus[i].dmpGetGravity(&gravity, &q);
    mpus[i].dmpGetYawPitchRoll(ypr, &q, &gravity);
  
    // Calculate readable angles
    // Adds 90-degrees to center servos when sensor is level (e.g. 0 degrees)
    // TODO: Move the +90 gyro/servo leveling magic number to another method, 
    // ...it doesn't belong here and will cause readability problems later.
    Yaw = (ypr[0] * 180.0 / M_PI) + 90;
    Pitch = (ypr[1] *  180.0 / M_PI) + 90;
    Roll = (ypr[2] *  180.0 / M_PI) + 90;

    //TODO: Add YPR values to temp var for each MPU, so that we can diff them.

    // Serial.print the current angle values of the gyro position.
    // DPRINTSTIMER(x) argument is the number of milliseconds between print events
    // Smaller numbers give more reliable switching between MPU 0 an 1 but seem to cause crashing (Buffer overrun?)
    //    DPRINTSTIMER(1) {
    //      DPRINTSFN(15, "\tValues for MPU :", i, 6, 1);
    //      DPRINTSFN(15, "\tYaw:", Yaw, 6, 1);
    //      DPRINTSFN(15, "\tPitch:", Pitch, 6, 1);
    //      DPRINTSFN(15, "\tRoll:", Roll, 6, 1);
    //      DPRINTLN();
    //    }
    
  }

    // Timer - don't try and move the servos too frequently, they need time to get to the current position.
    // TODO: Remove delay magic number and move to global var for tuning mechanical response
    if (currentMillis >= prevMillis + 15) {
      moveServos(round(Yaw), round(Pitch)); // TODO:  Change these values to the new diff'd values.
    }
}

/**
 * Move the servos to the current gyro postion
 * =============================================================
 *
 * @author Mike Muscato
 * @date   2017-01-14
 *
 * @param  {integer}   int servoYaw      The rounded angle of the z-axis rotation of the gyro
 * @param  {integer}   int servoPitch    The rounded angle of the y-axis rotation of the gryo
 * @return {void}       
 */
void moveServos(int servoYaw, int servoPitch) {
  //Serial.println("moveServos");

  // TODO: Determine if yaw/pitch is beyond normal servo angles i.e. > 179 or < 0
  // May not be necessary after adding 2nd gyro and using relative angles.
  // If so, assume actual gyro position and provide max servo value.
  
  // Invert Yaw values for my servo orientation.
  int invertedYaw = map(servoYaw, 0, 179, 179, 0);

  // Prevent the servos from panning farther than my head can rotate.
  // May not be necessary after adding 2nd gyro and using relative angles.
  if (invertedYaw > maxYaw) {
    invertedYaw = maxYaw;
  }
  else if (invertedYaw < minYaw) {
    invertedYaw = minYaw; 
  }

  // Prevent the servos from tilting farther than my head can nod.
  // May not be necessary after adding 2nd gyro and using relative angles.
  if (servoPitch > maxPitch) {
    servoPitch = maxPitch;
  }
  else if (servoPitch < minPitch) {
    servoPitch = minPitch;  
  }

  // Print out the current yaw and pitch angle values being sent to the servos.
  //  DPRINTSTIMER(100) {
  //    DPRINTSFN(15, "\tServo - Yaw:", servoYaw, 6, 1);
  //    DPRINTSFN(15, "\tServo - invertYaw:", invertedYaw, 6, 1);
  //    DPRINTSFN(15, "\tPitch:", servoPitch, 6, 1);
  //    DPRINTLN();
  //  }

  // Move the servos to the current yaw/pitch values
  yawServo.write(invertedYaw);
  pitchServo.write(servoPitch);

  // Reset the timer for comparison on the next loop
  prevMillis = currentMillis;

}


/**
 * =============================================================
 * ===                 Arduino Setup                         ===
 * =============================================================
 *
 * @author Mike Muscato
 * @date   2017-01-30
 *
 * @return {void}
 */
void setup() {
  Serial.begin(38400);
  while (!Serial);  // Wait for the connection to be established?

  // Run MPU initializations
  Serial.println(F("i2cSetup"));
  i2cSetup();
  Serial.println(F("MPU6050 Connection Routine"));
  MPU6050Connect();
  Serial.println(F("Setup complete"));

  pinMode(LED_PIN, OUTPUT);

  // Assign PWM Pins to servo signal wires
  yawServo.attach(9);
  pitchServo.attach(10);

  // Center the servo start positions
  yawServo.write(90);
  pitchServo.write(90);
}


/**
 * =============================================================
 * ===                  Arduino Loop                         ===
 * =============================================================
 *
 * @author Mike Muscato
 * @date   2017-01-30
 *
 * @return {void}
 */
void loop() {
  currentMillis = millis();   // Capture the latest value of millis()

  if (mpuInterrupt[0] && mpuInterrupt[1]) { // Wait for MPU interrupt or extra packet(s) available on both MPUs
    GetDMP();
  }

  // Uncomment for setting and adjusting the hardware start position. 
  // (e.g. Setting rotation and leveling the pan/tilt servo arms at initial 90-degree position)
  // IMPORTANT: Comment out the moveServos() method call in MPUMath() when using this. 
  // TODO: Remove in final program once hardware and software calibration is completed.
  // 
  // yawServo.write(90);
  // pitchServo.write(90);

}






















