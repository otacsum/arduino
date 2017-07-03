/**
 * ardu-predator.ino
 *
 * @author Mike Muscato
 * @date   2017-01-11
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

// Initialize gyro instance
// TODO:  Update this to initialize two gyros with +VDC/Ground arguments
MPU6050 mpuStatic(0x68);
MPU6050 mpuDynam(0x69);

// Initialize servo instances
Servo yawServo;
Servo pitchServo;


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


// TODO:  Find out if it's really necessary to turn on this LED...probably not.
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
// Gyro 0 (head) calibrated at:   {-3074,  -2036,  1236,    63,    -16,     78}
//                           XA      YA      ZA     XG      YG      ZG
int mpuStaticOffsets[6] = {-3074,  -2036,  1236,    63,    -16,     78};
int mpuDynamOffsets[6]  = {-1143,  -1197,   989,   147,    -78,    -10};

// var to store the servo position when testing
int pos = 0;    

// vars for angle values.
float Yaw, Pitch, Roll;

// Will be used for timing events.
unsigned long currentMillis = 0;    // stores the value of millis() in each iteration of loop()
unsigned long prevMillis = 0;    // stores the value of millis() in each iteration of loop()

// Will be used for switching readings so that we don't overrun the buffer
int whichMPU = 0;





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
volatile bool mpuInterrupt0 = false;     // indicates whether MPU interrupt pin has gone high
volatile bool mpuInterrupt1 = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady0() {
    mpuInterrupt0 = true;
}
void dmpDataReady1() {
    mpuInterrupt1 = true;
}



// ================================================================
// ===                      MPU DMP SETUP                       ===
// ================================================================
int FifoAlive0 = 0; // tests if the interrupt is triggering
int FifoAlive1 = 0; // tests if the interrupt is triggering
//int IsAlive = -20;     // counts interrupt start at -20 to get 20+ good values before assuming connected
// MPU control/status vars
uint8_t mpuIntStatus0;   // holds actual interrupt status byte from MPU
uint8_t devStatus0;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize0;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount0;     // count of all bytes currently in FIFO
uint8_t fifoBuffer0[64]; // FIFO storage buffer

uint8_t mpuIntStatus1;   // holds actual interrupt status byte from MPU
uint8_t devStatus1;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize1;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount1;     // count of all bytes currently in FIFO
uint8_t fifoBuffer1[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q0;           // [w, x, y, z]         quaternion container
Quaternion q1;           // [w, x, y, z]         quaternion container
//VectorInt16 aa;         // [x, y, z]            accel sensor measurements
//VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
//VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity0;    // [x, y, z]            gravity vector
VectorFloat gravity1;    // [x, y, z]            gravity vector
//float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr0[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float ypr1[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
//byte StartUP = 200; // lets get 200 readings from the MPU before we start trusting them (Bot is not trying to balance at this point it is just starting up.)



/**
 * MPU6050Connect
 * =============================================================
 *
 * @Edited by Mike Muscato to enable dual MPU configuration 
 * @date   2017-07-02
 *
 */
void MPU6050Connect(int connectMPU) {
  static int MPUInitCntr = 0;

  Serial.print(F("Initializing MPU ")); Serial.println(connectMPU);

  if (connectMPU == 0) {
    // initialize device
    mpuStatic.initialize();
    // load and configure the DMP
    devStatus0 = mpuStatic.dmpInitialize();
  }
  else {
    // initialize device
    mpuDynam.initialize();
    // load and configure the DMP
    devStatus1 = mpuDynam.dmpInitialize();
  }
  
  if (devStatus0 != 0) {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)

    char * StatStr[5] { "No Error", "initial memory load failed", "DMP configuration updates failed", "3", "4"};

    MPUInitCntr++;

    Serial.print(F("MPU connection Try #"));
    Serial.println(MPUInitCntr);
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(StatStr[devStatus0]);
    Serial.println(F(")"));

    if (MPUInitCntr >= 10) return; //only try 10 times
    delay(1000);
    MPU6050Connect(connectMPU); // Lets try again
    return;
  }

  if (connectMPU == 0) {
    mpuStatic.setXAccelOffset(mpuStaticOffsets[0]);
    mpuStatic.setYAccelOffset(mpuStaticOffsets[1]);
    mpuStatic.setZAccelOffset(mpuStaticOffsets[2]);
    mpuStatic.setXGyroOffset(mpuStaticOffsets[3]);
    mpuStatic.setYGyroOffset(mpuStaticOffsets[4]);
    mpuStatic.setZGyroOffset(mpuStaticOffsets[5]);

    Serial.println(F("Enabling DMP on Static MPU..."));
    mpuStatic.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection on Static MPU (Arduino external interrupt pin 2 on the Uno)..."));
    Serial.print(F("mpuStatic.getInterruptDrive=  ")); Serial.println(mpuStatic.getInterruptDrive());
    attachInterrupt(0, dmpDataReady0, RISING); //pin 2 on the Uno
    mpuIntStatus0 = mpuStatic.getIntStatus();
    // get expected DMP packet size for later comparison
    packetSize0 = mpuStatic.dmpGetFIFOPacketSize();
    delay(1000); // Let it Stabalize
    mpuStatic.resetFIFO(); // Clear fifo buffer
    mpuStatic.getIntStatus();
    mpuInterrupt0 = false; // wait for next interrupt
  }
  else {
    mpuDynam.setXAccelOffset(mpuDynamOffsets[0]);
    mpuDynam.setYAccelOffset(mpuDynamOffsets[1]);
    mpuDynam.setZAccelOffset(mpuDynamOffsets[2]);
    mpuDynam.setXGyroOffset(mpuDynamOffsets[3]);
    mpuDynam.setYGyroOffset(mpuDynamOffsets[4]);
    mpuDynam.setZGyroOffset(mpuDynamOffsets[5]);

    Serial.println(F("Enabling DMP on Dyanmic MPU..."));
    mpuDynam.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection on Dynamic MPU (Arduino external interrupt pin 3 on the Uno)..."));
    Serial.print(F("mpuDynam.getInterruptDrive=  ")); Serial.println((mpuDynam.getInterruptDrive()));
    attachInterrupt(1, dmpDataReady1, RISING); //pin 3 on the Uno
    mpuIntStatus1 = mpuDynam.getIntStatus();
    // get expected DMP packet size for later comparison
    packetSize1 = mpuDynam.dmpGetFIFOPacketSize();
    delay(1000); // Let it Stabalize
    mpuDynam.resetFIFO(); // Clear fifo buffer
    mpuDynam.getIntStatus();
    mpuInterrupt1 = false; // wait for next interrupt
  }

}



// ================================================================
// ===                    MPU DMP Get Data                      ===
// ================================================================
void GetDMP(int dmpMPU) { // Best version I have made so far
  //Serial.println("dmpMPU");
  // Serial.println(F("FIFO interrupt at:"));
  // Serial.println(micros());

  if (dmpMPU == 0) {
    static unsigned long LastGoodPacketTime;
    mpuInterrupt0 = false;
    FifoAlive0 = 1;
    fifoCount0 = mpuStatic.getFIFOCount();
    if ((!fifoCount0) || (fifoCount0 % packetSize0)) { // we have failed Reset and wait till next time!
      digitalWrite(LED_PIN, LOW); // lets turn off the blinking light so we can see we are failing.
        mpuStatic.resetFIFO();// clear the buffer and start over
    } 
    else {
      while (fifoCount0  >= packetSize0) { // Get the packets until we have the latest!
          mpuStatic.getFIFOBytes(fifoBuffer0, packetSize0); // lets do the magic and get the data
          fifoCount0 -= packetSize0;
      }
      LastGoodPacketTime = millis();
      MPUMath(dmpMPU); // <<<<<<<<<<<<<<<<<<<<<<<<<<<< On success MPUMath() <<<<<<<<<<<<<<<<<<<
      digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Blink the Light
    }
  }
  else {
    static unsigned long LastGoodPacketTime;
    mpuInterrupt1 = false;
    FifoAlive1 = 1;
    fifoCount1 = mpuDynam.getFIFOCount();
    if ((!fifoCount1) || (fifoCount1 % packetSize1)) { // we have failed Reset and wait till next time!
      digitalWrite(LED_PIN, LOW); // lets turn off the blinking light so we can see we are failing.
        mpuDynam.resetFIFO();// clear the buffer and start over
    } 
    else {
      while (fifoCount1  >= packetSize1) { // Get the packets until we have the latest!
          mpuDynam.getFIFOBytes(fifoBuffer1, packetSize1); // lets do the magic and get the data 
          fifoCount1 -= packetSize1;
      }
      LastGoodPacketTime = millis();
      MPUMath(dmpMPU); // <<<<<<<<<<<<<<<<<<<<<<<<<<<< On success MPUMath() <<<<<<<<<<<<<<<<<<<
      digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Blink the Light
    } 
  }
}


// ================================================================
// ===                        MPU Math                          ===
// ================================================================
void MPUMath(int mathMPU) {
  //Serial.println("mathMPU");
  if (mathMPU == 0) {
    mpuStatic.dmpGetQuaternion(&q0, fifoBuffer0);
    mpuStatic.dmpGetGravity(&gravity0, &q0);
    mpuStatic.dmpGetYawPitchRoll(ypr0, &q0, &gravity0);

    // Add 90-degrees to center servos when sensor is level (e.g. 0 degrees)
    // TODO: Move the +90 gyro/servo leveling magic number to another method, 
    // ...it doesn't belong here and will cause readability problems later.
    Yaw = (ypr0[0] * 180.0 / M_PI) + 90;
    Pitch = (ypr0[1] *  180.0 / M_PI) + 90;
    Roll = (ypr0[2] *  180.0 / M_PI) + 90;
  }
  else {
    mpuDynam.dmpGetQuaternion(&q1, fifoBuffer1);
    mpuDynam.dmpGetGravity(&gravity1, &q1);
    mpuDynam.dmpGetYawPitchRoll(ypr1, &q1, &gravity1);

    // Add 90-degrees to center servos when sensor is level (e.g. 0 degrees)
    // TODO: Move the +90 gyro/servo leveling magic number to another method, 
    // ...it doesn't belong here and will cause readability problems later.
    Yaw = (ypr1[0] * 180.0 / M_PI) + 90;
    Pitch = (ypr1[1] *  180.0 / M_PI) + 90;
    Roll = (ypr1[2] *  180.0 / M_PI) + 90;
  }


  Serial.print(F("\tValues for MPU :"));Serial.print(mathMPU);
  Serial.print(F("\tYaw:"));Serial.print(Yaw);
  Serial.print(F("\tPitch:"));Serial.print(Pitch);
  Serial.print(F("\tRoll:"));Serial.println(Roll);
  
  // Serial.print the current actual values of the gyro position.
//  DPRINTSTIMER(100) {
//    DPRINTSFN(15, "\tValues for MPU :", mathMPU, 6, 1);
//    DPRINTSFN(15, "\tYaw:", Yaw, 6, 1);
//    DPRINTSFN(15, "\tPitch:", Pitch, 6, 1);
//    DPRINTSFN(15, "\tRoll:", Roll, 6, 1);
//    DPRINTLN();
//  }
  

/*
  // Timer - don't try and move the servos too frequently, they need time to get to the current position.
  // TODO: Remove delay magic number and move to global var for tuning mechanical
  if (currentMillis >= prevMillis + 15) {
    moveServos(round(Yaw), round(Pitch));
  }
*/

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
  if (invertedYaw > maxYaw) {
    invertedYaw = maxYaw;
  }
  else if (invertedYaw < minYaw) {
    invertedYaw = minYaw; 
  }

  // Prevent the servos from tilting farther than my head can nod.
  if (servoPitch > maxPitch) {
    servoPitch = maxPitch;
  }
  else if (servoPitch < minPitch) {
    servoPitch = minPitch;  
  }

  // Print out the current yaw and pitch angle values being sent to the servos.
  DPRINTSTIMER(100) {
    DPRINTSFN(15, "\tServo - Yaw:", servoYaw, 6, 1);
    DPRINTSFN(15, "\tServo - invertYaw:", invertedYaw, 6, 1);
    DPRINTSFN(15, "\tPitch:", servoPitch, 6, 1);
    DPRINTLN();
  }

  // Move the servos to the current yaw/pitch values
  //yawServo.write(invertedYaw);
  //pitchServo.write(servoPitch);

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
  Serial.begin(115200);
  while (!Serial);  // Wait for the connection to be established?

  // Run MPU initializations
  Serial.println(F("i2cSetup"));
  i2cSetup();
  Serial.println(F("MPU6050Connect"));
  MPU6050Connect(0);
  MPU6050Connect(1);
  Serial.println(F("Setup complete"));

  pinMode(LED_PIN, OUTPUT);

  // Assign PWM Pins to servo signal wires
  yawServo.attach(9);
  pitchServo.attach(10);

  // Center the servos' start position
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


    if (mpuInterrupt0) { // Wait for MPU interrupt or extra packet(s) available
      GetDMP(0);
    }
 
    if (mpuInterrupt1) { // Wait for MPU interrupt or extra packet(s) available
      GetDMP(1);
    } 



  // Uncomment for setting and adjusting the hardware start position. 
  // (e.g. Setting rotation and leveling the pan/tilt servo arms at initial 90-degree position)
  // IMPORTANT: Comment out the moveServos() method call in MPUMath() when using this. 
  // TODO: Remove in final program once hardware and software calibration is completed.
  // 
  // yawServo.write(90);
  // pitchServo.write(90);

}






















