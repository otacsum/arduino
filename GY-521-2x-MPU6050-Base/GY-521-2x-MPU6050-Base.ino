/**
 * GY-521-2x-MPU6050-Base.ino
 *
 * @author Mike Muscato
 * @date   2017-07-04
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

// DEBUG serial printing macros to limit serial monitor rate
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

const int LED_PIN = 13;

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
   
// vars for angle values.
float Yaw, Pitch, Roll;




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
    Yaw = (ypr[0] * 180.0 / M_PI);
    Pitch = (ypr[1] *  180.0 / M_PI);
    Roll = (ypr[2] *  180.0 / M_PI);

    // Serial.print the current angle values of the gyro position.
    // DPRINTSTIMER(x) argument is the number of milliseconds between print events
    // Smaller numbers give more reliable switching between MPU 0 an 1 but seem to cause crashing (Buffer overrun?)
    DPRINTSTIMER(1) {
      DPRINTSFN(15, "\tValues for MPU :", i, 6, 1);
      DPRINTSFN(15, "\tYaw:", Yaw, 6, 1);
      DPRINTSFN(15, "\tPitch:", Pitch, 6, 1);
      DPRINTSFN(15, "\tRoll:", Roll, 6, 1);
      DPRINTLN();
    }
    
  }
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
  if (mpuInterrupt[0] && mpuInterrupt[1]) { // Wait for MPU interrupt or extra packet(s) available on both MPUs
    GetDMP();
  }
}






















