#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "Servo.h"

//initialize gyro instance
//TODO:  Change this to initialize two gyros with +VDC/Ground arguments
MPU6050 mpu;

//initialize servo instances
Servo yawServo;
Servo pitchServo;

#define DEBUG  //Comment to turn off Serial printing

//Debug serial printing macros
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


//TODO:  Find out if it's really necessary to turn on this LED...probably not.
const int LED_PIN = 13;

//Prevent the servos from turning beyond mechanical limits 
const int maxPitch = 135;
const int minPitch = 45;
//Yaw direction is inverted via map() method in moveServos()
const int maxYaw = 135;
const int minYaw = 45;


// supply gyro offsets here, scaled for min sensitivity use MPU6050_calibration.ino
// Gyro 0 (head) calibrated at:   {-3074,  -2036,  1236,    63,    -16,     78}
//                     XA      YA      ZA      XG      YG      ZG
int MPUOffsets0[6] = {-3074,  -2036,  1236,    63,    -16,     78};

unsigned long currentMillis = 0;    // stores the value of millis() in each iteration of loop()
unsigned long prevMillis = 0;    // stores the value of millis() in each iteration of loop()

int pos = 0;    // variable to store the servo position when testing

// ================================================================
// ===                      i2c SETUP Items                     ===
// ================================================================
void i2cSetup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
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
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===                      MPU DMP SETUP                       ===
// ================================================================
int FifoAlive = 0; // tests if the interrupt is triggering
int IsAlive = -20;     // counts interrupt start at -20 to get 20+ good values before assuming connected
// MPU control/status vars
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
byte StartUP = 100; // lets get 100 readings from the MPU before we start trusting them (Bot is not trying to balance at this point it is just starting up.)

void MPU6050Connect() {
  static int MPUInitCntr = 0;
  // initialize device
  mpu.initialize(); // same
  // load and configure the DMP
  devStatus = mpu.dmpInitialize();// same

  if (devStatus != 0) {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)

    char * StatStr[5] { "No Error", "initial memory load failed", "DMP configuration updates failed", "3", "4"};

    MPUInitCntr++;

    Serial.print(F("MPU connection Try #"));
    Serial.println(MPUInitCntr);
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(StatStr[devStatus]);
    Serial.println(F(")"));

    if (MPUInitCntr >= 10) return; //only try 10 times
    delay(1000);
    MPU6050Connect(); // Lets try again
    return;
  }
  mpu.setXAccelOffset(MPUOffsets0[0]);
  mpu.setYAccelOffset(MPUOffsets0[1]);
  mpu.setZAccelOffset(MPUOffsets0[2]);
  mpu.setXGyroOffset(MPUOffsets0[3]);
  mpu.setYGyroOffset(MPUOffsets0[4]);
  mpu.setZGyroOffset(MPUOffsets0[5]);

  Serial.println(F("Enabling DMP..."));
  mpu.setDMPEnabled(true);
  // enable Arduino interrupt detection
  Serial.println(F("Enabling interrupt detection (Arduino external interrupt pin 2 on the Uno)..."));
  Serial.print("mpu.getInterruptDrive=  "); Serial.println(mpu.getInterruptDrive());
  attachInterrupt(0, dmpDataReady, RISING); //pin 2 on the Uno
  mpuIntStatus = mpu.getIntStatus(); // Same
  // get expected DMP packet size for later comparison
  packetSize = mpu.dmpGetFIFOPacketSize();
  delay(1000); // Let it Stabalize
  mpu.resetFIFO(); // Clear fifo buffer
  mpu.getIntStatus();
  mpuInterrupt = false; // wait for next interrupt
}


// ================================================================
// ===                    MPU DMP Get Data                      ===
// ================================================================
void GetDMP() { // Best version I have made so far
  // Serial.println(F("FIFO interrupt at:"));
  // Serial.println(micros());
  static unsigned long LastGoodPacketTime;
  mpuInterrupt = false;
  FifoAlive = 1;
  fifoCount = mpu.getFIFOCount();
  if ((!fifoCount) || (fifoCount % packetSize)) { // we have failed Reset and wait till next time!
    digitalWrite(LED_PIN, LOW); // lets turn off the blinking light so we can see we are failing.
    mpu.resetFIFO();// clear the buffer and start over
  } else {
    while (fifoCount  >= packetSize) { // Get the packets until we have the latest!
      mpu.getFIFOBytes(fifoBuffer, packetSize); // lets do the magic and get the data
      fifoCount -= packetSize;
    }
    LastGoodPacketTime = millis();
    MPUMath(); // <<<<<<<<<<<<<<<<<<<<<<<<<<<< On success MPUMath() <<<<<<<<<<<<<<<<<<<
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Blink the Light
  }
}


// ================================================================
// ===                        MPU Math                          ===
// ================================================================
float Yaw, Pitch, Roll;
void MPUMath() {
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  //TODO: Move the +90 gyro/servo leveling math to another method, 
  //it doesn't belong here and will cause readability problems later.
  Yaw = (ypr[0] * 180.0 / M_PI) + 90;
  Pitch = (ypr[1] *  180.0 / M_PI) + 90;
  Roll = (ypr[2] *  180.0 / M_PI) + 90;

  /*
  //Serial.log the current actual values of the gyro position.
  DPRINTSTIMER(100) {
    DPRINTSFN(15, "\tYaw:", Yaw, 6, 1);
    DPRINTSFN(15, "\tPitch:", Pitch, 6, 1);
    DPRINTSFN(15, "\tRoll:", Roll, 6, 1);
    DPRINTLN();
  }
  */

  if (currentMillis >= prevMillis + 15) {
    moveServos(round(Yaw), round(Pitch));
  }


}

/**
 * Move the servos to the gyro postion
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

  //TODO: Determine if yaw/pitch is beyond normal servo angles
  //i.e. > 179 or < 0
  //If so, assume actual gyro position and provide max servo value.
  //Create trimAngles() method.
  
  //Invert Yaw
  int invertedYaw = map(servoYaw, 0, 179, 179, 0);

  //Prevent the servos from moving farther than my head can rotate.
  if (invertedYaw > maxYaw) {
    invertedYaw = maxYaw;
  }
  else if (invertedYaw < minYaw) {
    invertedYaw = minYaw; 
  }

  //Prevent the servos from tilting farther than my head can nod.
  if (servoPitch > maxPitch) {
    servoPitch = maxPitch;
  }
  else if (servoPitch < minPitch) {
    servoPitch = minPitch;  
  }

  DPRINTSTIMER(100) {
    DPRINTSFN(15, "\tServo - Yaw:", servoYaw, 6, 1);
    DPRINTSFN(15, "\tServo - invertYaw:", invertedYaw, 6, 1);
    DPRINTSFN(15, "\tPitch:", servoPitch, 6, 1);
    DPRINTLN();
  }


  yawServo.write(invertedYaw);
  pitchServo.write(servoPitch);

  prevMillis = currentMillis;

}


// ================================================================
// ===                         Setup                            ===
// ================================================================
void setup() {
  Serial.begin(115200); //115200
  while (!Serial);
  Serial.println("i2cSetup");
  i2cSetup();
  Serial.println("MPU6050Connect");
  MPU6050Connect();
  Serial.println("Setup complete");
  pinMode(LED_PIN, OUTPUT);

  yawServo.attach(9);
  pitchServo.attach(10);

  yawServo.write(90);
  pitchServo.write(90);
}


// ================================================================
// ===                          Loop                            ===
// ================================================================
void loop() {
  currentMillis = millis();   // capture the latest value of millis()
  if (mpuInterrupt ) { // wait for MPU interrupt or extra packet(s) available
    GetDMP();
  }

  //for calibrating the start position in the hardware.  
  //TODO: Comment when completed or it will conflict with the 
  //motion of the robot.
  // yawServo.write(90);
  // pitchServo.write(90);

  //for testing hardware  
  // //TODO: Comment when completed
  // for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
  //   // in steps of 1 degree
  //   yawServo.write(pos);              // tell servo to go to position in variable 'pos'
  //   delay(20);                       // waits 15ms for the servo to reach the position
  // }
  // for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
  //   yawServo.write(pos);              // tell servo to go to position in variable 'pos'
  //   delay(20);                       // waits 15ms for the servo to reach the position
  // }
}





















