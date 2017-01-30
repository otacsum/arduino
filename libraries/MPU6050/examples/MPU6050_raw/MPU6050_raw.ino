// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added multiple output formats
//                 - added seamless Fastwire support
//      2011-10-07 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu0(0x68);
MPU6050 mpu1(0x69);

int16_t ax0, ay0, az0;
int16_t gx0, gy0, gz0;

int16_t ax1, ay1, az1;
int16_t gx1, gy1, gz1;

// supply gyro offsets here, scaled for min sensitivity use MPU6050_calibration.ino
// Gyro 0 (head) calibrated at:   {-3074,  -2036,  1236,    63,    -16,     78}
// Gyro 1 (fixed) calibrated at:  {-1143,  -1197,   989,   147,    -78,    -10}  //May need yaw calibration.
//                     XA      YA      ZA      XG      YG      ZG
int MPUOffsets0[6] = {-3074,  -2036,  1236,    63,    -16,     78};
int MPUOffsets1[6] = {-1143,  -1197,   989,   147,    -78,    -10};



// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO


#define LED_PIN 13
bool blinkState = false;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    mpu0.initialize();
    mpu1.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(mpu0.testConnection() ? "MPU6050 connection 0 successful" : "MPU6050 connection 0 failed");
    Serial.println(mpu1.testConnection() ? "MPU6050 connection 1 successful" : "MPU6050 connection 1 failed");

    // use the code below to change accel/gyro offset values
    
    Serial.println("Updating internal sensor offsets...");

      mpu0.setXAccelOffset(MPUOffsets0[0]);
      mpu0.setYAccelOffset(MPUOffsets0[1]);
      mpu0.setZAccelOffset(MPUOffsets0[2]);
      mpu0.setXGyroOffset(MPUOffsets0[3]);
      mpu0.setYGyroOffset(MPUOffsets0[4]);
      mpu0.setZGyroOffset(MPUOffsets0[5]);

      mpu1.setXAccelOffset(MPUOffsets1[0]);
      mpu1.setYAccelOffset(MPUOffsets1[1]);
      mpu1.setZAccelOffset(MPUOffsets1[2]);
      mpu1.setXGyroOffset(MPUOffsets1[3]);
      mpu1.setYGyroOffset(MPUOffsets1[4]);
      mpu1.setZGyroOffset(MPUOffsets1[5]);
    

    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
    // read raw accel/gyro measurements from device
    mpu0.getMotion6(&ax0, &ay0, &az0, &gx0, &gy0, &gz0);
    mpu1.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);

    #ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
        Serial.print("a/g 0:\t");
        Serial.print(ax0); Serial.print("\t");
        Serial.print(ay0); Serial.print("\t");
        Serial.print(az0); Serial.print("\t");
        Serial.print(gx0); Serial.print("\t");
        Serial.print(gy0); Serial.print("\t");
        Serial.println(gz0);

        Serial.print("a/g 1:\t");
        Serial.print(ax1); Serial.print("\t");
        Serial.print(ay1); Serial.print("\t");
        Serial.print(az1); Serial.print("\t");
        Serial.print(gx1); Serial.print("\t");
        Serial.print(gy1); Serial.print("\t");
        Serial.println(gz1);
    #endif

    #ifdef OUTPUT_BINARY_ACCELGYRO
        Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
        Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
        Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
        Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
        Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
        Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
    #endif

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}
