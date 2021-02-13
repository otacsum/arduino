#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>

#define TCAADDR 0x70

/* Assign a unique ID to the sensors */
Adafruit_9DOF                 dof1   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel1 = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag1   = Adafruit_LSM303_Mag_Unified(30302);

Adafruit_9DOF                 dof2   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel2 = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag2   = Adafruit_LSM303_Mag_Unified(30302);

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;


void tcaSelect(uint8_t i) 
{
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}


void displaySensorData( int port, 
                        Adafruit_9DOF dof, 
                        Adafruit_LSM303_Accel_Unified accel, 
                        Adafruit_LSM303_Mag_Unified   mag) 
{
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_vec_t   orientation;

  /* Calculate pitch and roll from the raw accelerometer data */
  accel.getEvent(&accel_event);
  if (dof.accelGetOrientation(&accel_event, &orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
    Serial.print(F("["));
    Serial.print(port);
    Serial.print(F("] "));
    Serial.print(F("Roll: "));
    Serial.print(orientation.roll);
    Serial.print(F("; "));
    Serial.print(F("Pitch: "));
    Serial.print(orientation.pitch);
    Serial.print(F("; "));
  }
  
  /* Calculate the heading using the magnetometer */
  mag.getEvent(&mag_event);
  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
  {
    /* 'orientation' should have valid .heading data now */
    Serial.print(F("Heading: "));
    Serial.print(orientation.heading);
    Serial.print(F("; "));
  }

}


void setup(void) 
{
  Wire.begin();
  Serial.begin(115200);
  Serial.println("9DOF Sensor Tests:"); Serial.println("");
  
  /* Init sensor 1 */
  tcaSelect(1);
  if(!accel1.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected (1-1)... Check your wiring!");
    while(1);
  }

  if(!mag1.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected (1-2)... Check your wiring!");
    while(1);
  }


  /* Init sensor 2 */
  tcaSelect(2);
  if(!accel2.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected (2)... Check your wiring!"));
    while(1);
  }

  if(!mag2.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected (2)... Check your wiring!");
    while(1);
  }
}

void loop(void) 
{
  tcaSelect(1);
  displaySensorData(1, dof1, accel1, mag1);
  tcaSelect(2);
  displaySensorData(2, dof2, accel2, mag2);

  Serial.println(F(""));
  delay(5);
}