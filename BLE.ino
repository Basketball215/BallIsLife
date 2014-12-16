#include <SPI.h>
#include <Adafruit_BLE_UART.h>
#include <motordriver_4wd.h>
#include <seeed_pwm.h>

// Adafruit gyro
#include <Wire.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>

// Connect CLK/MISO/MOSI to hardware SPI
// e.g. On UNO & compatible: CLK = 13, MISO = 12, MOSI = 11
#define ADAFRUITBLE_REQ 3
#define ADAFRUITBLE_RDY 2     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST PC6

// Adafruit BLE
Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);

// Adafruit gyro
Adafruit_9DOF                dof   = Adafruit_9DOF();
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);

/* Update this with the correct SLP for accurate altitude measurements */
short x = 0;
short y = 0;
float yaw_bias = 0.0;
float current_angle = 0.0;
float desired_angle = 0.0;
float alpha = 0.99;
float current_pwm = 40;
String state = "STRAIGHT";

//Orientation Global
sensors_vec_t   orientation;
aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

/**************************************************************************/
/*!
    Setup
*/
/**************************************************************************/
void setup(void)
{ 
  
  MOTOR.init();

  Serial.begin(115200);
  while(!Serial); // Leonardo/Micro should wait for serial init
  
  Serial.println(F("Starting"));
  BTLEserial.setDeviceName("BALLIFE"); /* 7 characters max! */
  BTLEserial.begin();
  
//  Serial.println(F("Adafruit 9 DOF Pitch/Roll/Heading Example")); Serial.println("");
  initSensors();  
}

/**************************************************************************/
/*!
    Adafruit gyro methods
*/
/**************************************************************************/
void initSensors()
{
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
}

void getGyroData() 
{
  // Retrieve gyro Data
  sensors_event_t mag_event;
  
   /* Calculate the heading using the magnetometer */
  mag.getEvent(&mag_event);
  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
  {
    /* 'orientation' should have valid .heading data now */
//    Serial.print(F("Heading: "));
//    Serial.print(orientation.heading);
//    Serial.print(F("; \n"));
  }
}

/**************************************************************************/
/*!
    Adafruit BLE methods
*/
/**************************************************************************/

void handleBLEData()
{  
  // Tell the nRF8001 to do whatever it should be working on.
  BTLEserial.pollACI();

  // Ask what is our current status
  aci_evt_opcode_t status = BTLEserial.getState();
  // If the status changed....
  if (status != laststatus) {
    // print it out!
    if (status == ACI_EVT_DEVICE_STARTED) {
        Serial.println(F("* Advertising started"));
    }
    if (status == ACI_EVT_CONNECTED) {
        Serial.println(F("* Connected!"));
    }
    if (status == ACI_EVT_DISCONNECTED) {
        Serial.println(F("* Disconnected or advertising timed out"));
    }
    // OK set the last status change to this one
    laststatus = status;
  }

  if (status == ACI_EVT_CONNECTED) {
    // Lets see if there's any data for us!
    if (BTLEserial.available()) {
      Serial.print("* "); Serial.print(BTLEserial.available()); Serial.println(F(" bytes available from BTLE"));
    }
    while(BTLEserial.available()) {
      x = readCoordinate();
      y = readCoordinate();
      Serial.print("x: "); Serial.print(x); Serial.print(", y: "); Serial.println(y);
      desired_angle = current_angle - 180 * atan2(y,x) / M_PI;
    }
  } 
}

short readCoordinate() {
  return (BTLEserial.read() << 8 | BTLEserial.read()) - 32768;
}

/**************************************************************************/
/*!
    Navigation Methods
*/
/**************************************************************************/
void getDestinationAngle()
{
  
}

void turnLeft() {
  MOTOR.setSpeedDir1(60, DIRF);
  MOTOR.setSpeedDir2(60, DIRR);
}


void turnRight() {
  MOTOR.setSpeedDir1(60, DIRR);
  MOTOR.setSpeedDir2(60, DIRF);
}

void stopTurns() {
  MOTOR.setSpeedDir1(0, DIRF);
  MOTOR.setSpeedDir2(0, DIRR);
}
/**************************************************************************/
/*!
    Run Loop
*/
/**************************************************************************/
void loop()
{  
  handleBLEData();
  getGyroData();
  current_angle = orientation.heading - 180;
  Serial.println(orientation.heading);
  Serial.print("desired angle: "); Serial.print(desired_angle); Serial.print(" current_angle: "); Serial.print(current_angle); Serial.print(" desired - current: "); Serial.println(desired_angle-current_angle);
  float delta = desired_angle - current_angle;
  if(delta < -180) {
    delta += 360;
  }
  if(delta > 180) {
    delta -= 360;
  }
  if (delta > 10) {
//    Serial.println("turn right");
    turnRight();
  } else if (delta < -10) {
//    Serial.println("turn left");
    turnLeft();
  } else {
    Serial.println("within thresh");
    stopTurns();
  }
  delay(10);
  return;
//  
//  MOTOR.setSpeedDir1(50, DIRF);
//  MOTOR.setSpeedDir2(50, DIRR);
//  delay(1000);
//  MOTOR.setSpeedDir1(50, DIRR);
//  MOTOR.setSpeedDir2(50, DIRF);
//  delay(1000);
}

