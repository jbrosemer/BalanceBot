// WRITTEN BY JORDYN BROSEMER FOR ME134 TUFTS UNIVERSITY BALANCE BOT 
// WALLACE B ANNOYING 
// AKA 
// WALL-B
// Include Wire Library for I2C
#include <Wire.h>
// Include Lidar Library
#include "Adafruit_VL53L0X.h"
//set new Lidar Object
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
// define MPU hex i2c address
const int MPU_addr=0x68;
//16 bit ints to temporarily store raw gyro and acceleration data
int16_t GrawX, GrawY, RawAcX, RawAcY, RawAcZ;
//defines the angles to send to servos
int angle, otherangle;
//all floats that will be set later
float GyX,GyY,GyZ, Acceleration_angleX, timer, timePrev, TimePast, Total_angle, angleerror, preverror, pidd, pidp, pid, pidi;
//gyro gain as defined in IMU datasheet
float gyro_gain = 131.0;
// angle the robot should be at when sitting flat / straight up
float idealangle = 4;
// PID Constants, probably should increase Kd, but will need to adjust mapping of angles to servo values to compensate for this error
float Kp = 20;
float Ki = 0.000000;
float Kd = 0.15;
// Include Adafruit PCA9685 Servo Library
#include <Adafruit_PWMServoDriver.h>
#define SERVOMIN  286 // Minimum servo value for max speed in CCW direction
#define SERVOMAX  450  // Maximum value value for max speed in CW direction

// Creat object to represent PCA9685 at default I2C address
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);

#define SER0  0  //Servo Motor 0 on connector 0
#define SER1  1  //Servo Motor 1 on connector 1
#define SER2  2  //Servo Motor 2 on connector 2

//setup loop
void setup() {
  // put your setup code here, to run once:
  // Serial monitor setup
  Serial.begin(115200);
  //i2c wire startup
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  //write from 107 from the mpu, the 0 resets it.
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  // starts the timer
  timer = millis(); 

    // Initialize PCA9685
  pca9685.begin();
 
  // Set PWM Frequency to 60
  pca9685.setPWMFreq(60);
}

void loop() {
  //VL53L0X_RangingMeasurementData_t measure;
  // Serial.print("Reading a measurement... ");
  /*
  if (measure.RangeStatus != 4)
  { // phase failures have incorrect data
  Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
  }
  else
  {
  Serial.println(" out of range ");
  }
  if(measure.RangeMilliMeter > 200){
    pca9685.setPWM(SER2, 0, 400);
  }
  else{
    pca9685.setPWM(SER2, 1000, 120);
  }
  */
  //updates timer
  timePrev = timer;
  // update angle error for PID
  preverror = angleerror;
  //gets time that has past
  timer = millis();
  //finds the amount of time between last error and current error in seconds
  TimePast = (timer - timePrev) / 1000;
  //begins talking to imu
  Wire.beginTransmission(MPU_addr);
  //0x3b in hex is 59, registers 59-64 are the accelerometer values.
  Wire.write(0x3B);
  Wire.endTransmission(false);
  //read the 6 registers after 59
  Wire.requestFrom(MPU_addr,6,true);
  //read the accelerometer register and combine the high portion * 256 + low portion bit shifting left is multiplying by 256
  RawAcX=Wire.read()<<8|Wire.read();
  RawAcY=Wire.read()<<8|Wire.read();
  RawAcZ=Wire.read()<<8|Wire.read();
  //converts the raw accelerometer values to the acceleration angle x. 16384 is 2G which is the default accelerometer setting
  Acceleration_angleX = atan((RawAcY/16384.0)/sqrt(pow((RawAcX/16384.0),2) + pow((RawAcZ/16384.0),2)))*RAD_TO_DEG;
  //re-enable comms with the IMU
  Wire.beginTransmission(0x68);
  //0x43 in hex is 67 in decimal, registers 67-70 are the raw gyro readings.
  Wire.write(0x43); 
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,4,true); 
  //same processes as before to get raw gyro
  GrawX=Wire.read()<<8|Wire.read(); 
  GrawY=Wire.read()<<8|Wire.read(); 
  //divides the raw value by the gyro gain defined in the datasheet
  GyX = GrawX / gyro_gain;
  GyY = GrawY / gyro_gain;
  // take the complimentary filter of the angles from gyro and acceleration
  Total_angle = 0.96 *(Total_angle + GyX*TimePast) + 0.04*Acceleration_angleX;
  //find the error of the imu angle vs the vertical angle
  angleerror = Total_angle - idealangle;
  //Use PID constants to reduce instability
  //error * kp constant is the pid-proportional gain
  pidp = Kp*angleerror;
  //pid i was useless
  //the change in error divided by the time that has past between those two readings is the pid-derivative gain
  pidd = Kd*((angleerror - preverror)/TimePast);
  //sum all pid gains
  pid = pidp + pidd + pidi;
  //take the abs value 
  pid = abs(pid);
  //map pid value to servo speeds. 367 is the midpoint i.e not moving goes cw or ccw
  angle = map(pid,15,90,367,SERVOMAX);
  otherangle = map(pid,15,90,367,SERVOMIN);
  //printing for debugging purposes- most likely slows the loop down so uncomment to debug
  //Serial.println(Total_angle);

  //this is the raresst case, if he needs to pick himself up
  if(Total_angle > 75){
  VL53L0X_RangingMeasurementData_t measure;
  //will only pick himself up if the Lidar also says he is not standing up.
    if(measure.RangeMilliMeter > 100){
      //sends noncontinuous servo to lift position
      pca9685.setPWM(SER2,0,450);
      //waits to lift
      delay(500);
      //rotates wheels to help pick it up
      pca9685.setPWM(SER0, 0, SERVOMAX);
      pca9685.setPWM(SER1, 0, SERVOMIN);
    }
  }
  //case logic to rotate wheels in the appropriate direction if it is falling
  else if(Total_angle > idealangle+4){
  //if the angle is too just sets the motors to their associated maxs
  //when feeding the angle when PID was too high resulted in crashes.
  if(Total_angle < idealangle+10){
    pca9685.setPWM(SER0, 0, SERVOMIN);
    pca9685.setPWM(SER1, 0, SERVOMAX);
  }
  //these also set the noncontinuous servo to hold its lifting arm up kind of like Rocky
  else{
    pca9685.setPWM(SER0, 0, otherangle);
    pca9685.setPWM(SER1, 0, angle);
    pca9685.setPWM(SER2,0,180);
  }
  }
  else if(Total_angle < idealangle-4){
  if(Total_angle < idealangle-10){
    pca9685.setPWM(SER0, 0, SERVOMAX);
    pca9685.setPWM(SER1, 0, SERVOMIN);
  }
  else{
    pca9685.setPWM(SER0, 0, angle);
    pca9685.setPWM(SER1, 0, otherangle);
    pca9685.setPWM(SER2,0,180);
      }
  }
  //cuts the motors when it is standing within +-4 degrees of the ideal
  else{
  pca9685.setPWM(SER0, 0, 0);
  pca9685.setPWM(SER1, 0, 0);
  pca9685.setPWM(SER2,0,180);
  }
}
