#include <Wire.h>
#include "I2Cdev.h"
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTFusionRTQF.h" 
#include "CalLib.h"
#include <EEPROM.h>
#include <Servo.h>

Servo leftServo;
Servo rightServo; 
RTIMU *imu;                                           // the IMU object
RTFusionRTQF fusion;                                  // the fusion object
RTIMUSettings settings;                               // the settings object

#define DISPLAY_INTERVAL  300                         // interval between pose displays
#define  SERIAL_PORT_SPEED  115200
RTVector3 RYP;
int pitchSetPoint=85;
int servoPinLeft,servoPinRight;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 2.2;                  //Gain setting for the roll P-controller (1.3)
float pid_i_gain_roll = 0.0006;             //Gain setting for the roll I-controller (0.05)
float pid_d_gain_roll = 2.5;                //Gain setting for the roll D-controller (15)
int pid_max_roll = 255;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 5;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 1;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 255;                     //Maximum output of the PID-controller (+/-)
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
    int errcode;
    Serial.begin(SERIAL_PORT_SPEED);
    Wire.begin();
    imu = RTIMU::createIMU(&settings);                        // create the imu object
  
    Serial.print("ArduinoIMU starting using device "); Serial.println(imu->IMUName());
    if ((errcode = imu->IMUInit()) < 0) {
        Serial.print("Failed to init IMU: "); Serial.println(errcode);
    }
  
    if (imu->getCalibrationValid())
        Serial.println("Using compass calibration");
    else
        Serial.println("No valid compass calibration data");

//    lastDisplay = lastRate = millis();
//    sampleCount = 0;

    // Slerp power controls the fusion and can be between 0 and 1
    // 0 means that only gyros are used, 1 means that only accels/compass are used
    // In-between gives the fusion mix.
    
    fusion.setSlerpPower(0.02);
    
    // use of sensors in the fusion algorithm can be controlled here
    // change any of these to false to disable that sensor
    
    fusion.setGyroEnable(true);
    fusion.setAccelEnable(true);
    fusion.setCompassEnable(true);
    leftServo.attach(servoPinLeft);
    rightServo.attach(servoPinRight);
  
}
void loop()
{
  int loopCount=1;
  while(imu->IMURead()){
    
    if (++loopCount>=10)
          continue;
    fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
    RYP = fusion.getFusionPose();
/*    Serial.print(RYP.x()*180/3.14);
    Serial.print(" ");
    Serial.print(RYP.y()*180/3.14);
    Serial.print(" ");
    Serial.println(RYP.z()*180/3.14); */
    calculatePID();
    leftPos = pid_output_pitch*(1);
    rightPos = pid_output_pitch*(1); 
    leftServo.write(leftPos);
    rightServo.write(rightPos);
    
  }
  
}

void calculatePID(){
  
  /Pitch calculations
  pid_error_temp = RPY.y() + pitchSetPoint ; //you need to sort this before flying;
  //Serial.println(gyro_pitch_input);
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;
  
  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;
  
  pid_last_pitch_d_error = pid_error_temp;
/*
  //Roll calculations
  pid_error_temp = RPY.x() + rollSetPoint; //you need to sort this before flying;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;
  
  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;
  
  pid_last_roll_d_error = pid_error_temp;
*/
}
