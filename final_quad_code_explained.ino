#include <Wire.h>                          //Include the Wire.h library so we can communicate with the gyro.
#include "I2Cdev.h"                        //Include the I2Cdev library so we can communicate with the gyro using i2c communication protocol.
#include <SPI.h>                           //Include the SPI library so we can communicate with the NRF24l01 using SPI communication protocol.
#include "RF24.h"
#include "MPU6050_6Axis_MotionApps20.h"


RF24 radio(8,10);
MPU6050 mpu;


#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards0

// MPU control/status vars

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
int16_t gyro[3];
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int16_t mx, my, mz;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


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

float pid_p_gain_yaw = 5;//.05;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0;//.0001;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 1;//.04;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 255;                     //Maximum output of the PID-controller (+/-)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
byte eeprom_data[36];
byte highByte, lowByte;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
int esc_1, esc_2, esc_3, esc_4;
int throttle, battery_voltage;
int cal_int, start = 0, gyro_address=8;
int receiver_input[5];
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long loop_timer;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_axis[4], gyro_axis_cal[4];
float pid_error_temp,roll_receiver,pitch_receiver;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
void set_gyro_registers();
void calculate_pid();
int rate_gyro_roll_input,yaw_receiver=0,paitch_receiver=0;
int rate_gyro_pitch_input;
int rate_gyro_yaw_input,offset_yaw=0;
const uint64_t pipe = 0xE8E8F0F0E1LL;
int msg[4];                                                                                                             //msg
int sensorValue = 0;                           // value read from the pot
int outputValue = 0;                           // value output to the PWM (analog out)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){



     pinMode(3, OUTPUT);                         //
     pinMode(5, OUTPUT);
     pinMode(6, OUTPUT);
     pinMode(9, OUTPUT);
     
      
     radio.begin();
     radio.openReadingPipe(1,pipe);
     radio.startListening();
     
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); 
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(38400);
    while (!Serial); 

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } 
    else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

 //mpu.setI2CBypassEnabled(true); // set bypass mode
// Now we can talk to the HMC5883l

// ==================== HMC5883L ============================
 // mag.initialize();
 // Serial.print("Testing Mag...  ");
 // Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");


    

  //Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs.
  DDRD |= B01101000;                                           //Configure digital port 3, 5 and 6 as output.
  DDRB |= B00000010;                                           //Configure digital port 9 as output.
    
  //Use the led on the Arduin0X68o for startup indication.
  digitalWrite(12,HIGH);                                       //Turn on the warning led.
  
  //set_gyro_registers();                                        //Set the specific gyro registers.
  
  for (cal_int = 0; cal_int < 1250 ; cal_int ++){              //Wait 5 seconds before continuing.
    PORTD |= B01101000;                                        //Set digital port 3, 5 and 6 high.
    PORTB |= B00000010;                                        //Set digital port 9 high.
    delayMicroseconds(100);                                   //Wait 1000us.
    
    PORTD &= B10010111;                                        //Set digital port 3, 5 and 6 low.
    PORTB &= B11111101;                                        //Set digital port 9 low.
    delayMicroseconds(3900);                                   //Wait 3000us.
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){
   //digitalWrite(4,HIGH );
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            mpu.dmpGetGyro(gyro, fifoBuffer);
        #endif
    }
    rate_gyro_yaw_input=gyro[2];
    rate_gyro_pitch_input=gyro[1];
    rate_gyro_roll_input=gyro[0];
    gyro_roll_input = ypr[2] * 180/M_PI ;            
    gyro_pitch_input = ypr[1] * 180/M_PI;         
    gyro_yaw_input = ypr[0] * 180/M_PI  ;             
   /*
    Serial.print(gyro[0]);
    Serial.print("\t");
    Serial.print(gyro[1]);
    Serial.print("\t");
    Serial.print(gyro[2]);
    Serial.print("\t");
    Serial.print(gyro_yaw_input);
    Serial.print("\t");
    Serial.print(gyro_pitch_input);
    Serial.print("\t");
    Serial.print(gyro_roll_input);
    Serial.print("\n");
   */
    // read raw heading measurements
   // mag.getHeading(&mx, &my, &mz);

    // display tab-separated mag x/y/z values
    //Serial.print(mx); Serial.println("\t");
    //Serial.print(my); Serial.print("\t");
    //Serial.print(mz ); Serial.print("\n");
    
    // To calculate heading in degrees. 0 degree indicates North
   // float heading = atan2(my, mx);
   // if(heading < 0) heading += 2 * M_PI;
   // heading = heading * 180/M_PI;
   // if(heading>0 && heading<180)
   // {
   // heading=map(heading,0,180,0,90);
   // }
   // else
   // {
   // heading=map(heading,180,359,90,359);
   // }
    
   // Serial.print(heading); Serial.print("\n");
    
      
   if (radio.available())
    {
    bool done = false;
    while (!done)
       {
       done = radio.read(&msg,sizeof(msg));
           
        Serial.print(msg[0]);
        Serial.print("\t");
        Serial.print(msg[1]);
        Serial.print("\t");
        Serial.print(msg[2]);
        Serial.print("\t");
        Serial.println(msg[3]);
        
        delay(5);
        
       }
    }
     
    throttle = msg[0]; 
    yaw_receiver = msg[1];
    
    roll_receiver = msg[2]; 
    if (roll_receiver <= -40)
    {
      roll_receiver = -40;
      }
    else if (roll_receiver >= 40)
    {
      roll_receiver = 40;
      }
      
    pitch_receiver = msg[3];
    
    if (pitch_receiver <= -40)
    {
      pitch_receiver = -40;
      }
    else if (pitch_receiver >= 40)
    {
      pitch_receiver = 40;
      }
   
   
   //For starting the motors: throttle low 
    if(throttle == 0 && start < 1000)
   {
    //Reset the pid controllers for a bumpless start.
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
    offset_yaw = gyro_yaw_input;
    start =start+ 1;
   }
   
  //PID inputs are known. So we can calculate the pid output.
  calculate_pid();

   if(start == 1000)
  {                                                          // sensor readings stabilized
                                                                               //We need some room to keep full control at full throttle.
    
    esc_1 = throttle - pid_output_roll- pid_output_pitch - pid_output_yaw;// Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = throttle - pid_output_roll + pid_output_pitch + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = throttle + pid_output_roll + pid_output_pitch - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = throttle + pid_output_roll - pid_output_pitch + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

     if(esc_1 > 250)esc_1 = 250;                                           //Limit the esc-1 pulse to 2000us.
    if(esc_1 < 0)esc_1 = 0;
    if(esc_2 > 250)esc_2 = 250;                                           //Limit the esc-2 pulse to 2000us.
    if(esc_2 < 0)esc_2 = 0;
    if(esc_3 > 250)esc_3 = 250;                                           //Limit the esc-1 pulse to 2000us.
    if(esc_3 < 0)esc_3 = 0;
    if(esc_4 > 250)esc_4 = 250;                                           //Limit the esc-2 pulse to 2000us.
    if(esc_4 < 0)esc_4 = 0;
    }
  
       
       analogWrite(3,esc_1 );
       analogWrite(5,esc_2 );
       analogWrite(6,esc_3 );
       analogWrite(9,esc_4 );
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for calculating pid outputs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//The PID controllers are explained in part 5 of the YMFC-3D video session:
//www.youtube.com/watch?v=JBvnB0279-Q

void calculate_pid(){
  //Roll calculations
  pid_error_temp = rate_gyro_roll_input + (gyro_roll_input) + roll_receiver; //you need to sort this before flying;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;
  
  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;
  
  pid_last_roll_d_error = pid_error_temp;
  
  //Pitch calculations
  pid_error_temp = rate_gyro_pitch_input - (gyro_pitch_input)+ pitch_receiver ; //you need to sort this before flying;
  //Serial.println(gyro_pitch_input);
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;
  
  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;
        
  pid_last_pitch_d_error = pid_error_temp;
    
  //Yaw calculations
  pid_error_temp = -rate_gyro_yaw_input;// - 0.5*gyro_yaw_input + 0.5*offset;// + yaw_receiver; //you need to sort this before flying
  //Serial.println(-gyro_yaw_input+ offset);
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;
  
  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;
    
  pid_last_yaw_d_error = pid_error_temp;
}


