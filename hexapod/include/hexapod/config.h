#ifndef CONFIG_H
#define CONFIG_H
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include "../include/hexapod/Hexapod_handler.h"
#include "../include/dynamixel_sdk/dynamixel_sdk.h"
#include <Eigen/Dense>
#include <math.h>


// Default setting
extern uint8_t Servo_1;					// Dynamixel ID: 1
extern uint8_t Servo_2;					// Dynamixel ID: 2
extern uint8_t Servo_3;					// Dynamixel ID: 3
extern uint8_t Servo_4;					// Dynamixel ID: 4
extern uint8_t Servo_5;					// Dynamixel ID: 5
extern uint8_t Servo_6;					// Dynamixel ID: 6
extern uint8_t Servo_7;					// Dynamixel ID: 7
extern uint8_t Servo_8;					// Dynamixel ID: 8
extern uint8_t Servo_9;					// Dynamixel ID: 9
extern uint8_t Servo_10;					// Dynamixel ID: 10
extern uint8_t Servo_11;					// Dynamixel ID: 11
extern uint8_t Servo_12;					// Dynamixel ID: 12
extern uint8_t Servo_13;					// Dynamixel ID: 13
extern uint8_t Servo_14;					// Dynamixel ID: 14
extern uint8_t Servo_15;					// Dynamixel ID: 15
extern uint8_t Servo_16;					// Dynamixel ID: 16
extern uint8_t Servo_17;					// Dynamixel ID: 17
extern uint8_t Servo_18;					// Dynamixel ID: 18

//Leg named
extern const char* leg1;
extern const char* leg2;
extern const char* leg3;
extern const char* leg4;
extern const char* leg5;
extern const char* leg6;

//Force Sensor setting
//WiringPi Pin, check on wiringpi.com/pins for more information
extern const uint8_t FS_Leg_1;
extern const uint8_t FS_Leg_2;
extern const uint8_t FS_Leg_3;
extern const uint8_t FS_Leg_4;
extern const uint8_t FS_Leg_5;
extern const uint8_t FS_Leg_6;

//Velocity
extern int8_t lin_vel;
extern int8_t ang_vel;

using namespace std;

//Moving Setting
extern Hexapair* swinging_pair;
extern Hexapair* standing_pair;

//Hexapod Caculation Settup
extern int16_t primitive_rising;                        //In degree
extern const float angle_bit_calibration;          //Calibration to convert angle to 16 bit
extern const float max_time;		//time of swinging and standing phase, in seconds
extern const int n;				//number of discretization, max = n*increment

extern const double LEG_MAX;	//Update it with equation
extern const int leg_ang_3;	//Existed due to the leg configuration
extern const int leg_ang_5;	//Existed due to the leg configuration

#endif 
