#ifndef CONFIG_H
#define CONFIG_H
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include "../include/hexapod/Hexapod_handler.h"
#include "../include/dynamixel_sdk/dynamixel_sdk.h"
#include <eigen3/Eigen/Dense>
#include <math.h>


// Default setting
uint8_t Servo_1 = 1;					// Dynamixel ID: 1
uint8_t Servo_2 = 2;					// Dynamixel ID: 2
uint8_t Servo_3 = 3;					// Dynamixel ID: 3
uint8_t Servo_4 = 4;					// Dynamixel ID: 4
uint8_t Servo_5 = 5;					// Dynamixel ID: 5
uint8_t Servo_6 = 6;					// Dynamixel ID: 6
uint8_t Servo_7 = 7;					// Dynamixel ID: 7
uint8_t Servo_8 = 8;					// Dynamixel ID: 8
uint8_t Servo_9 = 9;					// Dynamixel ID: 9
uint8_t Servo_10 = 10;					// Dynamixel ID: 10
uint8_t Servo_11 = 11;					// Dynamixel ID: 11
uint8_t Servo_12 = 12;					// Dynamixel ID: 12
uint8_t Servo_13 = 13;					// Dynamixel ID: 13
uint8_t Servo_14 = 14;					// Dynamixel ID: 14
uint8_t Servo_15 = 15;					// Dynamixel ID: 15
uint8_t Servo_16 = 16;					// Dynamixel ID: 16
uint8_t Servo_17 = 17;					// Dynamixel ID: 17
uint8_t Servo_18 = 18;					// Dynamixel ID: 18

//Leg named
const char* leg1 = "Leg1";
const char* leg2 = "Leg2";
const char* leg3 = "Leg3";
const char* leg4 = "Leg4";
const char* leg5 = "Leg5";
const char* leg6 = "Leg6";

//Force Sensor setting
//WiringPi Pin, check on wiringpi.com/pins for more information
const uint8_t FS_Leg_1 = 0;
const uint8_t FS_Leg_2 = 1;
const uint8_t FS_Leg_3 = 2;
const uint8_t FS_Leg_4 = 3;
const uint8_t FS_Leg_5 = 4;
const uint8_t FS_Leg_6 = 5;

//Velocity
int8_t lin_vel = 0;
int8_t ang_vel = 0;

using namespace std;

//Moving Setting
Hexapair* swinging_pair = NULL;
Hexapair* standing_pair = NULL;

//Hexapod Caculation Settup
int16_t primitive_rising = 20;                        //In degree
const float angle_bit_calibration = 3.41333;          //Calibration to convert angle to 16 bit
const float max_time = 2;		//time of swinging and standing phase, in seconds
const int n = 10;				//number of discretization, max = n*increment

Eigen::Matrix3f const rot_mat_1((Eigen::Matrix3f() << cos(deg2rad(45)), -sin(deg2rad(45)), 0, sin(deg2rad(45)), cos(deg2rad(45)), 0, 0, 0, 1).finished());          //Rotation Matrix of Leg 1
Eigen::Matrix3f const rot_mat_2((Eigen::Matrix3f() << cos(deg2rad(0)), -sin(deg2rad(0)), 0, sin(deg2rad(0)), cos(deg2rad(0)), 0, 0, 0, 1).finished());          //Rotation Matrix of Leg 2
Eigen::Matrix3f const rot_mat_3((Eigen::Matrix3f() << cos(deg2rad(-45)), -sin(deg2rad(-45)), 0, sin(deg2rad(-45)), cos(deg2rad(-45)), 0, 0, 0, 1).finished());          //Rotation Matrix of Leg 3
Eigen::Matrix3f const rot_mat_4((Eigen::Matrix3f() << cos(deg2rad(135)), -sin(deg2rad(135)), 0, sin(deg2rad(135)), cos(deg2rad(135)), 0, 0, 0, 1).finished());          //Rotation Matrix of Leg 4
Eigen::Matrix3f const rot_mat_5((Eigen::Matrix3f() << cos(deg2rad(180)), -sin(deg2rad(180)), 0, sin(deg2rad(180)), cos(deg2rad(180)), 0, 0, 0, 1).finished());          //Rotation Matrix of Leg 5
Eigen::Matrix3f const rot_mat_6((Eigen::Matrix3f() << cos(deg2rad(225)), -sin(deg2rad(225)), 0, sin(deg2rad(225)), cos(deg2rad(225)), 0, 0, 0, 1).finished());          //Rotation Matrix of Leg 6

//Relative Positions of Leg joints on Body, respected on body relative reference frame
Eigen::Vector3f const relative_body_position_1(75.91, 140.91, 0);                              //Preset this vector
Eigen::Vector3f const relative_body_position_2(118.5, 0, 0);                                   //Preset this vector
Eigen::Vector3f const relative_body_position_3(75.91, -140.91, 0);                              //Preset this vector
Eigen::Vector3f const relative_body_position_4(-75.91, 140.91, 0);                              //Preset this vector
Eigen::Vector3f const relative_body_position_5(-118.5, 0, 0);                                   //Preset this vector
Eigen::Vector3f const relative_body_position_6(-75.91, -140.91, 0);                              //Preset this vector 

//Leg configuration
Eigen::VectorXf leg_config(6);

const double LEG_MAX = 0;	//Update it with equation
const int leg_ang_3 = 30;	//Existed due to the leg configuration
const int leg_ang_5 = 60;	//Existed due to the leg configuration
#endif 
