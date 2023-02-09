#ifndef HEXAPOD_H
#define HEXAPOD_H
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <stdio.h>
#include <wiringPi.h>
#include <iostream>
#include <eigen3/Eigen/Dense>

// Control table address
#define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_PRESENT_POSITION        36
#define ADDR_MX_MOVING_SPEED            32
#define ADDR_MX_MOVING_STATUS           46
#define DXL_MOVING_STATUS_THRESHOLD     10

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
// ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      0                   // 0 degree
#define DXL_MAXIMUM_POSITION_VALUE      1023                // 300 degree
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b
#define EPS                             7

using namespace std;

extern double deg2rad(double deg);
extern double rad2deg(double rad);
extern uint16_t servo_deg2bit(double deg);
extern double servo_bit2deg(uint16_t bit);
//Convert from degree per second to rpm and then to bit
extern uint16_t servo_dps_rpm_bit(double degs);

class Hexaleg {
public:
    uint8_t& first_Servo;
    uint8_t& second_Servo;
    uint8_t& third_Servo;
    uint8_t& force_sensor;
    Eigen::MatrixXf desired_angle;                      //In degree
    Eigen::MatrixXf desired_velocity;                   //In mm/s
    Eigen::MatrixXf desired_relative_planning_position; //Matrix containing discreted planning position, size is updated when n is set
    Eigen::VectorXf leg_configuration;                      //The configurations of the leg

    Hexaleg(uint8_t& s1, uint8_t& s2, uint8_t& s3, uint8_t fs,const char* leg) : 
        first_Servo{ s1 }, second_Servo{ s2 }, third_Servo{ s3 }, force_sensor{ fs }, name{ leg } ,
        desired_angle(1,1) , desired_velocity(1,1) , desired_relative_planning_position(1,1) , leg_configuration(1)
    {
        //Initialize matrices and vectors
        desired_angle << 0;
        desired_velocity << 0;
        desired_relative_planning_position << 0;
        leg_configuration << 0;
        relative_body_position << 0,0,0;
        rotation_matrix << 0,0,0 ,0,0,0 ,0,0,0;
        relative_planning_position << 0,0,0;
        relative_current_position << 0,0,0;
    }

    Hexaleg(const Hexaleg &L);
    Hexaleg& operator=(const Hexaleg &L);

    Eigen::Matrix3f updateRollPitchYaw(float& roll, float& pitch, float& yaw);
    void matricesSetup(const Eigen::Vector3f& body_position, const Eigen::Matrix3f& rotation, Eigen::VectorXf configuration, int ang3, int ang5);      //Set up and initialize all of matrices
    void checkWorkspace(bool pair, int n);                                                                                                             //check if the desired position is in the workspace. Shift or modify the desired positions based on the pair
    void angleGenerator(bool pair, int n);                                                                                                             //Generate angle based on desired positions, using inverse kinematics
    void planningStepGenerator(const Eigen::Vector3f ang, const Eigen::Vector3f linear, bool pair, int n);                                                  //Generate desired positions with the commands
    bool checkServoStatus(dynamixel::PortHandler* port, dynamixel::PacketHandler* packet, uint8_t servo, uint16_t des_ang);                                    //Check if servo reachs epsilon of desired positions
    bool moveToDesiredPosition(dynamixel::PortHandler* port, dynamixel::PacketHandler* packet, uint8_t servo, uint16_t position);                                                                               //Move servos to their desired position
    bool onGround();                                                                                                                            //If on ground, return angle of 3rd servo. If not, return -1
    void stop(dynamixel::PortHandler* port, dynamixel::PacketHandler* packet);                                                                                                                                //Stop at the current state
    bool isMoving(dynamixel::PortHandler* port, dynamixel::PacketHandler* packet, uint8_t servo);                                      //Check if servo is moving
    void update(dynamixel::PortHandler* port, dynamixel::PacketHandler* packet);                                                                //Update relative_current_position                                                                                                           //Update the relative current position
    void speedSetup(dynamixel::PortHandler* port, dynamixel::PacketHandler* packet);

private:

    Eigen::Vector3f relative_body_position;                 //The relative position of connecting joint
    Eigen::Matrix3f rotation_matrix;                        //the rotation matrix of the connecting joint between leg and body
    Eigen::Vector3f relative_planning_position;             //The relative position of the tip of the Leg, respected to the connencting joint with the body
    Eigen::Vector3f relative_current_position;              //THe relative position of the leg EOF, respected to the joint attaching to the body
    bool phase = false;                             // Detecting the current of phase of swinging or standing. False = 1st half, True = 2nd half
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    const char* name;
    int q3;                                         //The angle existed due to the leg configuration
    int q5;                                         //The angle existed due to the leg configuration
    uint8_t dxl_error = 0;                          // Dynamixel error
    uint16_t dxl_present_position = 0;              // Present position
};

class Hexapair {
public:
    Hexaleg* fLeg;                                                      //First leg
    Hexaleg* sLeg;                                                      //Second leg
    Hexaleg* tLeg;                                                      //Third leg
    bool pStatus{ false };
    Hexapair() {}

    Hexapair(const Hexapair &L);
    Hexapair& operator=(const Hexapair &L);

    void setTripod(Hexaleg& f, Hexaleg& s, Hexaleg& t);                 //Set pairs in order of tripod gait
    void setTetrapod(Hexaleg& f, Hexaleg& s);                           //Set pairs in order of tetrapod gait
    bool checkPairStatus();                                              //Check if all legs in the pair reach their desired position
    void resetPair();                                                   //Unpair legs
    void planningStepGenerator(Eigen::Matrix3f rpy, Eigen::Vector3f linear, bool pair);       //Generate the next step relative position based on current position and command given, True = Swinging & False = Standing
    void movePair(dynamixel::PortHandler* port, dynamixel::PacketHandler* packet);                                                                            //Move all legs in pair until they all reaches their final desired positions
    void onGroundCheck(dynamixel::PortHandler* port, dynamixel::PacketHandler* packet);                                                                       //Check if all legs are on the ground. If not, slowly lower the leg until it touches the ground
    void pairPlanningStepGenerator(Eigen::Vector3f ang, Eigen::Vector3f linear, bool spair, int n);
    void checkPairWorkSpace(bool spair, int n);
    void pairAngleGenerator(bool spair, int n);

private:
    bool tripod = false;                                                     //HIGH when tripod gait is chosen
    bool tetrapod = false;                                                   //HIGH when tetrapod gait is chosen
};

class Hexapod {
public:
    Hexapair firstPair;
    Hexapair secondPair;
    Hexapair thirdPair;
    
    Hexapod(Hexaleg& fR, Hexaleg& sR, Hexaleg& tR, Hexaleg& fL, Hexaleg& sL, Hexaleg& tL) : firstRightLeg{ fR }, secondRightLeg{ sR },
                                                                                            thirdRightLeg{ tR }, firstLeftLeg{ fL },
                                                                                            secondLeftLeg{ sL }, thirdLeftLeg{ tL } {}

    Hexapod(const Hexapod &L);
    Hexapod& operator=(const Hexapod &L);

    void tripodMode();                                                       //Setup for tripod gait perfomance
    void tetrapodMode();                                                     //Setup for tetrapod gait perfomance
    void resetPair();

private:
    Hexaleg& firstRightLeg;
    Hexaleg& secondRightLeg;
    Hexaleg& thirdRightLeg;
    Hexaleg& firstLeftLeg;
    Hexaleg& secondLeftLeg;
    Hexaleg& thirdLeftLeg;

    bool tripod = false;
    bool tetrapod = false;
};

#endif