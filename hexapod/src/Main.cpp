#include "config.h"
#include <wiringPi.h>
#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <thread>

// Setup Legs and Hexapod
Hexaleg Leg_1(Servo_1, Servo_2, Servo_3, FS_Leg_1, leg1);
Hexaleg Leg_2(Servo_4, Servo_5, Servo_6, FS_Leg_2, leg2);
Hexaleg Leg_3(Servo_7, Servo_8, Servo_9, FS_Leg_3, leg3);
Hexaleg Leg_4(Servo_10, Servo_11, Servo_12, FS_Leg_4, leg4);
Hexaleg Leg_5(Servo_13, Servo_14, Servo_15, FS_Leg_5, leg5);
Hexaleg Leg_6(Servo_16, Servo_17, Servo_18, FS_Leg_6, leg6);

Hexapod hexbot(Leg_1, Leg_2, Leg_3, Leg_4, Leg_5, Leg_6);


Eigen::Matrix3f rot_mat_1;          //Rotation Matrix of Leg 1
Eigen::Matrix3f rot_mat_2;          //Rotation Matrix of Leg 2
Eigen::Matrix3f rot_mat_3;          //Rotation Matrix of Leg 3
Eigen::Matrix3f rot_mat_4;          //Rotation Matrix of Leg 4
Eigen::Matrix3f rot_mat_5;          //Rotation Matrix of Leg 5
Eigen::Matrix3f rot_mat_6;          //Rotation Matrix of Leg 6

//Relative Positions of Leg joints on Body, respected on body relative reference frame
Eigen::Vector3f relative_body_position_1;                              //Preset this vector
Eigen::Vector3f relative_body_position_2;                                   //Preset this vector
Eigen::Vector3f relative_body_position_3;                              //Preset this vector
Eigen::Vector3f relative_body_position_4;                              //Preset this vector
Eigen::Vector3f relative_body_position_5;                                   //Preset this vector
Eigen::Vector3f relative_body_position_6;                              //Preset this vector 

//Leg configuration
Eigen::VectorXf leg_config(7);

dynamixel::PortHandler* portHandler;
dynamixel::PacketHandler* packetHandler;

void mat_init();
void portSetup();
void Setup();
bool legOnGround();
void trajectoryPlanning(Hexapair* pair, Eigen::Vector3f ang, Eigen::Vector3f linear, bool spair, int n);
void moveLeg(Hexapair* pair);
void readCommand(const geometry_msgs::Twist::ConstPtr& vel_msg);

int main(int argc, char* argv[])
{
	//Initialize hexapod configuration matrices
	mat_init();

	// Initialize PortHandler instance
	// Set the port path
	// Get methods and members of PortHandlerLinux or PortHandlerWindows
	portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

	// Initialize PacketHandler instance
	// Set the protocol version
	// Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
	packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
	//Set up port connecting with Dynamixel
	portSetup();
	//Set up Hexapod before starting operation
	Setup();

	ros::init(argc, argv, "hexbot");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe<geometry_msgs::Twist>("keyboard_control", 1, readCommand);
	ros::spin();
	return 0;
}

void mat_init()
{
	rot_mat_1 << cos(deg2rad(45)), -sin(deg2rad(45)), 0, sin(deg2rad(45)), cos(deg2rad(45)), 0, 0, 0, 1;          //Rotation Matrix of Leg 1
	rot_mat_2 << cos(deg2rad(0)), -sin(deg2rad(0)), 0, sin(deg2rad(0)), cos(deg2rad(0)), 0, 0, 0, 1;          //Rotation Matrix of Leg 2
	rot_mat_3 << cos(deg2rad(-45)), -sin(deg2rad(-45)), 0, sin(deg2rad(-45)), cos(deg2rad(-45)), 0, 0, 0, 1;          //Rotation Matrix of Leg 3
	rot_mat_4 << cos(deg2rad(135)), -sin(deg2rad(135)), 0, sin(deg2rad(135)), cos(deg2rad(135)), 0, 0, 0, 1;          //Rotation Matrix of Leg 4
	rot_mat_5 << cos(deg2rad(180)), -sin(deg2rad(180)), 0, sin(deg2rad(180)), cos(deg2rad(180)), 0, 0, 0, 1;          //Rotation Matrix of Leg 5
	rot_mat_6 << cos(deg2rad(225)), -sin(deg2rad(225)), 0, sin(deg2rad(225)), cos(deg2rad(225)), 0, 0, 0, 1;          //Rotation Matrix of Leg 6

	//Relative Positions of Leg joints on Body, respected on body relative reference frame
	relative_body_position_1 << 75.91, 140.91, 0;                              //Preset this vector
	relative_body_position_2 << 118.5, 0, 0;                                   //Preset this vector
	relative_body_position_3 << 75.91, -140.91, 0;                              //Preset this vector
	relative_body_position_4 << -75.91, 140.91, 0;                              //Preset this vector
	relative_body_position_5 << -118.5, 0, 0;                                   //Preset this vector
	relative_body_position_6 << -75.91, -140.91, 0;                              //Preset this vector 

	//Leg configuration
	leg_config <<  52, 66.11, 160.77, 39.39, 29, 44.05, 134.17;
}

void portSetup()
{
	uint8_t error = 0;                          // Dynamixel error
	int comm_result = COMM_TX_FAIL;

	// Open port
	if (!portHandler->openPort()) {
		ROS_ERROR("Failed to open the port!");
	}
	//Set BaudRate
	if (!portHandler->setBaudRate(BAUDRATE)) {
		ROS_ERROR("Failed to set the baudrate!");
	}

	// Enable Dynamixel Torque
	comm_result = packetHandler->write1ByteTxRx(portHandler, Servo_1, ADDR_MX_TORQUE_ENABLE, 1, &error);
	if (comm_result != COMM_SUCCESS) {
		ROS_ERROR("Failed to enable torque for Dynamixel ID %d", Servo_1);
	}
	comm_result = packetHandler->write1ByteTxRx(portHandler, Servo_2, ADDR_MX_TORQUE_ENABLE, 1, &error);
	if (comm_result != COMM_SUCCESS) {
		ROS_ERROR("Failed to enable torque for Dynamixel ID %d", Servo_2);
	}
	comm_result = packetHandler->write1ByteTxRx(portHandler, Servo_3, ADDR_MX_TORQUE_ENABLE, 1, &error);
	if (comm_result != COMM_SUCCESS) {
		ROS_ERROR("Failed to enable torque for Dynamixel ID %d", Servo_3);
	}
	comm_result = packetHandler->write1ByteTxRx(portHandler, Servo_4, ADDR_MX_TORQUE_ENABLE, 1, &error);
	if (comm_result != COMM_SUCCESS) {
		ROS_ERROR("Failed to enable torque for Dynamixel ID %d", Servo_4);
	}
	comm_result = packetHandler->write1ByteTxRx(portHandler, Servo_5, ADDR_MX_TORQUE_ENABLE, 1, &error);
	if (comm_result != COMM_SUCCESS) {
		ROS_ERROR("Failed to enable torque for Dynamixel ID %d", Servo_5);
	}
	comm_result = packetHandler->write1ByteTxRx(portHandler, Servo_6, ADDR_MX_TORQUE_ENABLE, 1, &error);
	if (comm_result != COMM_SUCCESS) {
		ROS_ERROR("Failed to enable torque for Dynamixel ID %d", Servo_6);
	}
	comm_result = packetHandler->write1ByteTxRx(portHandler, Servo_7, ADDR_MX_TORQUE_ENABLE, 1, &error);
	if (comm_result != COMM_SUCCESS) {
		ROS_ERROR("Failed to enable torque for Dynamixel ID %d", Servo_7);
	}
	comm_result = packetHandler->write1ByteTxRx(portHandler, Servo_8, ADDR_MX_TORQUE_ENABLE, 1, &error);
	if (comm_result != COMM_SUCCESS) {
		ROS_ERROR("Failed to enable torque for Dynamixel ID %d", Servo_8);
	}
	comm_result = packetHandler->write1ByteTxRx(portHandler, Servo_9, ADDR_MX_TORQUE_ENABLE, 1, &error);
	if (comm_result != COMM_SUCCESS) {
		ROS_ERROR("Failed to enable torque for Dynamixel ID %d", Servo_9);
	}
	comm_result = packetHandler->write1ByteTxRx(portHandler, Servo_10, ADDR_MX_TORQUE_ENABLE, 1, &error);
	if (comm_result != COMM_SUCCESS) {
		ROS_ERROR("Failed to enable torque for Dynamixel ID %d", Servo_10);
	}
	comm_result = packetHandler->write1ByteTxRx(portHandler, Servo_11, ADDR_MX_TORQUE_ENABLE, 1, &error);
	if (comm_result != COMM_SUCCESS) {
		ROS_ERROR("Failed to enable torque for Dynamixel ID %d", Servo_11);
	}
	comm_result = packetHandler->write1ByteTxRx(portHandler, Servo_12, ADDR_MX_TORQUE_ENABLE, 1, &error);
	if (comm_result != COMM_SUCCESS) {
		ROS_ERROR("Failed to enable torque for Dynamixel ID %d", Servo_12);
	}
	comm_result = packetHandler->write1ByteTxRx(portHandler, Servo_13, ADDR_MX_TORQUE_ENABLE, 1, &error);
	if (comm_result != COMM_SUCCESS) {
		ROS_ERROR("Failed to enable torque for Dynamixel ID %d", Servo_13);
	}
	comm_result = packetHandler->write1ByteTxRx(portHandler, Servo_14, ADDR_MX_TORQUE_ENABLE, 1, &error);
	if (comm_result != COMM_SUCCESS) {
		ROS_ERROR("Failed to enable torque for Dynamixel ID %d", Servo_14);
	}
	comm_result = packetHandler->write1ByteTxRx(portHandler, Servo_15, ADDR_MX_TORQUE_ENABLE, 1, &error);
	if (comm_result != COMM_SUCCESS) {
		ROS_ERROR("Failed to enable torque for Dynamixel ID %d", Servo_15);
	}
	comm_result = packetHandler->write1ByteTxRx(portHandler, Servo_16, ADDR_MX_TORQUE_ENABLE, 1, &error);
	if (comm_result != COMM_SUCCESS) {
		ROS_ERROR("Failed to enable torque for Dynamixel ID %d", Servo_16);
	}
	comm_result = packetHandler->write1ByteTxRx(portHandler, Servo_17, ADDR_MX_TORQUE_ENABLE, 1, &error);
	if (comm_result != COMM_SUCCESS) {
		ROS_ERROR("Failed to enable torque for Dynamixel ID %d", Servo_17);
	}
	comm_result = packetHandler->write1ByteTxRx(portHandler, Servo_18, ADDR_MX_TORQUE_ENABLE, 1, &error);
	if (comm_result != COMM_SUCCESS) {
		ROS_ERROR("Failed to enable torque for Dynamixel ID %d", Servo_18);
	}
}

void Setup()
{
	//printf("In setup\n");
	wiringPiSetup();
	//Setting FSR GPIOs
	pinMode(FS_Leg_1, INPUT);
	pinMode(FS_Leg_2, INPUT);
	pinMode(FS_Leg_3, INPUT);
	pinMode(FS_Leg_4, INPUT);
	pinMode(FS_Leg_5, INPUT);
	pinMode(FS_Leg_6, INPUT);

	//printf("Before matricesSetup\n");
	// Initial Anngle for each servos, 0 = 0 degree, 1023 = 300 degree
	Leg_1.matricesSetup(relative_body_position_1, rot_mat_1, leg_config, leg_ang_3, leg_ang_5);
	Leg_2.matricesSetup(relative_body_position_2, rot_mat_2, leg_config, leg_ang_3, leg_ang_5);
	Leg_3.matricesSetup(relative_body_position_3, rot_mat_3, leg_config, leg_ang_3, leg_ang_5);
	Leg_4.matricesSetup(relative_body_position_4, rot_mat_4, leg_config, leg_ang_3, leg_ang_5);
	Leg_5.matricesSetup(relative_body_position_5, rot_mat_5, leg_config, leg_ang_3, leg_ang_5);
	Leg_6.matricesSetup(relative_body_position_6, rot_mat_6, leg_config, leg_ang_3, leg_ang_5);

	//printf("Before initialize the leg\n");
	//Servo rotate from 0 to 300, 150 is in middle
	//Initalize the leg
	//Leg 1
	Leg_1.moveToDesiredPosition(portHandler, packetHandler, Leg_1.first_Servo, servo_deg2bit(150));
	Leg_1.moveToDesiredPosition(portHandler, packetHandler, Leg_1.second_Servo, servo_deg2bit(150 + leg_ang_3));
	Leg_1.moveToDesiredPosition(portHandler, packetHandler, Leg_1.third_Servo, servo_deg2bit(150 + leg_ang_5));
	//Leg 2
	Leg_2.moveToDesiredPosition(portHandler, packetHandler, Leg_2.first_Servo, servo_deg2bit(150));
	Leg_2.moveToDesiredPosition(portHandler, packetHandler, Leg_2.second_Servo, servo_deg2bit(150 + leg_ang_3));
	Leg_2.moveToDesiredPosition(portHandler, packetHandler, Leg_2.third_Servo, servo_deg2bit(150 + leg_ang_5));
	//Leg 3
	Leg_3.moveToDesiredPosition(portHandler, packetHandler, Leg_3.first_Servo, servo_deg2bit(150));
	Leg_3.moveToDesiredPosition(portHandler, packetHandler, Leg_3.second_Servo, servo_deg2bit(150 + leg_ang_3));
	Leg_3.moveToDesiredPosition(portHandler, packetHandler, Leg_3.third_Servo, servo_deg2bit(150 + leg_ang_5));
	//Leg 4
	Leg_4.moveToDesiredPosition(portHandler, packetHandler, Leg_4.first_Servo, servo_deg2bit(150));
	Leg_4.moveToDesiredPosition(portHandler, packetHandler, Leg_4.second_Servo, servo_deg2bit(150 + leg_ang_3));
	Leg_4.moveToDesiredPosition(portHandler, packetHandler, Leg_4.third_Servo, servo_deg2bit(150 + leg_ang_5));
	//Leg 5
	Leg_5.moveToDesiredPosition(portHandler, packetHandler, Leg_5.first_Servo, servo_deg2bit(150));
	Leg_5.moveToDesiredPosition(portHandler, packetHandler, Leg_5.second_Servo, servo_deg2bit(150 + leg_ang_3));
	Leg_5.moveToDesiredPosition(portHandler, packetHandler, Leg_5.third_Servo, servo_deg2bit(150 + leg_ang_5));
	//Leg 6
	Leg_6.moveToDesiredPosition(portHandler, packetHandler, Leg_6.first_Servo, servo_deg2bit(150));
	Leg_6.moveToDesiredPosition(portHandler, packetHandler, Leg_6.second_Servo, servo_deg2bit(150 + leg_ang_3));
	Leg_6.moveToDesiredPosition(portHandler, packetHandler, Leg_6.third_Servo, servo_deg2bit(150 + leg_ang_5));

	//printf("update current positions\n");
	//Update the relative current position
	Leg_1.update(portHandler, packetHandler);
	Leg_2.update(portHandler, packetHandler);
	Leg_3.update(portHandler, packetHandler);
	Leg_4.update(portHandler, packetHandler);
	Leg_5.update(portHandler, packetHandler);
	Leg_6.update(portHandler, packetHandler);

	//Set up pair mode
	//printf("setup tripod mode\n");
	hexbot.tripodMode();
	//printf("DOne setting mode\n");
}

bool legOnGround()
{
	//Check if all legs are on ground
	return (Leg_1.onGround() && Leg_2.onGround() && Leg_3.onGround() &&
		Leg_4.onGround() && Leg_5.onGround() && Leg_6.onGround()) ? true : false;
}

void trajectoryPlanning(Hexapair* pair, Eigen::Vector3f ang, Eigen::Vector3f linear, bool spair, int n)
{
	printf("before pairPlanningStepGenerator\n");
	//Getting new planning position commanded
	pair->pairPlanningStepGenerator(ang, linear, spair, n);
	//Check whether the new positions are in the workspace. If not, caclculate the new feasible planning position
	printf("before checkPairWorkSpace\n");
	pair->checkPairWorkSpace(spair, n);
	//Get discreted positions planned for legs, then converts it to angles
	//Calculate servo angles for legs
	printf("before pairAngleGenerator\n");
	pair->pairAngleGenerator(spair, n);
}

void moveLeg(Hexapair* pair)
{
	//Move pairs to its desired positions
	pair->movePair(portHandler, packetHandler);
	//Check if all the legs are on the ground. If not, make it to the ground
	pair->onGroundCheck(portHandler, packetHandler);
	//Update current position
	pair->fLeg->update(portHandler, packetHandler);
	pair->sLeg->update(portHandler, packetHandler);
	pair->tLeg->update(portHandler, packetHandler);
}

void readCommand(const geometry_msgs::Twist::ConstPtr& vel_msg)
{
	printf("In readCommand\n");
	Eigen::Vector3f translation_command(0, 0, 0);
	Eigen::Vector3f rotation_command(0, 0, 0);

	//Read the command
	translation_command(0) = vel_msg->linear.x;
	translation_command(1) = vel_msg->linear.y;
	translation_command(2) = vel_msg->linear.z;
	rotation_command(0) = vel_msg->angular.x;
	rotation_command(1) = vel_msg->angular.y;
	rotation_command(2) = vel_msg->angular.z;

	if (swinging_pair == NULL || standing_pair == NULL)
	{
		printf("setup swinging_pair and standing_pair\n");
		swinging_pair = &hexbot.firstPair;
		standing_pair = &hexbot.secondPair;
		
		printf("set TrajectoryPlanning\n");
		//Create desired trajectory for each legs
		thread th1(trajectoryPlanning, swinging_pair, rotation_command, translation_command, true, n);
		thread th2(trajectoryPlanning, standing_pair, rotation_command, translation_command, false, n);

		th1.join();
		th2.join();

		printf("Move leg\n");
		//Move leg along the trajectories
		thread th3(moveLeg, swinging_pair);
		thread th4(moveLeg, standing_pair);

		th3.join();
		th4.join();
	}
	else
	{
		printf("check if all legs on ground\n");
		//Double check if all of the legs are on the ground
		if (!legOnGround())
		{
			swinging_pair->onGroundCheck(portHandler, packetHandler);
			standing_pair->onGroundCheck(portHandler, packetHandler);
		}

		printf("swap pairs\n");
		//Swap pairs of legs assigned
		swap(swinging_pair, standing_pair);

		printf("set TrajectoryPlanning\n");
		//Create desired trajectory for each legs
		thread th1(trajectoryPlanning, swinging_pair, rotation_command, translation_command, true, n);
		thread th2(trajectoryPlanning, standing_pair, rotation_command, translation_command, false, n);

		th1.join();
		th2.join();

		printf("Move leg\n");
		//Move leg along the trajectories
		thread th3(moveLeg, swinging_pair);
		thread th4(moveLeg, standing_pair);

		th3.join();
		th4.join();
	}
}
