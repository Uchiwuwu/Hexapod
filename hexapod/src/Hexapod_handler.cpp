#include "Hexapod_handler.h"

//Declare variables and functions
double deg2rad(double deg) {
    return deg * atan2(0, -1) / 180.0;
}
double rad2deg(double rad) {
    return rad * 180.0 / atan2(0, -1);
}
uint16_t servo_deg2bit(float deg)
{
    return (uint16_t)(1023 * deg / 300);
}
float servo_bit2deg(uint16_t bit)
{
    return (float)(300 * bit / 1023);
}
//Convert from degree per second to rpm and then to bit
uint16_t servo_dps_rpm_bit(double degs)
{
    return (uint16_t)(degs / 6 * 1023 / 114);
}

Hexaleg::Hexaleg(const Hexaleg &L): first_Servo(L.first_Servo), second_Servo(L.second_Servo),
                                    third_Servo(L.third_Servo), force_sensor(L.force_sensor),
                                    desired_angle(L.desired_angle),desired_velocity(L.desired_velocity),
                                    desired_relative_planning_position(L.desired_relative_planning_position),leg_configuration(L.leg_configuration),
                                    relative_body_position(L.relative_body_position),relative_current_position(L.relative_current_position),
                                    relative_planning_position(L.relative_planning_position),rotation_matrix(L.rotation_matrix),
                                    phase(L.phase),name(L.name),dxl_comm_result(L.dxl_comm_result),q3(L.q3),q5(L.q5),
                                    dxl_error(L.dxl_error),dxl_present_position(L.dxl_present_position) {}

void Hexaleg::matricesSetup(const Eigen::Vector3f& body_position, const Eigen::Matrix3f& rotation, Eigen::VectorXf configuration, int ang3, int ang5)
{
    relative_body_position = body_position;
    rotation_matrix = rotation;
    leg_configuration.resize(configuration.size());
    leg_configuration = configuration;
    q3 = ang3;
    q5 = ang5;
}

bool Hexaleg::checkServoStatus(dynamixel::PortHandler* port, dynamixel::PacketHandler* packet, uint8_t servo, uint16_t des_ang)
{
    dxl_comm_result = packet->read2ByteTxRx(port, servo, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    { 
        printf("%s\n", packet->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packet->getRxPacketError(dxl_error));
    }

    if (servo == first_Servo)
    {
        return (dxl_present_position == des_ang) ? true : false;
    }
    else if (servo == second_Servo)
    {
        return (dxl_present_position == des_ang) ? true : false;
    }
    else if (servo == third_Servo)
    {
        return (dxl_present_position == des_ang) ? true : false;
    }
    else return false;
}

bool Hexaleg::moveToDesiredPosition(dynamixel::PortHandler* port, dynamixel::PacketHandler* packet, uint8_t servo, uint16_t position)
{
    //Move servo to the desired position, 1 = coxa servo, 2 = femur servo, 3 = tibia
    if (servo == first_Servo)
    {
        dxl_comm_result = packet->write2ByteTxRx(port, servo, ADDR_MX_GOAL_POSITION, position, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s %s %u\n", packet->getTxRxResult(dxl_comm_result), name, servo);
            return false;
        }
        else if (dxl_error != 0)
        {
            printf("%s %s %u\n", packet->getRxPacketError(dxl_error), name, servo);
            return false;
        }
        return true;
    }
    else if (servo == second_Servo) {
        dxl_comm_result = packet->write2ByteTxRx(port, servo, ADDR_MX_GOAL_POSITION, position, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s %s %u\n", packet->getTxRxResult(dxl_comm_result), name, servo);
            return false;
        }
        else if (dxl_error != 0)
        {
            printf("%s %s %u\n", packet->getRxPacketError(dxl_error), name, servo);
            return false;
        }
        return true;
    }
    else if (servo == third_Servo) {
        dxl_comm_result = packet->write2ByteTxRx(port, servo, ADDR_MX_GOAL_POSITION, position, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s %s %u\n", packet->getTxRxResult(dxl_comm_result), name, servo);
            return false;
        }
        else if (dxl_error != 0)
        {
            printf("%s %s %u\n", packet->getRxPacketError(dxl_error), name, servo);
            return false;
        }
        return true;
    }
    else
    {
        printf("Please choose servo from 1 to 3 %s\n", name);
        return false;
    }
}

bool Hexaleg::onGround()
{
    //Read data from FSR. If 1 = touching ground, 0 = not touching ground
    return (digitalRead(force_sensor)? true: false );
}

void Hexaleg::stop(dynamixel::PortHandler* port, dynamixel::PacketHandler* packet)
{
    dxl_comm_result = packet->read2ByteTxRx(port, first_Servo, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
    moveToDesiredPosition(port, packet, first_Servo, dxl_present_position);
    dxl_comm_result = packet->read2ByteTxRx(port, second_Servo, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
    moveToDesiredPosition(port, packet, second_Servo, dxl_present_position);
    dxl_comm_result = packet->read2ByteTxRx(port, third_Servo, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
    moveToDesiredPosition(port, packet, third_Servo, dxl_present_position);
}

void Hexaleg::update(dynamixel::PortHandler* port, dynamixel::PacketHandler* packet)
{
    uint16_t s1,s2,s3;
    dxl_comm_result = packet->read2ByteTxRx(port, first_Servo, ADDR_MX_PRESENT_POSITION, &s1, &dxl_error);
    dxl_comm_result = packet->read2ByteTxRx(port, second_Servo, ADDR_MX_PRESENT_POSITION, &s2, &dxl_error);
    dxl_comm_result = packet->read2ByteTxRx(port, third_Servo, ADDR_MX_PRESENT_POSITION, &s3, &dxl_error);

    relative_current_position(0) = leg_configuration(0) * cos(deg2rad(servo_bit2deg(s1))) + leg_configuration(3) * cos(deg2rad(servo_bit2deg(s1))) * cos(deg2rad(servo_bit2deg(s2))) + leg_configuration(4) 
        * cos(deg2rad(servo_bit2deg(s1))) * cos(deg2rad(servo_bit2deg(s2)) + deg2rad(q3)) + leg_configuration(5) * cos(deg2rad(servo_bit2deg(s1))) * cos(deg2rad(servo_bit2deg(s2)) + deg2rad(servo_bit2deg(s3))) 
        + leg_configuration(6) * cos(deg2rad(servo_bit2deg(s1))) * cos(deg2rad(servo_bit2deg(s2)) + deg2rad(servo_bit2deg(s3)) + deg2rad(q5));
    relative_current_position(1) = leg_configuration(0) * sin(deg2rad(servo_bit2deg(s1))) + leg_configuration(3) * sin(deg2rad(servo_bit2deg(s1))) * cos(deg2rad(servo_bit2deg(s2))) + leg_configuration(4)
        * sin(deg2rad(servo_bit2deg(s1))) * cos(deg2rad(servo_bit2deg(s2)) + deg2rad(q3)) + leg_configuration(5) * sin(deg2rad(servo_bit2deg(s1))) * cos(deg2rad(servo_bit2deg(s2)) + deg2rad(servo_bit2deg(s3)))
        + leg_configuration(6) * sin(deg2rad(servo_bit2deg(s1))) * cos(deg2rad(servo_bit2deg(s2)) + deg2rad(servo_bit2deg(s3)) + deg2rad(q5));
    relative_current_position(1) = - leg_configuration(3) * sin(deg2rad(servo_bit2deg(s2))) - leg_configuration(4) * sin(deg2rad(servo_bit2deg(s2)) + deg2rad(q3)) - leg_configuration(5) 
        * sin(deg2rad(servo_bit2deg(s2)) + deg2rad(servo_bit2deg(s3))) - leg_configuration(6) * sin(deg2rad(servo_bit2deg(s2)) + deg2rad(servo_bit2deg(s3)) + deg2rad(q5));
}

Eigen::Matrix3f Hexaleg::updateRollPitchYaw(float& roll, float& pitch, float& yaw)
{
    Eigen::Matrix3f yaw_pitch_row((Eigen::Matrix3f() << 0, 0, 0, 0, 0, 0, 0, 0, 0).finished());
    yaw_pitch_row << cos(deg2rad(yaw)) * cos(deg2rad(pitch)),
        cos(deg2rad(yaw))* sin(deg2rad(pitch))* sin(deg2rad(roll)) - sin(deg2rad(yaw)) * cos(deg2rad(roll)),
        cos(deg2rad(yaw))* sin(deg2rad(pitch))* cos(deg2rad(roll)) + sin(deg2rad(yaw)) * sin(deg2rad(roll)),
        sin(deg2rad(yaw))* cos(deg2rad(pitch)),
        sin(deg2rad(yaw))* sin(deg2rad(pitch))* sin(deg2rad(roll)) + cos(deg2rad(yaw)) * cos(deg2rad(roll)),
        sin(deg2rad(yaw))* sin(deg2rad(pitch))* cos(deg2rad(roll)) - cos(deg2rad(yaw)) * sin(deg2rad(roll)),
        -sin(deg2rad(pitch)),
        cos(deg2rad(pitch))* sin(deg2rad(roll)),
        cos(deg2rad(pitch))* cos(deg2rad(roll));
    return yaw_pitch_row;
}

void Hexaleg::checkWorkspace(bool pair, int n)
{
    printf("calculate ang %d\n", pair);
    float ang = deg2rad(40) - acos((pow(leg_configuration(2), 2) + pow(leg_configuration(6), 2) - pow(leg_configuration(5), 2)) / (2 * leg_configuration(2) * leg_configuration(6)));
    Eigen::Vector3f LEG_MAX;
    LEG_MAX << leg_configuration(0) + leg_configuration(2) * cos(ang) + sqrt(pow(leg_configuration(1), 2) + pow(sin(ang) - abs(relative_planning_position(2)), 2)), 0, relative_planning_position(2);
    //True = swinging, False = standing
    cout << LEG_MAX << endl;
    printf("Before checking pairs %d\n", pair);
    if (pair)
    {
        //For swinging pair,if it is out of workspace, shift all of desired planning position back inside the workspace
        if (relative_planning_position.norm() > LEG_MAX.norm())
        {
            float a = (relative_planning_position - relative_current_position).squaredNorm();
            float b = 2 * (relative_planning_position - relative_current_position).dot(relative_current_position);
            float c = relative_current_position.squaredNorm();
            float d = (-b + sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);

            Eigen::Vector3f distance_vector = relative_planning_position - relative_current_position;
            if (d < 0 && d >= -1)
            {  
                d = (-b - sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);
            }
            relative_planning_position = d * distance_vector + relative_current_position;
            for (int i = 0; i < desired_relative_planning_position.cols(); i++)
                desired_relative_planning_position.col(i) = desired_relative_planning_position.col(i) - distance_vector;
            //Current position will replace desired positions that passed it
            for (int a = 0; a < n - ceil(d * n) + 1; a++)
                desired_relative_planning_position.col(a) = relative_current_position;
            
             cout << desired_relative_planning_position.cols() << endl;
        }
    }
    else
    {
        //For standing pair, if it is out of workspace, cut down the out-of-bound desired positions
        int i = 0;
        cout << desired_relative_planning_position.col(i) << endl;
        while ((desired_relative_planning_position.col(i).norm() > LEG_MAX.norm()) || (i == desired_relative_planning_position.cols())) i++;
        Eigen::MatrixXf temp = desired_relative_planning_position;
        if(i == 0) printf("checkWorkSpace: i == 0, invalid\n");
        else
        {
            desired_relative_planning_position.resize(3, i);
        }
        for (int j = 0; j < i; j++)
            desired_relative_planning_position.col(j) = temp.col(j);
        cout << desired_relative_planning_position.cols() << endl;
    }
}

//Generate desired angle from all of desired relative planning position, and calculate the velocity needed;
void Hexaleg::angleGenerator(bool pair, int n)
{
    float q1, q2, q3;
    float x, y, z, a;
    float q11, q12, q13;
    cout << this->desired_relative_planning_position.cols() << endl;
    printf("Before resize matrices %d %s\n", pair, name);
    desired_angle.resize(3, this->desired_relative_planning_position.cols() - 1);
    desired_velocity.resize(3, this->desired_relative_planning_position.cols() - 1);
    printf("done resizing %d\n", pair);
    if (pair)
    {
        printf("in %d\n", pair);
        for (int i = 0; i < desired_relative_planning_position.cols(); i++)
        {
            x = desired_relative_planning_position(0, i);
            y = desired_relative_planning_position(1, i);
            z = desired_relative_planning_position(2, i);
            q1 = atan2(y, x);
            a = x / cos(q1) - leg_configuration(0);
            q2 = abs(atan2(z, a)) - acos((pow(leg_configuration(1), 2) + pow(z, 2) + pow(a, 2) - pow(leg_configuration(2), 2)) / (2 * leg_configuration(1) * sqrt(pow(a, 2) + pow(z, 2)))) 
                - acos((pow(leg_configuration(1), 2) + pow(leg_configuration(3), 2) - pow(leg_configuration(4), 2)) / (2 * leg_configuration(1) * leg_configuration(3)))
                - deg2rad(-120 / pow(desired_relative_planning_position.cols(), 2) * pow(i + 1, 2) + 120 / desired_relative_planning_position.cols() * (i + 1));        //Adding upside down parabola to make a upper swinging shape
            q3 = acos((pow(a, 2) + pow(z, 2) - pow(leg_configuration(1), 2) - pow(leg_configuration(2), 2)) / (2 * leg_configuration(1) * leg_configuration(2))) - acos((pow(leg_configuration(5), 2) + pow(leg_configuration(2), 2) 
                - pow(leg_configuration(6), 2)) / (2 * leg_configuration(5) * leg_configuration(2)));
            //Skip the first desired planning position because it is the current position
            if (i != 0)
            {
                desired_angle(0, i - 1) = servo_deg2bit(150 + rad2deg(q1));
                desired_angle(1, i - 1) = servo_deg2bit(150 - rad2deg(q2));
                desired_angle(2, i - 1) = servo_deg2bit(150 - rad2deg(q3));
                desired_velocity(0, i - 1) = servo_dps_rpm_bit(rad2deg(abs(q1 - q11)) / (n / desired_relative_planning_position.cols()));
                desired_velocity(1, i - 1) = servo_dps_rpm_bit(rad2deg(abs(q2 - q12)) / (n / desired_relative_planning_position.cols()));
                desired_velocity(2, i - 1) = servo_dps_rpm_bit(rad2deg(abs(q3 - q13)) / (n / desired_relative_planning_position.cols()));
            }
            q11 = q1;
            q12 = q2;
            q13 = q3;
        }
    }
    else
    {
        printf("in %d\n", pair);
        for (int i = 0; i < desired_relative_planning_position.cols(); i++)
        {
            x = desired_relative_planning_position(0, i);
            y = desired_relative_planning_position(1, i);
            z = desired_relative_planning_position(2, i);
            q1 = atan2(y, x);   
            a = x / cos(q1) - leg_configuration(0);
            q2 = abs(atan2(z, a)) - acos((pow(leg_configuration(1), 2) + pow(z, 2) + pow(a, 2) - pow(leg_configuration(2), 2)) / (2 * leg_configuration(1) * sqrt(pow(a, 2) + pow(z, 2))))
                - acos((pow(leg_configuration(1), 2) + pow(leg_configuration(3), 2) - pow(leg_configuration(4), 2)) / (2 * leg_configuration(1) * leg_configuration(3)));  
            q3 = acos((pow(a, 2) + pow(z, 2) - pow(leg_configuration(1), 2) - pow(leg_configuration(2), 2)) / (2 * leg_configuration(1) * leg_configuration(2))) - acos((pow(leg_configuration(5), 2) + pow(leg_configuration(2), 2)
                - pow(leg_configuration(6), 2)) / (2 * leg_configuration(5) * leg_configuration(2)));
            //Skip the first desired planning position because it is the current position
            if (i != 0)
            {
                desired_angle(0, i - 1) = servo_deg2bit(150 + rad2deg(q1));
                desired_angle(1, i - 1) = servo_deg2bit(150 - rad2deg(q2));
                desired_angle(2, i - 1) = servo_deg2bit(150 - rad2deg(q3));
                desired_velocity(0, i - 1) = servo_dps_rpm_bit(rad2deg(abs(q1 - q11)) / (n / desired_relative_planning_position.cols()));
                desired_velocity(1, i - 1) = servo_dps_rpm_bit(rad2deg(abs(q2 - q12)) / (n / desired_relative_planning_position.cols()));
                desired_velocity(2, i - 1) = servo_dps_rpm_bit(rad2deg(abs(q3 - q13)) / (n / desired_relative_planning_position.cols()));
            }
            q11 = q1;
            q12 = q2;
            q13 = q3;
        }
    }

}

//Generate desired relative planning positions for each leg with linear and angular commands, the number of positions generated based on the number of discretization
void Hexaleg::planningStepGenerator(const Eigen::Vector3f ang, const Eigen::Vector3f linear, bool pair, int n)
{
    Eigen::Vector3f temp_ang;
    Eigen::Matrix3f rpy;
    printf("before resize desired_relative_planning_position %d\n",pair);
    desired_relative_planning_position.resize(3, n);
    //cout << desired_relative_planning_position.cols() << endl;
    printf("Use linear and angular command to calculate the desired relative planning position %d\n",pair);
    if (pair == true)
    {
        for (int i = 0 ; i < n; i++)
        {
            temp_ang = ang * i / n;
            rpy = updateRollPitchYaw(temp_ang(0), temp_ang(1), temp_ang(2));
            desired_relative_planning_position.col(i) = rotation_matrix.transpose() * (linear * i / n + rpy * (relative_body_position + rotation_matrix * relative_current_position) - relative_body_position);
        }
        relative_planning_position = desired_relative_planning_position.col(n-1);
    }
    else
    {
        printf("in false\n");
        for (int i = 0 ; i < n; i++)
        {
            temp_ang = ang * i / n;
            rpy = updateRollPitchYaw(temp_ang(0), temp_ang(1), temp_ang(2));
            desired_relative_planning_position.col(i) = rotation_matrix.transpose() * (rpy.transpose() * (relative_body_position + rotation_matrix * relative_current_position - linear * i / n) - relative_body_position);
        }
        relative_planning_position = desired_relative_planning_position.col(n-1);
    }
    printf("DOne planningStepGenerator %d %s\n",pair, name);
    cout << desired_relative_planning_position.cols() << endl;
}

Hexapair::Hexapair(const Hexapair &L): fLeg(L.fLeg), sLeg(L.sLeg), tLeg(L.tLeg), pStatus(L.pStatus), tripod(L.tripod), tetrapod(L.tetrapod) {}

void Hexapair::setTripod(Hexaleg& f, Hexaleg& s, Hexaleg& t)
{
    //Turn on Tripod mode, turn off Tetrapod mode
    fLeg = &f;
    sLeg = &s;
    tLeg = &t;
    tripod = true;
    tetrapod = false;
}

void Hexapair::setTetrapod(Hexaleg& f, Hexaleg& s)
{
    //Turn on Tetrapod mode, turn off Tripod mode
    fLeg = &f;
    sLeg = &s;
    tripod = false;
    tetrapod = true;
}

void Hexapair::resetPair()
{
    //Reset all of pointer linking with legs;
    fLeg = NULL;
    sLeg = NULL;
    tLeg = NULL;
}

void Hexapair::pairPlanningStepGenerator(Eigen::Vector3f ang, Eigen::Vector3f linear, bool spair, int n)
{
    printf("Before 1st leg planningStepGenerator %d\n", spair);
    fLeg->planningStepGenerator(ang, linear, spair, n);
    sLeg->planningStepGenerator(ang, linear, spair, n);
    tLeg->planningStepGenerator(ang, linear, spair, n);
}
void Hexapair::checkPairWorkSpace(bool spair, int n)
{
    printf("check 1st work space %d\n",spair);
    fLeg->checkWorkspace(spair, n);
    sLeg->checkWorkspace(spair, n);
    tLeg->checkWorkspace(spair, n);
}
void Hexapair::pairAngleGenerator(bool spair, int n)
{
    fLeg->angleGenerator(spair, n);
    sLeg->angleGenerator(spair, n);
    tLeg->angleGenerator(spair, n);
}

void Hexapair::movePair(dynamixel::PortHandler* port, dynamixel::PacketHandler* packet)
{
    //Reason for this code is to have it pseudo-concurrently move three legs due to the lack of thread;
    bool leg1 = false;
    bool leg2 = false;
    bool leg3 = false;
    int col1 = 0;
    int col2 = 0;
    int col3 = 0;
    //A loop of moving legs. Exit when all legs reach their final desired angles.
    while (leg1 && leg2 && leg3)
    {
        fLeg->moveToDesiredPosition(port, packet, fLeg->first_Servo, servo_deg2bit(fLeg->desired_angle(0, col1)));
        fLeg->moveToDesiredPosition(port, packet, fLeg->second_Servo, servo_deg2bit(fLeg->desired_angle(1, col1)));
        fLeg->moveToDesiredPosition(port, packet, fLeg->third_Servo, servo_deg2bit(fLeg->desired_angle(2, col1)));

        sLeg->moveToDesiredPosition(port, packet, sLeg->first_Servo, servo_deg2bit(sLeg->desired_angle(0, col2)));
        sLeg->moveToDesiredPosition(port, packet, sLeg->second_Servo, servo_deg2bit(sLeg->desired_angle(1, col2)));
        sLeg->moveToDesiredPosition(port, packet, sLeg->third_Servo, servo_deg2bit(sLeg->desired_angle(2, col2)));

        tLeg->moveToDesiredPosition(port, packet, tLeg->first_Servo, servo_deg2bit(tLeg->desired_angle(0, col3)));
        tLeg->moveToDesiredPosition(port, packet, tLeg->second_Servo, servo_deg2bit(tLeg->desired_angle(1, col3)));
        tLeg->moveToDesiredPosition(port, packet, tLeg->third_Servo, servo_deg2bit(tLeg->desired_angle(2, col3)));

        if (fLeg->checkServoStatus(port, packet, fLeg->first_Servo, servo_deg2bit(fLeg->desired_angle(0, col1)))
            && fLeg->checkServoStatus(port, packet, fLeg->second_Servo, servo_deg2bit(fLeg->desired_angle(1, col1)))
            && fLeg->checkServoStatus(port, packet, fLeg->third_Servo, servo_deg2bit(fLeg->desired_angle(2, col1))))                                             
        {
            if(col1 < fLeg->desired_angle.cols()) 
                col1++;
            else
                leg1 = true;
        }

        if (sLeg->checkServoStatus(port, packet, sLeg->first_Servo, servo_deg2bit(sLeg->desired_angle(0, col2)))
            && sLeg->checkServoStatus(port, packet, sLeg->second_Servo, servo_deg2bit(sLeg->desired_angle(1, col2)))
            && sLeg->checkServoStatus(port, packet, sLeg->third_Servo, servo_deg2bit(sLeg->desired_angle(2, col2))))
        {
            if (col2 < sLeg->desired_angle.cols())
                col2++;
            else
                leg2 = true;
        }

        if (tLeg->checkServoStatus(port, packet, tLeg->first_Servo, servo_deg2bit(tLeg->desired_angle(0, col3)))
            && tLeg->checkServoStatus(port, packet, tLeg->second_Servo, servo_deg2bit(tLeg->desired_angle(1, col3)))
            && tLeg->checkServoStatus(port, packet, tLeg->third_Servo, servo_deg2bit(tLeg->desired_angle(2, col3))))
        {
            if (col3 < tLeg->desired_angle.cols())
                col3++;
            else
                leg3 = true;
        }
    }
}

void Hexapair::onGroundCheck(dynamixel::PortHandler* port, dynamixel::PacketHandler* packet)
{
    //Lower the leg until it reach the ground
    bool leg1 = fLeg->onGround();
    bool leg2 = sLeg->onGround();
    bool leg3 = tLeg->onGround();
    while (true)
    {
        leg1 = fLeg->onGround();
        leg2 = sLeg->onGround();
        leg3 = tLeg->onGround();
        if (!leg1)
        {
            fLeg->moveToDesiredPosition(port, packet, fLeg->third_Servo, servo_deg2bit(fLeg->desired_angle(2, fLeg->desired_angle.cols() - 1)++));
        }
        else
        {
            fLeg->stop(port, packet);
        }

        if (!leg2)
        {
            sLeg->moveToDesiredPosition(port, packet, sLeg->third_Servo, servo_deg2bit(sLeg->desired_angle(2, sLeg->desired_angle.cols() - 1)++));
        }
        else
        {
            sLeg->stop(port, packet);
        }

        if (!leg3)
        {
            tLeg->moveToDesiredPosition(port, packet, tLeg->third_Servo, servo_deg2bit(tLeg->desired_angle(2, tLeg->desired_angle.cols() - 1)++));
        }
        else
        {
            tLeg->stop(port, packet);
        }

        if (leg1 && leg2 && leg3) break;
    }
}

Hexapod::Hexapod(const Hexapod &L):firstPair(L.firstPair),secondPair(L.secondPair),thirdPair(L.thirdPair),firstRightLeg(L.firstRightLeg),
                                    secondRightLeg(L.secondRightLeg),thirdRightLeg(L.thirdRightLeg),firstLeftLeg(L.firstLeftLeg),
                                    secondLeftLeg(L.secondLeftLeg),thirdLeftLeg(L.thirdLeftLeg) {}

void Hexapod::tripodMode()
{
    firstPair.setTripod(firstRightLeg, thirdRightLeg, secondLeftLeg);
    secondPair.setTripod(secondRightLeg, firstLeftLeg, thirdLeftLeg);
    tripod = true;
    tetrapod = false;
}

void Hexapod::tetrapodMode()
{
    firstPair.setTetrapod(thirdRightLeg, secondLeftLeg);
    secondPair.setTetrapod(secondRightLeg, firstLeftLeg);
    thirdPair.setTetrapod(firstRightLeg, thirdLeftLeg);
    tetrapod = true;
    tripod = false;
}

void Hexapod::resetPair()
{
    firstPair.resetPair();
    secondPair.resetPair();
    thirdPair.resetPair();
}