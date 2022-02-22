#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include <map>
#include <vector>

// Map for movement keys
std::map<char, std::vector<float>> moveBindings
{
  //In order of x,y,z,roll,pitch,yaw
  {'d', {1, 0, 0, 0 , 0 , 0}},
  {'a', {-1, 0, 0, 0 , 0 , 0}},
  {'w', {0, 1, 0, 0 , 0 , 0}},
  {'s', {0, -1, 0, 0 , 0 , 0}},
  {'q', {0, 0, 1, 0 , 0 , 0}},
  {'e', {0, 0, -1, 0 , 0 , 0}},
  {'S', {0, 0, 0, 1 , 0 , 0}},
  {'W', {0, 0, 0, -1 , 0 , 0}},
  {'D', {0, 0, 0, 0 , 1 , 0}},
  {'A', {0, 0, 0, 0 , -1 , 0}},
  {'Q', {0, 0, 0, 0 , 0 , 1}},
  {'E', {0, 0, 0, 0 , 0 , -1}}
};

// Map for speed keys
std::map<char, std::vector<float>> speedBindings
{
  {'q', {1.1, 1.1}},
  {'z', {0.9, 0.9}},
  {'w', {1.1, 1}},
  {'x', {0.9, 1}},
  {'e', {1, 1.1}},
  {'c', {1, 0.9}}
};

// Reminder message
const char* msg = R"(
Reading from the keyboard and Publishing to Twist!
---------------------------
w : Forward
s : Backward
a : Left
d : Right
q : Up
e : Down
---------------------------
W : Backward roll
S : Forward roll
A : Left pitch
D : Right pitch
Q : CCW yaw
E : CW yaw

CTRL-C to quit
)";

// Init variables
float lin_param(30);                                    // Linear parameter (mm)
float ang_param(5);                                     // Angular parameter (deg)
float x(0), y(0), z(0), roll(0), pitch(0), yaw(0); 
char key(' ');

// For non-blocking keyboard inputs
int getch(void)
{
    int ch;
    struct termios oldt;
    struct termios newt;

    // Store old settings, and copy to new settings
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // Make required changes and apply the settings
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_iflag |= IGNBRK;
    newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
    newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
    newt.c_cc[VMIN] = 1;
    newt.c_cc[VTIME] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &newt);

    // Get the current character
    ch = getchar();

    // Reapply old settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return ch;
}

int main(int argc, char** argv)
{
    // Init ROS node
    ros::init(argc, argv, "hexapod_control");
    ros::NodeHandle nh;

    // Init cmd_vel publisher
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("keyboard_control", 1);

    // Create Twist message
    geometry_msgs::Twist twist;

    printf("%s", msg);
    printf("\rCurrent: linear parameter %f\tangular parameter %f | Awaiting command...\r", lin_param, ang_param);

    while (true) {

        // Get the pressed key
        key = getch();

        // If the key corresponds to a key in moveBindings
        if (moveBindings.count(key) == 1)
        {
            // Grab the direction data
            x = moveBindings[key][0];
            y = moveBindings[key][1];
            z = moveBindings[key][2];
            roll = moveBindings[key][3];
            pitch = moveBindings[key][4];
            yaw = moveBindings[key][5];

            printf("\rCurrent: linear parameter %f\tangular parameter %f | Awaiting command...\r", lin_param, ang_param);
        }
        else
        {
            x = 0;
            y = 0;
            z = 0;
            roll = 0;
            pitch = 0;
            yaw = 0;

            // If ctrl-C (^C) was pressed, terminate the program
            if (key == '\x03')
            {
                printf("\n\n                 .     .\n              .  |\\-^-/|  .    \n             /| } O.=.O { |\\\n\n                 CH3EERS\n\n");
                break;
            }

            printf("\rCurrent: linear parameter %f\tangular parameter %f | Invalid command! %c", lin_param, ang_param, key);
        }

        // Update the Twist message
        twist.linear.x = x * lin_param;
        twist.linear.y = y * lin_param;
        twist.linear.z = z * lin_param;

        twist.angular.x = roll * ang_param;
        twist.angular.y = pitch * ang_param;
        twist.angular.z = yaw * ang_param;

        // Publish it and resolve any remaining callbacks
        pub.publish(twist);
        ros::spinOnce();
    }

    return 0;
}