#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

int getch()
{
    static struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    int c = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return c;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cmd_controler_node");
    ros::NodeHandle nh;
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    geometry_msgs::Twist twist_msg;

    while(ros::ok())
    {
        int key = getch();

        if(key == 'w')
        {
            twist_msg.linear.x = 0.3;
            twist_msg.linear.y = 0.0;
        }
        else if(key == 's')
        {
            twist_msg.linear.x = -0.3;
            twist_msg.linear.y = 0.0;
        }
        else if(key == 'a')
        {
            twist_msg.linear.x = 0.0;
            twist_msg.linear.y = 0.3;
        }
        else if(key == 'd')
        {
            twist_msg.linear.x = 0.0;
            twist_msg.linear.y = -0.3;
        }
        else if(key == 'q')
        {
            twist_msg.linear.x = 0.0;
            twist_msg.linear.y = 0.0;
        }
        else
        {
            continue;  // 忽略其他按键输入
        }

        vel_pub.publish(twist_msg);

        // ros::spinOnce();
    }

    return 0;
}