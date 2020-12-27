#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
 #include "serial_msgs/serial.h"
#include <serial/serial.h>


#define rBUFFERSIZE     25
//unsigned char send_buffer[] = {97,53,49,53,98,97};
unsigned char send_buffer[] = {165,21,186};
static int counter = 0;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    //发布主题command
    ros::Publisher command_pub = n.advertise<serial_msgs::serial>("command", 1000);
    ros::Rate loop_rate(50);
    
    int count = 0;
    while (ros::ok())
    {
        serial_msgs::serial msg;
        ros::spinOnce();
        ////==============
        for(int i=0;i < 3;i++)
        {
            ROS_INFO("[0x%02x]",send_buffer[i]);
        }
        ROS_INFO_STREAM("commd topic");
        for(int l=0;l<3;l++)
        {
            msg.serial.push_back(send_buffer[l]);
        }
        command_pub.publish(msg);
        ////==============
        loop_rate.sleep();
    }
    return 0;
}
