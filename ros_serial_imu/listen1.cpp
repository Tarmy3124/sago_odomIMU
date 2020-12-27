#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include "iostream"

void Callback(const sensor_msgs::Imu& imumsg)
{
    //ROS_INFO_STREAM("0000");
    std::cout<<"print Q4 data!"<<std::endl;
    std::cout<<imumsg.orientation.x<<std::endl;
    std::cout<<imumsg.orientation.y<<std::endl;
    std::cout<<imumsg.orientation.z<<std::endl;
    std::cout<<imumsg.orientation.w<<std::endl;

    std::cout<<"print GRY data!"<<std::endl;
    std::cout<<imumsg.angular_velocity.x<<std::endl;
    std::cout<<imumsg.angular_velocity.y<<std::endl;
    std::cout<<imumsg.angular_velocity.z<<std::endl;

    std::cout<<"print ACC data!"<<std::endl;
    std::cout<<imumsg.linear_acceleration.x<<std::endl;
    std::cout<<imumsg.linear_acceleration.y<<std::endl;
    std::cout<<imumsg.linear_acceleration.z<<std::endl;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listen1");
    ros::NodeHandle n;
    //ROS_INFO_STREAM("1111");
    ros::Subscriber sub = n.subscribe("imumsg", 1000, Callback);
    ros::spin();
    return 0;
}
