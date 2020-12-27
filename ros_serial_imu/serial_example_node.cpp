#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>//中间测试用
#include <std_msgs/Empty.h>
//#include <serial_msgs/serial.h>
#include <std_msgs/Int64.h>
#include <iostream>
#include "sensor_msgs/Imu.h"  //SDK 编解码写好后加入！！！
#include "carMsgs/pid.h"

#include "math.h"
#include "string"




#define   cmd_num         6
#define   sBUFFERSIZE     25
//#define   cmd              (0xa515ba)

/****

97
53
49
53
98
97
unsigned char s_buffer[sBUFFERSIZE];
//unsigned char r_buffer[rBUFFERSIZE];
serial::Serial ser;
void chatterCallback(const serial_msgs::serial::ConstPtr& msg)
{
    ROS_INFO_STREAM("Writing to serial port");
    for(int i=0;i<msg->serial.size();++i)
    {
        ROS_INFO("Writing: [0x%02x]", msg->serial[i]);
        s_buffer[i]=msg->serial[i];
    }
    ser.write(s_buffer,sBUFFERSIZE);
}
*******/
serial::Serial ser;
//uint32_t cmd_ = 10819002;

void write_callback(const std_msgs::Int64::ConstPtr& send_cmd){
    ROS_INFO_STREAM("Writing to serial port" << send_cmd->data);
    std::cout<<"111111"<<std::endl;
    ser.write("10819002");
}
//CMD_OUTPUT_ACC
int main (int argc, char** argv){
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;

    ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000);

    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }
    ROS_INFO_STREAM("111111");
    ros::Rate loop_rate(500);
     while(ros::ok()){
         ROS_INFO_STREAM("111111");
       // serial_msgs::serial msg;
        ros::spinOnce();

       // if(ser.available()){
         //   ROS_INFO_STREAM("Reading from serial port");
          //  std_msgs::Float64 result;
          //  result.data = ser.read(ser.available());

            //ser.read(r_buffer,rBUFFERSIZE );
            //for(int i=0;i<rBUFFERSIZE;i++)
            //{
            //    ROS_INFO("[0x%02x]",r_buffer[i]);
            //}
            // ROS_INFO_STREAM("End reading from serial port");
            //for(int l = 0;l < rBUFFERSIZE; l++)
            //{
            //   msg.serial.push_back(r_buffer[l]);
            //}
            //msg_pub.publish(msg);

            //=================================================
            //获取缓冲区内的字节数
            ROS_INFO_STREAM("111111");
           // ser.write("0xa515ba");
            size_t n = ser.available();
            if(n!=0)
            {
                uint8_t buffer[1024];
                //读出数据
                n = ser.read(buffer, n);
                for(int i=0; i<n; i++)
                {
                    //16进制的方式打印到屏幕
                    std::cout << std::hex << (buffer[i] & 0xff) << "_";
                }
                std::cout << std::endl;
                //把数据发送回去
                //ser.write(buffer, n);
            }
            loop_rate.sleep();
            //==================================================

            //ROS_INFO_STREAM("Read: " << result.data);
            //read_pub.publish(result);
      //  }
       // loop_rate.sleep();

    }
    return 0;
}


