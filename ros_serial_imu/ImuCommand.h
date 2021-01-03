/************************************************************************************************************
 *  Copyright Notice
 *  Copyright (c) 2018,冯
 *
 *  @File    : ImuCommand.h
 *  @Brief   : IMu指令
 *
 *  @Version : V1.0
 *  @Date    : 2018/11/23

 *	@History :
 *    Author :
 *    Descrip: 命令字节，由外部控制器发送至GY953，帧头0xa5,指令格式：帧头+指令+校验和
 *************************************************************************************************************/

#ifndef IMUCOMMAND_H
#define IMUCOMMAND_H

#include <iostream>
//#include "serial/serial.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "odom_serial/serial.h"
#include <sensor_msgs/Imu.h>
#include "carMsgs/pid.h"

//for odom
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

//boost serial
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#define CMD_FrameHead								   165		   //指令帧头
#define READ_BUFFERSIZE                                 13         //读取数据数组长度
#define cmd_num                                          3         //指令数组长度
#define PIDCHAR_NUM                                     13         //pid串口字符缓冲数量
#define DATA_LENGTH                                      3         //解码后数组长度
#define Q4DATA_LENGTH                                    4         //解码后四元数数组长度
#define mulitCmd_num                                     9         //复合指令数组长度
#define EULAR_MEASURE                                100.0         //欧拉角度量，原始数据除以之
#define Q4_MEASURE                                  1000.0         //四元数度量，原始数据除以之


//sago Odom
#define DATA_TYPE_ODOM_Vx                   (unsigned char)(40)   //odom vx
#define DATA_TYPE_ODOM_Vy                   (unsigned char)(41)   //odom vy
#define DATA_TYPE_ODOM_Vz                   (unsigned char)(42)   //odom vz


//判断本帧数据类型
#define DATA_FrameHead                   (unsigned char)(90)       //接收数据的帧头
#define DATA_TYPE_ACC                    (unsigned char)(21)       //该帧输出数据为加速度类型
#define DATA_TYPE_GRY                    (unsigned char)(37)       //该帧输出数据为陀螺仪类型
#define DATA_TYPE_MAG                    (unsigned char)(53)       //该帧输出数据为磁力计类型
#define DATA_TYPE_EULAR                  (unsigned char)(69)       //该帧输出数据为欧拉角类型
#define DATA_TYPE_STOP                   (unsigned char)(85)       //该帧输出数据为保留不用类型
#define DATA_TYPE_Q4                     (unsigned char)(101)      //该帧输出数据为四元数类型
#define DATA_TYPE_PRECISION              (unsigned char)(117)      //该帧输出数据为传感器精度，频率类型
#define DATA_TYPE_RANGE                  (unsigned char)(133)      //该帧输出数据为传感器量程




//串口波特率设置指令，每次修改后需要重新上电生效，具有掉电保持特性
#define CMD_SET_BAUD115200               (unsigned char)(175)      //115200(默认)
#define CMD_SET_BAUD9600                 (unsigned char)(174)      //9600

//传感器配置指令
#define  CMD_CLOSE_JIAJI                 (unsigned char)(81)     //关闭加计
#define  CMD_CLOSE_GYR                   (unsigned char)(82)     //关闭陀螺
#define  CMD_CLOSE_MAG                   (unsigned char)(83)     //关闭磁场
#define  CMD_GYR_CALIBRATE               (unsigned char)(87)     //加陀校准
#define  CMD_MAG_CALIBRATE               (unsigned char)(88)     //磁场校准
#define  CMD_RES_FACTORY                 (unsigned char)(89)     //恢复出厂设置，即清除保存的校准数据
#define  CMD_OUTPUT_50HZ                 (unsigned char)(164)    //模块输出频率50hz设置
#define  CMD_OUTPUT_100HZ                (unsigned char)(165)    //模块输出频率100hz设置
#define  CMD_OUTPUT_200HZ                (unsigned char)(166)    //模块输出频率200hz设置

//自动输出指令（具有开关功能，第一次发送打开，第二次发送关闭）
#define  CMD_OUTPUT_ACC                  (unsigned char)(21)     //输出加速度原始数据指令
#define  CMD_OUTPUT_GYR                  (unsigned char)(37)     //输出陀螺仪原始数据
#define  CMD_OUTPUT_MAG                  (unsigned char)(53)     //输出磁场原始数据指令
#define  CMD_OUTPUT_EULAR                (unsigned char)(69)     //输出欧拉角指令（默认50HZ）
#define  CMD_OUTPUT_CHAREULAR            (unsigned char)(85)     //输出欧拉角字符形式（串口助手）
#define  CMD_OUTPUT_Q4                   (unsigned char)(101)    //输出四元数数据指令

//查询输出指令
#define  CMD_READ_CALIBRATE_PRECISION    (unsigned char)(117)     //读取三传感器校准精度，输出频率
#define  CMD_READ_RANGE                  (unsigned char)(133)     //读取传感器量程，获取陀螺量程，加计量程，磁场量程
#define  CMD_OPENCV_ALLSENSOR            (unsigned char)(80)      //开启所有传感器
#define  CMD_PULL_OUTPUT_EULAR           (unsigned char)(149)     //输出欧拉角指令（默认50HZ）
#define  CMD_PULL_OUTPUT_Q4              (unsigned char)(181)     //输出四元数数据指令
#define  CMD_PULL_OUTPUT_ACC             (unsigned char)(197)     //输出加速度原始数据指令
#define  CMD_PULL_OUTPUT_GYR             (unsigned char)(213)     //输出陀螺仪原始数据
#define  CMD_PULL_OUTPUT_MAG             (unsigned char)(229)     //输出磁场原始数据指令


extern ros::Publisher msg_pub;
extern ros::Subscriber pid_sub;
//extern ros::Publisher odom_pub; 
//extern tf::TransformBroadcaster odom_broadcaster;
typedef boost::shared_ptr<boost::asio::serial_port> serialp_ptr;
namespace IPSG
{   
    
	class CImuCommand
	{
        
        //serial::Serial ser;
		sensor_msgs::Imu imu_data;
	public:

		CImuCommand(){};

		~CImuCommand();
               void float2char(float f,unsigned char *s);
               void char2float(float *f,unsigned char *s);
        

		/***********************************************
		 *  @Name    : CImuCommand::cmdFrame
		 *  @Descrip : 向IMU发送一帧指令
         *  @in      : 指令
         *  @out     : 原始数据
		 *  @Return  : bool true:success false:failed
		 *  @Notes   : None
		 ***********************************************/
		bool cmdFrame(unsigned char imucmd);
		/***********************************************
		 *  @Name    : CImuCommand::muliteCmdFrame
		 *  @Descrip : 向IMU发送三个指令，查询输出
		 *  @Para    : [in]三个指令
		 *	@Para    : [in/out]	v_ucData：原始数据帧
		 *  @Return  : bool true:success false:failed
		 *  @Notes   : None
		 ***********************************************/
		bool muliteCmdFrame(unsigned char imucmd1,unsigned char imucmd2,unsigned char imucmd3);

		/***********************************************
		 *  @Name    : CImuCommand::decodeFrame
		 *  @Descrip : 解码
		 *  @Notes   : None
		 ***********************************************/
               bool decodeFrame(unsigned char tmpBuffer[READ_BUFFERSIZE]);

		/***********************************************
		 *  @Name    : CImuCommand::display_Imumsg
		 *  @Descrip : 打印imu.msg消息类型
		 *  @para    : [in]sensor::Imu 数据类型
		 *  @Notes   : None
		 ***********************************************/
		bool display_Imumsg(sensor_msgs::Imu imumsg);

		/***********************************************
		 *  @Name    : CImuCommand::serialInit
		 *  @Descrip : 串口初始化
		 *  @Notes   : None
		 ***********************************************/
		//bool serialInit();

		/***********************************************
		 *  @Name    : CImuCommand::serialInit
		 *  @Descrip : 总运行程序
		 *  @Notes   : None
		 ***********************************************/
		bool RUN();
                void pid_write_callback(const carMsgs::pidPtr &pid_msg);


		/***********************************************
		 *  @Name    : CImuCommand::display_decodeData
		 *  @Descrip : 打印角速度、线加速度
		 *  @Notes   : None
		 ***********************************************/
        bool display_decodeData(double tmpBuffer[DATA_LENGTH]);
				/***********************************************
		 *  @Name    : CImuCommand::display_Q4decodeData
		 *  @Descrip : 打印四元数
		 *  @Notes   : None
		 ***********************************************/
        bool display_Q4decodeData(double tmpBuffer[Q4DATA_LENGTH]);
        bool new_odommsg_flag=false;
    private:
       // class Serial *mSerial;
        bool init_serial(void);
        boost::system::error_code ec_;
        boost::asio::io_service io_service_;
        serialp_ptr sp_;

	public:
        unsigned char                pid_char_buffer[PIDCHAR_NUM];
        unsigned char                cmd_buffer[cmd_num];
        unsigned char     mulit_cmd_buffer[mulitCmd_num];
        unsigned char          r_buffer[READ_BUFFERSIZE];
        unsigned char          r_buffer_helper[26];
        unsigned char          *r_buffer_handled=NULL;
        unsigned char          odom_Vx_char[4];
        unsigned char          odom_Vy_char[4];
        unsigned char          odom_Vz_char[4];
        double                        EULAR[DATA_LENGTH];
        double              read_buffer[READ_BUFFERSIZE];
        double                          GYR[DATA_LENGTH];
        double                         Q4[Q4DATA_LENGTH];
        double                          ACC[DATA_LENGTH];
        float                 Vx_float;
        float                 Vy_float;
        float                 Vz_float;
        float                 x,y,th;
		
	};


}

#endif //IMUCOMMAND_H
