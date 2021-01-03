#include"ImuCommand.h"

ros::Publisher msg_pub;
ros::Subscriber pid_sub;
ros::Publisher odom_pub;  
IPSG::CImuCommand::~CImuCommand(){

     if(sp_)
    {
        sp_->cancel();
        sp_->close();
        sp_.reset();
    }
    io_service_.stop();
    io_service_.reset();
}
bool IPSG::CImuCommand::init_serial(void)
{
if(sp_)
    {
        ROS_ERROR("The SerialPort is already opened!");
        return false;
    }
     sp_ = serialp_ptr(new boost::asio::serial_port(io_service_));
     sp_->open("/dev/ttyUSB0",ec_);
     if(ec_)
     {
        ROS_ERROR_STREAM("error : port_->open() failed...port_name=" << "USBO"<< ", e=" << ec_.message().c_str());
        return false;
     }
    sp_->set_option(boost::asio::serial_port_base::baud_rate(115200));
    sp_->set_option(boost::asio::serial_port_base::character_size(8));
    sp_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    sp_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    sp_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
}
//Convert float to char and send it through serial port
void IPSG::CImuCommand::float2char(float f,unsigned char *s)
{
 unsigned char *p;
 p = (unsigned char *)&f;

    *s = *p;

    *(s+1) = *(p+1);

    *(s+2) = *(p+2);

    *(s+3) = *(p+3);

}
void IPSG::CImuCommand::char2float(float *f,unsigned char *s)
{
    //f = (float *)s;
    memcpy(f, s, 4);
}

bool IPSG::CImuCommand::display_decodeData(double *tmpBuffer)
{
    for(int i = 0;i < DATA_LENGTH;i ++)
    {
        std::cout<<tmpBuffer[i]<<std::endl;
    }
    return true;
}

bool IPSG::CImuCommand::display_Q4decodeData(double tmpBuffer[Q4DATA_LENGTH])
{
    for(int i = 0;i < Q4DATA_LENGTH;i ++)
    {
        std::cout<<tmpBuffer[i]<<std::endl;
    }
    return true;
}

bool IPSG::CImuCommand::display_Imumsg(sensor_msgs::Imu imumsg)
{   
   /* std::cout<<"print Q4 data!"<<std::endl;
    std::cout<<imumsg.orientation.x<<std::endl;
    std::cout<<imumsg.orientation.y<<std::endl;
    std::cout<<imumsg.orientation.z<<std::endl;
    std::cout<<imumsg.orientation.w<<std::endl;*/

    std::cout<<"print GRY data!"<<std::endl;
    std::cout<<imumsg.angular_velocity.x<<std::endl;
    std::cout<<imumsg.angular_velocity.y<<std::endl;
    std::cout<<imumsg.angular_velocity.z<<std::endl;

    std::cout<<"print ACC data!"<<std::endl;
    std::cout<<imumsg.linear_acceleration.x<<std::endl;
    std::cout<<imumsg.linear_acceleration.y<<std::endl;
    std::cout<<imumsg.linear_acceleration.z<<std::endl;

    std::cout<<"Odom data-----------------!"<<std::endl;
    std::cout<<Vx_float<<std::endl;
    std::cout<<Vy_float<<std::endl;
    std::cout<<Vz_float<<std::endl;
}

bool IPSG::CImuCommand::decodeFrame(unsigned char tmpBuffer[READ_BUFFERSIZE])
{
    if(tmpBuffer[0] == DATA_FrameHead && tmpBuffer[1] == DATA_FrameHead)
    {
        imu_data.header.frame_id = "imu_link";
 	imu_data.header.stamp = ros::Time::now();
        switch(tmpBuffer[2])
        {
            case DATA_TYPE_Q4:  //该帧输出数据为四元数类型
                { 
                   /* Q4[0] = (double)(tmpBuffer[4]<<8|tmpBuffer[5])/Q4_MEASURE;
                    Q4[1] = (double)(tmpBuffer[6]<<8|tmpBuffer[7])/Q4_MEASURE;
                    Q4[2] = (double)(tmpBuffer[8]<<8|tmpBuffer[9])/Q4_MEASURE;
                    Q4[3] = (double)(tmpBuffer[10]<<8|tmpBuffer[11])/Q4_MEASURE;
                    display_Q4decodeData(Q4);
                    
                    imu_data.orientation.x = Q4[1];
                    imu_data.orientation.y = Q4[2];
                    imu_data.orientation.z = Q4[3];
                    imu_data.orientation.w = Q4[0];*/
                    
                    break;
                }


            case DATA_TYPE_GRY: //该帧输出数据为陀螺仪数据类型
                {                
                    GYR[0] = (short)(((uint16_t)tmpBuffer[4]<<8)|tmpBuffer[5]);
                    GYR[1] = (short)(((uint16_t)tmpBuffer[6]<<8)|tmpBuffer[7]);
                    GYR[2] = (short)(((uint16_t)tmpBuffer[8]<<8)|tmpBuffer[9]);
                    display_decodeData(GYR);
                    imu_data.angular_velocity.x = (GYR[0]/16.04*M_PI/180);
                    imu_data.angular_velocity.y = (GYR[1]/16.04*M_PI/180);
                    imu_data.angular_velocity.z = (GYR[2]/16.04*M_PI/180);

                    break;
                }

            case DATA_TYPE_ACC:  //该帧输出数据为加速度类型
                 { 
                     ACC[0] = (short)((uint16_t)tmpBuffer[4]<<8|tmpBuffer[5]);
                     ACC[1] = (short)((uint16_t)tmpBuffer[6]<<8|tmpBuffer[7]);
                     ACC[2] = (short)((uint16_t)tmpBuffer[8]<<8|tmpBuffer[9]);
                     display_decodeData(ACC);
                     imu_data.linear_acceleration.x = (ACC[0]/2048*9.98);
                     imu_data.linear_acceleration.y = (ACC[1]/2048*9.98);
                     imu_data.linear_acceleration.z = (ACC[2]/2048*9.98);
                     
                     break;
                 } 
            case DATA_TYPE_ODOM_Vx:
                 {
                     odom_Vx_char[0] = tmpBuffer[4];
                     odom_Vx_char[1] = tmpBuffer[5];
                     odom_Vx_char[2] = tmpBuffer[6];
                     odom_Vx_char[3] = tmpBuffer[7];
                     IPSG::CImuCommand::char2float(&Vx_float,odom_Vx_char);
                     new_odommsg_flag=true;

                 } 
            case DATA_TYPE_ODOM_Vy:
                 {
                     odom_Vy_char[0] = tmpBuffer[4];
                     odom_Vy_char[1] = tmpBuffer[5];
                     odom_Vy_char[2] = tmpBuffer[6];
                     odom_Vy_char[3] = tmpBuffer[7];
                     IPSG::CImuCommand::char2float(&Vy_float,odom_Vy_char);

                 } 
            case DATA_TYPE_ODOM_Vz:
                 {
                     odom_Vz_char[0] = tmpBuffer[4];
                     odom_Vz_char[1] = tmpBuffer[5];
                     odom_Vz_char[2] = tmpBuffer[6];
                     odom_Vz_char[3] = tmpBuffer[7];
                     IPSG::CImuCommand::char2float(&Vz_float,odom_Vz_char);
                 } 

            case DATA_TYPE_MAG:  {std::cout<<"未更新DATA_TYPE_MAG解码程序！"<<std::endl;break;}   //该帧输出数据为磁力计类型

            case DATA_TYPE_EULAR:  //解码欧拉角原始数据
                { 
                     EULAR[0] = (double)(tmpBuffer[4]<<8|tmpBuffer[5])/EULAR_MEASURE;
                     EULAR[1] = (double)(tmpBuffer[6]<<8|tmpBuffer[7])/EULAR_MEASURE;
                     EULAR[2] = (double)(tmpBuffer[8]<<8|tmpBuffer[9])/EULAR_MEASURE;
                     display_decodeData(EULAR);
                     break;
                }

            case DATA_TYPE_STOP: {std::cout<<"未更新DATA_TYPE_STOP解码程序！"<<std::endl;break;} //该帧输出数据为保留不用类型

            case DATA_TYPE_PRECISION:{std::cout<<"未更新DATA_TYPE_PRECISION解码程序！"<<std::endl;break;}  //该帧输出数据为传感器精度，频率类型

            case DATA_TYPE_RANGE: {std::cout<<"未更新DATA_TYPE_RANGE解码程序！"<<std::endl;break;}  //该帧输出数据为传感器量程
            default: break;
        }

    }
    else
    {
        std::cout<<"数据帧头错误"<<std::endl;
        return false;
    }
    imu_data.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,imu_data.linear_acceleration.z);
    imu_data.orientation_covariance = {0.05, 0, 0,
	    			       0, 0.05, 0,
				       0, 0, 0.05};
    imu_data.angular_velocity_covariance = {0.01, 0, 0,
	    				    0, 0.01, 0,
					    0, 0, 0.01};
    imu_data.linear_acceleration_covariance = {-1, 0, 0,
	    				        0, 0, 0,
					        0, 0, 0};

    msg_pub.publish(imu_data);
    return true;
}


bool IPSG::CImuCommand::cmdFrame(unsigned char imucmd)
{
    cmd_buffer[0] = CMD_FrameHead;
    cmd_buffer[1] = imucmd;
    cmd_buffer[2] = CMD_FrameHead + cmd_buffer[1];
    //return commd_buffer;

    //ser.write(cmd_buffer,cmd_num);
    int i=0;
//tarmychange
    //ser.read(r_buffer,READ_BUFFERSIZE);
    boost::asio::read(*sp_.get(), boost::asio::buffer(r_buffer,READ_BUFFERSIZE), ec_);
    if(r_buffer[12]!=DATA_TYPE_STOP)
    {
      for (;i<26;i++)
     {
      if(r_buffer[i]=DATA_TYPE_STOP)break;

     }
//tarmychange
    //ser.read(r_buffer_helper,READ_BUFFERSIZE+i+1);
    boost::asio::read(*sp_.get(), boost::asio::buffer(r_buffer_helper,READ_BUFFERSIZE+i+1), ec_);
    r_buffer_handled=(&r_buffer_helper[i+1]);
    decodeFrame(r_buffer_handled);
   ROS_INFO("The message was truncated");
    }else
    decodeFrame(r_buffer);
    return true;
}


bool IPSG::CImuCommand::muliteCmdFrame(unsigned char imucmd1,unsigned char imucmd2,unsigned char imucmd3)
{
    //第一个指令
   // cmdFrame(imucmd1);
    //ROS_INFO("OUTPUT_Q4");   //如何输出宏定义的名字？？？？

    //第二个指令
    //cmdFrame(imucmd2);
    ROS_INFO("OUTPUT_GYR");

    //第三个指令
    cmdFrame(imucmd3);
    ROS_INFO("OUTPUT_ACC");

return true;
}


void IPSG::CImuCommand::pid_write_callback(const carMsgs::pidPtr &pid_msg)
{
     int i=0;
     float pid[3];
     pid[0]=pid_msg->k1_p; 
     pid[1]=pid_msg->k1_i;
     pid[2]=pid_msg->k1_d;
     for (;i<3;i++){
     IPSG::CImuCommand::float2char(pid[i],&pid_char_buffer[4*i]);
     }
  //tarmychange
    if(ec_){
        ROS_INFO("Serial Port has not been opened");
      }else{

        ROS_INFO_STREAM("Serial Port initialized");
       // cmdFrame(CMD_OUTPUT_200HZ);
       //tarmychange
        //ser.write(pid_char_buffer,13);
       boost::asio::write(*sp_.get(),boost::asio::buffer(pid_char_buffer,13),ec_);
      }
}
bool IPSG::CImuCommand::RUN()
{   
    ros::NodeHandle nh;
    
    int odom_count;
   // ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    msg_pub = nh.advertise<sensor_msgs::Imu>("imu_raw", 100);
    odom_pub =nh.advertise<nav_msgs::Odometry>("odom_wheel", 100);
    tf::TransformBroadcaster odom_broadcaster;
    pid_sub = nh.subscribe("pid_float",200,&IPSG::CImuCommand::pid_write_callback,this);
    //串口初始化
    init_serial();
    ros::Rate loop_rate(300);
    pid_char_buffer[12]='\r';
    //for odom 
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    while(ros::ok())
    {

            muliteCmdFrame(CMD_PULL_OUTPUT_Q4,CMD_PULL_OUTPUT_GYR,CMD_PULL_OUTPUT_ACC);
            display_Imumsg(imu_data);
            //msg_pub.publish(imu_data);
            ros::spinOnce();
            //for odom
            current_time = ros::Time::now();
            //compute odometry in a typical way given the velocities of the robot
            float dt = (current_time - last_time).toSec();
            float delta_x = (Vx_float * cos(th)) * dt;
            float delta_y = (Vy_float * sin(th)) * dt;
            float delta_th = Vz_float * dt;
       
             x += delta_x;
             y += delta_y;
             th += delta_th;
             //since all odometry is 6DOF we'll need a quaternion created from yaw
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
            //tf提供的功能允许从yaw(偏航值)容易地创建四元数，并且可以方便地获取四元数的偏航值。     
             //first, we'll publish the transform over tf
             geometry_msgs::TransformStamped odom_trans;
             odom_trans.header.stamp = current_time;
             odom_trans.header.frame_id = "wheel_link";
             odom_trans.child_frame_id = "base_footprint";
             /*在这里，我们将创建一个TransformStamped消息，通过tf发送。
            我们要发布在current_time时刻的从"odom"坐标系到“base_link”坐标系的变换。
            因此，我们将相应地设置消息头和child_frame_id，
            确保使用“odom”作为父坐标系，将“base_link”作为子坐标系。*/
 
            odom_trans.transform.translation.x = x;
            odom_trans.transform.translation.y = y;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = odom_quat;
 
            //send the transform
            odom_broadcaster.sendTransform(odom_trans);
            //将我们的里程数据中填入变换消息中，然后使用TransformBroadcaster发送变换。
 
            //next, we'll publish the odometry message over ROS
            nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "wheel_link";
        //还需要发布nav_msgs/Odometry消息，以便导航包可以从中获取速度信息。
        //将消息的header设置为current_time和“odom”坐标系。
        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
 
        //set the velocity
        odom.child_frame_id = "base_footprint";
        odom.twist.twist.linear.x = Vx_float;
        odom.twist.twist.linear.y = Vy_float;
        odom.twist.twist.angular.z= Vz_float;
        //这将使用里程数据填充消息，并发送出去。
        //我们将消息的child_frame_id设置为“base_link”坐标系，
        //因为我们要发送速度信息到这个坐标系。
 
        //fill cov
    if((Vx_float==0)&&(Vy_float==0)&&(Vz_float==0))
    {
    odom.twist.covariance = { 1e-9, 0, 0, 0, 0, 0, 
                              0, 1e-3, 1e-9, 0, 0, 0, 
                              0, 0, 1e6, 0, 0, 0,
                              0, 0, 0, 1e6, 0, 0, 
                              0, 0, 0, 0, 1e6, 0, 
                              0, 0, 0, 0, 0, 1e-9 };
    odom.pose.covariance = { 1e-9, 0, 0, 0, 0, 0, 
                              0, 1e-3, 1e-9, 0, 0, 0, 
                              0, 0, 1e6, 0, 0, 0,
                              0, 0, 0, 1e6, 0, 0, 
                              0, 0, 0, 0, 1e6, 0, 
                              0, 0, 0, 0, 0, 1e-9 };
    }
    else
    {
    odom.twist.covariance = { 1e-3, 0, 0, 0, 0, 0, 
                              0, 1e-3, 1e-9, 0, 0, 0, 
                              0, 0, 1e6, 0, 0, 0,
                              0, 0, 0, 1e6, 0, 0, 
                              0, 0, 0, 0, 1e6, 0, 
                              0, 0, 0, 0, 0, 1e-3 };
    odom.pose.covariance = { 1e-3, 0, 0, 0, 0, 0, 
                              0, 1e-3, 1e-9, 0, 0, 0, 
                              0, 0, 1e6, 0, 0, 0,
                              0, 0, 0, 1e6, 0, 0, 
                              0, 0, 0, 0, 1e6, 0, 
                              0, 0, 0, 0, 0, 1e-3 };

    }
        //publish the message
        //ROS_INFO("The message was truncated===============================");

        odom_pub.publish(odom);
        last_time = current_time;


            loop_rate.sleep();
    }

        return true;
}
