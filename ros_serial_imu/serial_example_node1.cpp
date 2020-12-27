#include "ImuCommand.h"

int main (int argc, char** argv)
{
    ros::init(argc, argv, "serial_example_node1");
    IPSG::CImuCommand  ImuCmd;
    ImuCmd.RUN();

    return true ;   
 }
   


