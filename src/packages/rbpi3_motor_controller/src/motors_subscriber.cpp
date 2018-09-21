#include "ros/ros.h"
#include "rbpi3_motor_controller/motors_signal.h"

#include "wiringPi.h"

void motors_callback(const rbpi3_motor_controller::motors_signal::ConstPtr& motors_signal){
    ROS_INFO("motor left :%i  motor right: %i",motors_signal->motor_left,motors_signal->motor_right);
}

main(int argc, char **argv)
{
    // initialize ros node
    ros::init(argc,argv, "motors_subscriber");
    // testing wiringPi
    wiringPiSetup();
    // create an object of type nodehandle
    ros::NodeHandle node_object;
    // create a publisher object targeting a topic
    ros::Subscriber msg_subscriber = \
    node_object.subscribe<rbpi3_motor_controller::motors_signal>("/motors_topic",10,motors_callback);
    // spinning for callback
    ros::spin();
    
    return 0;
}
