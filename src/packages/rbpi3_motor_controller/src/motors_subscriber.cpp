#include "ros/ros.h"
#include "rbpi3_motor_controller/motors_signal.h"

#include "wiringPi.h"

// Defining Pins
#define LEFT_FORWARD 0
#define LEFT_BACKWARD 2
#define RIGHT_FORWARD 4
#define RIGHT_BACKWARD 5


/*************************************************************************/
/************************* MOTORS CALLBACK *******************************/
/*************************************************************************/
void motors_callback(const rbpi3_motor_controller::motors_signal::ConstPtr& motors_signal){
    
    if (motors_signal->motor_left == 1) {
        digitalWrite (LEFT_BACKWARD, LOW) ;
        digitalWrite (LEFT_FORWARD, HIGH) ;
    }else if(motors_signal->motor_left == -1)
    {
        digitalWrite (LEFT_FORWARD, LOW) ;
        digitalWrite (LEFT_BACKWARD, HIGH) ;
    }

    if (motors_signal->motor_right == 1) {
        digitalWrite (RIGHT_BACKWARD, LOW) ;
        digitalWrite (RIGHT_FORWARD, HIGH) ;
    }else if(motors_signal->motor_right == -1)
    {
        digitalWrite (RIGHT_BACKWARD, LOW) ;
        digitalWrite (RIGHT_FORWARD, HIGH) ;   
    }

    if(motors_signal->motor_right == 0 and motors_signal->motor_left == 0)
    {
        digitalWrite (LEFT_FORWARD, LOW) ;
        digitalWrite (LEFT_BACKWARD, LOW) ;
    }
    
    ROS_INFO("motor left :%i  motor right: %i",motors_signal->motor_left,motors_signal->motor_right);
}

main(int argc, char **argv)
{
    // initialize ros node
    ros::init(argc,argv, "motors_subscriber");
    /************************************/
    /********* wiringPi setup ***********/
    /************************************/
    
    // setting all pins as wiringPi
    wiringPiSetup();
    // setting required pins as output
    pinMode(LEFT_FORWARD, OUTPUT);
    pinMode(LEFT_BACKWARD, OUTPUT);
    pinMode(RIGHT_FORWARD, OUTPUT);
    pinMode(RIGHT_BACKWARD, OUTPUT);

    /************************************/
    /*********** ROS setup **************/
    /************************************/
    // create an object of type nodehandle
    ros::NodeHandle node_object;
    // create a publisher object targeting a topic
    ros::Subscriber msg_subscriber = \
    node_object.subscribe<rbpi3_motor_controller::motors_signal>("/motors_topic",10,motors_callback);
    // spinning for callback
    ros::spin();
    
    return 0;
}
