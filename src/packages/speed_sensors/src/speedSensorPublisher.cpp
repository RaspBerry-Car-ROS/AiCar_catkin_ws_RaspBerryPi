#include "ros/ros.h"
#include "wiringPi.h"
#include "speed_sensor/speed_sensors_msg.h"
// Defining Pins
#define INTERRUPT_PIN_R 3
#define INTERRUPT_PIN_L 7
#define WHEEL_DIAMETER 0.065 //meters
#define PI 3.14159265359
// the event counter 
volatile int eventCounter_R = 0;
volatile int eventCounter_L = 0;
// number of wheel revolutions
volatile float distance_R = 0.0;
volatile float distance_L = 0.0;

void myInterruptR(void) {
   eventCounter_R++;
   distance_R = (eventCounter_R/40.0)*PI*WHEEL_DIAMETER;
}

void myInterruptL(void) {
   eventCounter_L++;
   distance_L = (eventCounter_L/40.0)*PI*WHEEL_DIAMETER;

}

main(int argc, char **argv)
{
    // setting all pins as wiringPi
    wiringPiSetup();
    // setting required pins as input
    pinMode(INTERRUPT_PIN_R, INPUT);
    pinMode(INTERRUPT_PIN_L, INPUT);
    // setting pull down resistor for the pin
    pullUpDnControl (INTERRUPT_PIN_R, PUD_DOWN);
    pullUpDnControl (INTERRUPT_PIN_L, PUD_DOWN);
    // setting the interruption funtion with its callback myInterrupt
    wiringPiISR (INTERRUPT_PIN_R, INT_EDGE_RISING, &myInterruptR);
    wiringPiISR (INTERRUPT_PIN_L, INT_EDGE_RISING, &myInterruptL);

    //Initializing ROS node with the name of speed_sensors_publisher
	ros::init(argc, argv,"speed_sensors_publisher");

    //Created a node handle object
	ros::NodeHandle node_obj;

    //Created a publisher object
	ros::Publisher speed_sensors_publisher =  
                        node_obj.advertise<speed_sensor::speed_sensors_msg>("/speed_sensors_topic",10);// The 10 is the buffer size \
                                                                             (number of msgs accumulated before sending)

    while ( ros::ok()) {
        

        //Created a speed_sensors_msg message object
		speed_sensor::speed_sensors_msg sensors_msg;
        
        //Inserted data to message variables
        sensors_msg.left_wheel_distance=distance_L;
        sensors_msg.right_wheel_distance=distance_R;

        //printing message data
        ROS_INFO( "---Distance traveled--- LEFT: %f  RIGHT: %f", sensors_msg.left_wheel_distance,\
                                                                sensors_msg.right_wheel_distance);

        //Publishing the topic 
		speed_sensors_publisher.publish(sensors_msg);

        //Spinning once for doing the  all operation once (not really necessary for publishers)
		ros::spinOnce();
    }

    return 0;
}
