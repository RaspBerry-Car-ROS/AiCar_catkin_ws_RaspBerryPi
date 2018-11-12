#include "ros/ros.h"
#include "wiringPi.h"
#include <wiringPiI2C.h>
#include "imu_measurements_package/Imu_message.h"

#define MPU6050_ADDRESS 0x68
#define ACCEL_X_1 0x3B
#define ACCEL_X_2 0x3C
#define ACCEL_Y_1 0x3D
#define ACCEL_Y_2 0x3E
#define ACCEL_Z_1 0x3F
#define ACCEL_Z_2 0x40
#define GYRO_X_1 0x43
#define GYRO_X_2 0x44
#define GYRO_Y_1 0x45
#define GYRO_Y_2 0x46
#define GYRO_Z_1 0x47
#define GYRO_Z_2 0x48
#define MPU_POWER 0x6B 
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG 0x1B
#define MPU_MASTER_MODE 0x6A
#define MPU_BYPASS_MODE 0x37 //Access the I2C bus to connect with slaves
#define MPU_SLAVE_0_REGISTER 0x25
#define MPU_SLAVE_0_STARTING_REGISTER 0x26
#define MPU_SLAVE_0_ENABLE_LEN_TRANSFER 0x27
#define MPU_SLAVE_TIMING_DATA 0x67
#define SLAVE_0_COMPASS_X1 0x49
#define SLAVE_0_COMPASS_X2 0x4A
#define SLAVE_0_COMPASS_Z1 0x4B
#define SLAVE_0_COMPASS_Z2 0x4C
#define SLAVE_0_COMPASS_Y1 0x4D
#define SLAVE_0_COMPASS_Y2 0x4E

#define HMC5883L_ADDRESS 0x1E
#define HMC_MODE 0x02
#define HMC_SCALE_MAGNETOMETER 0x01
#define COMPASS_X1 0x03
#define COMPASS_X2 0x04
#define COMPASS_Z1 0x05
#define COMPASS_Z2 0x06
#define COMPASS_Y1 0x07
#define COMPASS_Y2 0x08
main(int argc, char **argv)
{   
    int MPU_ID, COMPASS_ID;
    int16_t accel_x,accel_y,accel_z, gyro_x,gyro_y,gyro_z,compass_x,compass_y,compass_z;
    int8_t power_register;
    float gForceX,gForceY,gForceZ, rotationX, rotationY, rotationZ,magn_x,magn_y,magn_z;
    wiringPiSetup ();
    MPU_ID=wiringPiI2CSetup (MPU6050_ADDRESS) ; /*Use i2cdetect command to find your respective device address*/
    COMPASS_ID=wiringPiI2CSetup(HMC5883L_ADDRESS);
    
    //Initializing ROS node with the name of imu_sensors_publisher
	ros::init(argc, argv,"imu_sensor_publisher");
    //Created a node handle object
	ros::NodeHandle node_obj;

    //Created a publisher object
	ros::Publisher imu_sensors_publisher =  
                        node_obj.advertise<imu_measurements_package::Imu_message>("/imu_sensors_topic",10);// The 10 is the buffer size \
                                                                             (number of msgs accumulated before sending)
    // create a rate object to set frequency in hz
    ros::Rate frequency_loop(400);//this is the frequency to which the project will work that is 2.5ms reaction\
                                    in this way we light the load for the processor 
    if(MPU_ID==-1 or COMPASS_ID==-1){
        ROS_INFO("Can't setup the I2C device");
    }
    else{
        // waking-up MPU sensor
        wiringPiI2CWriteReg8(MPU_ID,MPU_POWER,0);
        // waking-up COmpass sensor //Note that all I2C communication is through the master MPU
        wiringPiI2CWriteReg8(MPU_ID,MPU_MASTER_MODE,0);//Disable Master mode
        wiringPiI2CWriteReg8(MPU_ID,MPU_BYPASS_MODE,0x02);//enable slave mode
        wiringPiI2CWriteReg8(COMPASS_ID,HMC_MODE,0);
        wiringPiI2CWriteReg8(COMPASS_ID,HMC_SCALE_MAGNETOMETER,0x60);//Setting magnetometer to +-2.5 Gauss
        wiringPiI2CWriteReg8(MPU_ID,MPU_BYPASS_MODE,0);//disable slave mode
        wiringPiI2CWriteReg8(MPU_ID,MPU_MASTER_MODE,0x20);//enable Master mode
        // Acelerometer setup
        wiringPiI2CWriteReg8(MPU_ID,ACCEL_CONFIG,0x08); //setting scale range +-4g
        // Gyroscope setup
        wiringPiI2CWriteReg8(MPU_ID,GYRO_CONFIG,0x08); //setting scale range +-500º/s
        // MPU reading magnetometer as slave setup
        wiringPiI2CWriteReg8(MPU_ID,MPU_SLAVE_0_REGISTER,HMC5883L_ADDRESS|0x80);//setting the adress of compass and MPU in reader mode for that slave
        wiringPiI2CWriteReg8(MPU_ID,MPU_SLAVE_0_STARTING_REGISTER,COMPASS_X1);//setting the register from which we will start to read from slave 0 i.e. from HMC5883L 
        wiringPiI2CWriteReg8(MPU_ID,MPU_SLAVE_0_ENABLE_LEN_TRANSFER,6|0x80); //6 bytes transferred and enable slave 0
        wiringPiI2CWriteReg8(MPU_ID,MPU_SLAVE_TIMING_DATA,1);//enables slave 0 delay, to read at the same time as the MPU's accelerometer and gyrpscope
        
        //Created a imu_measurements message object
		imu_measurements_package::Imu_message imu_msg;

        while ( ros::ok()){

            //getting the x accelerometer measurements in gForce units
            accel_x=wiringPiI2CReadReg8(MPU_ID,ACCEL_X_1)<< 8|wiringPiI2CReadReg8(MPU_ID,ACCEL_X_2);
            gForceX = accel_x/8192.0; 
            //getting the y accelerometer measurements in gForce units
            accel_y=wiringPiI2CReadReg8(MPU_ID,ACCEL_Y_1)<< 8|wiringPiI2CReadReg8(MPU_ID,ACCEL_Y_2);
            gForceY = accel_y/8192.0; 
            //getting the z accelerometer measurements in gForce units
            accel_z =wiringPiI2CReadReg8(MPU_ID,ACCEL_Z_1)<< 8|wiringPiI2CReadReg8(MPU_ID,ACCEL_Z_2);
            gForceZ = accel_z/8192.0; 
            ////ROS_INFO("acceleration in x: %f, acceleration in y: %f, acceleration in z: %f",gForceX,gForceY,gForceZ);
            //getting the x gyroscope measurements in º/s units
            gyro_x=wiringPiI2CReadReg8(MPU_ID,GYRO_X_1)<< 8|wiringPiI2CReadReg8(MPU_ID,GYRO_X_2);
            rotationX = gyro_x/65.5; 
            //getting the x gyroscope measurements in º/s units
            gyro_y=wiringPiI2CReadReg8(MPU_ID,GYRO_Y_1)<< 8|wiringPiI2CReadReg8(MPU_ID,GYRO_Y_2);
            rotationY = gyro_y/65.5; 
            //getting the x gyroscope measurements in º/s units
            gyro_z=wiringPiI2CReadReg8(MPU_ID,GYRO_Z_1)<< 8|wiringPiI2CReadReg8(MPU_ID,GYRO_Z_2);
            rotationZ = gyro_z/65.5; 
            //ROS_INFO("angular velocity in x: %f, in y: %f, in z: %f",rotationX,rotationY,rotationZ);
            
            //getting the x magnetomer measurements in gauss units
            compass_x=wiringPiI2CReadReg8(MPU_ID,SLAVE_0_COMPASS_X1)<< 8|wiringPiI2CReadReg8(MPU_ID,SLAVE_0_COMPASS_X2);
            magn_x = compass_x/660.0; 
            //getting the y magnetomer measurements in gauss units
            compass_y=wiringPiI2CReadReg8(MPU_ID,SLAVE_0_COMPASS_Y1)<< 8|wiringPiI2CReadReg8(MPU_ID,SLAVE_0_COMPASS_Y2);
            magn_y = compass_y/660.0; 
            //getting the x magnetomer measurements in gauss units
            compass_z=wiringPiI2CReadReg8(MPU_ID,SLAVE_0_COMPASS_Z1)<< 8|wiringPiI2CReadReg8(MPU_ID,SLAVE_0_COMPASS_Z2);
            magn_z = compass_z/660.0; 
            //ROS_INFO("magnetomer in gauss x: %f, in y: %f, in z: %f",magn_x,magn_y,magn_z);

            //Inserted data to message variables
            imu_msg.acceleration_x_gF=gForceX;
            imu_msg.acceleration_y_gF=gForceY;
            imu_msg.acceleration_z_gF=gForceZ;
            imu_msg.rotation_x_degSec=rotationX;
            imu_msg.rotation_y_degSec=rotationY;
            imu_msg.rotation_z_degSec=rotationZ;
            imu_msg.compass_x_gauss=magn_x;
            imu_msg.compass_y_gauss=magn_y;
            imu_msg.compass_z_gauss=magn_z;

            //Publishing the topic 
		    imu_sensors_publisher.publish(imu_msg);

            // frequency loop sleeping the necessary time for meeting the frequency stated before 
            // it sleeps the time remaining to meet the frequency after executing all the above code lines 
            frequency_loop.sleep();
        }
    }
    
    return 0;
}