// https://github.com/richardghirst/PiBits/blob/master/MPU6050-Pi-Demo/demo_dmp.cpp
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <signal.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/transform_datatypes.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

//Typically, motion processing algorithms should be run at a high rate, often around 200Hz,
//in order to provide accurate results with low latency. This is required even if the application
//updates at a much lower rate; for example, a low power user interface may update as slowly
//as 5Hz, but the motion processing should still run at 200Hz.
//Page 25 of MPU6050 datasheet.
#define DEFAULT_SAMPLE_RATE_HZ	10

#define MPU_FRAMEID "base_imu"

#define SENSORS_DPS_TO_RADS               (0.017453293F)          /**< Degrees/s to rad/s multiplier */

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gg;         // [x, y, z]            gyro sensor measurments


int sample_rate;
std::string frame_id;

// mpu offsets
int ax, ay, az, gx, gy, gz;

// float gyro_scale = 1;
// if (gyro_range == MPU6050_RANGE_250_DEG)
// gyro_scale = 131;
// if (gyro_range == MPU6050_RANGE_500_DEG)
// gyro_scale = 65.5;
// if (gyro_range == MPU6050_RANGE_1000_DEG)
// gyro_scale = 32.8;
// if (gyro_range == MPU6050_RANGE_2000_DEG)
// gyro_scale = 16.4;
float gyro_scale = 131;

bool debug = false;

// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
bool ado = false;

double angular_velocity_covariance, pitch_roll_covariance, yaw_covariance, linear_acceleration_covariance;
double linear_acceleration_stdev_, angular_velocity_stdev_, yaw_stdev_, pitch_roll_stdev_;

ros::Publisher imu_pub;

void mySigintHandler(int sig){
	ROS_INFO("Shutting down mpu6050_node...");

	mpu.reset();

	// All the default sigint handler does is call shutdown()
	ros::shutdown();
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop(ros::NodeHandle pn, ros::NodeHandle n) {

    // if programming failed, don't try to do anything
    if (!dmpReady) return;

	ros::Time now = ros::Time::now();

	//http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/Imu.html
	sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = now;
    imu_msg.header.frame_id = frame_id;

    // http://www.i2cdevlib.com/forums/topic/4-understanding-raw-values-of-accelerometer-and-gyrometer/
	//The output scale for any setting is [-32768, +32767] for each of the six axes.
    //The default setting in the I2Cdevlib class is +/- 2g for the accel and +/- 250 deg/sec
    //for the gyro. If the device is perfectly level and not moving, then:
	//
	//    X/Y accel axes should read 0
	//    Z accel axis should read 1g, which is +16384 at a sensitivity of 2g
	//    X/Y/Z gyro axes should read 0
	//
	//In reality, the accel axes won't read exactly 0 since it is difficult to be perfectly level
    //and there is some noise/error, and the gyros will also not read exactly 0 for the same
    //reason (noise/error).

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    if (fifoCount == 1024) {

        // reset so we can continue cleanly
        mpu.resetFIFO();
        if(debug) printf("FIFO overflow!\n");

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (fifoCount >= 42) {

    	// read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

		// display quaternion values in easy matrix form: w x y z
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		if(debug) printf("quat %7.2f %7.2f %7.2f %7.2f    ", q.w,q.x,q.y,q.z);

	    imu_msg.orientation.x = q.x;
		imu_msg.orientation.y = q.y;
		imu_msg.orientation.z = q.z;
		imu_msg.orientation.w = q.w;

		imu_msg.linear_acceleration_covariance[0] = linear_acceleration_covariance;
		imu_msg.linear_acceleration_covariance[4] = linear_acceleration_covariance;
		imu_msg.linear_acceleration_covariance[8] = linear_acceleration_covariance;

		imu_msg.angular_velocity_covariance[0] = angular_velocity_covariance;
		imu_msg.angular_velocity_covariance[4] = angular_velocity_covariance;
		imu_msg.angular_velocity_covariance[8] = angular_velocity_covariance;

		imu_msg.orientation_covariance[0] = pitch_roll_covariance;
		imu_msg.orientation_covariance[4] = pitch_roll_covariance;
		imu_msg.orientation_covariance[8] = yaw_covariance;
        
        // http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
		// Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
        // see also https://answers.ros.org/question/200480/imu-message-definition/
        mpu.dmpGetAccel(&aa, fifoBuffer);
        imu_msg.linear_acceleration.x = aa.x * 1/16384. * 9.80665;
        imu_msg.linear_acceleration.y = aa.y * 1/16384. * 9.80665;
        imu_msg.linear_acceleration.z = aa.z * 1/16384. * 9.80665;

        if(debug) printf("areal (raw) %6d %6d %6d    ", aa.x, aa.y, aa.z);
        if(debug) printf("areal (m/s^2) %6d %6d %6d    ", imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z);
        
        mpu.dmpGetGyro(&gg, fifoBuffer);
        // Should be in rad/sec.
        imu_msg.angular_velocity.x = float(gg.x) / gyro_scale * SENSORS_DPS_TO_RADS;
        imu_msg.angular_velocity.y = float(gg.y) / gyro_scale * SENSORS_DPS_TO_RADS;
        imu_msg.angular_velocity.z = float(gg.z) / gyro_scale * SENSORS_DPS_TO_RADS;

		imu_pub.publish(imu_msg);

		if(debug) printf("\n");
    }
}

int main(int argc, char **argv){

    ros::init(argc, argv, "mpu6050");

    // Allows parameters passed in via <param>
    ros::NodeHandle pn("~");

    // Does not allow parameters being passed in.
    ros::NodeHandle n;

    signal(SIGINT, mySigintHandler);

    ROS_INFO("Starting mpu6050_node...");

    pn.param<int>("frequency", sample_rate, DEFAULT_SAMPLE_RATE_HZ);
	std::cout << "Using sample rate: " << sample_rate << std::endl;

    pn.param<std::string>("frame_id", frame_id, MPU_FRAMEID);
    std::cout << "Using frame_id: " << frame_id << std::endl;

    pn.param<int>("ax", ax, 0);
    pn.param<int>("ay", ay, 0);
    pn.param<int>("az", az, 0);
    pn.param<int>("gx", gx, 0);
    pn.param<int>("gy", gy, 0);
    pn.param<int>("gz", gz, 0);

    pn.param<bool>("ado", ado, false);
    std::cout << "ADO: " << ado << std::endl << std::flush;

    pn.param<bool>("debug", debug, false);
    std::cout << "Debug: " << debug << std::endl << std::flush;

    // NOISE PERFORMANCE: Power Spectral Density @10Hz, AFS_SEL=0 & ODR=1kHz 400 ug/√Hz (probably wrong)
    pn.param("linear_acceleration_stdev", linear_acceleration_stdev_, (400 / 1000000.0) * 9.807 );

    // Total RMS Noise: DLPFCFG=2 (100Hz) 0.05 º/s-rms (probably lower (?) @ 42Hz)
    pn.param("angular_velocity_stdev", angular_velocity_stdev_, 0.05 * (M_PI / 180.0));

    // 1 degree for pitch and roll
    pn.param("pitch_roll_stdev", pitch_roll_stdev_, 1.0 * (M_PI / 180.0));

    // 5 degrees for yaw
    pn.param("yaw_stdev", yaw_stdev_, 5.0 * (M_PI / 180.0));

    angular_velocity_covariance = angular_velocity_stdev_ * angular_velocity_stdev_;
    linear_acceleration_covariance = linear_acceleration_stdev_ * linear_acceleration_stdev_;
    pitch_roll_covariance = pitch_roll_stdev_ * pitch_roll_stdev_;
    yaw_covariance = yaw_stdev_ * yaw_stdev_;

    // ================================================================
    // ===                      INITIAL SETUP                       ===
    // ================================================================

    printf("Initializing I2C...\n");
    I2Cdev::initialize();

    // verify connection
    printf("Testing device connections...\n");
    mpu = MPU6050(ado ? 0x69 : 0x68);
    if(mpu.testConnection()){
        std::cout << "MPU6050 connection successful" << std::endl << std::flush;
    }else{
        std::cout << "MPU6050 connection failed" << std::endl << std::flush;
        return 1;
    }

    // initialize device
    printf("Initializing I2C devices...\n");
    mpu.initialize();

    // load and configure the DMP
    printf("Initializing DMP...\n");
    devStatus = mpu.dmpInitialize();

    // Set accel offsets.
    std::cout << "Setting X accel offset: " << ax << std::endl;
    mpu.setXAccelOffset(ax);
    std::cout << "Setting Y accel offset: " << ay << std::endl;
    mpu.setYAccelOffset(ay);
    std::cout << "Setting Z accel offset: " << az << std::endl;
    mpu.setZAccelOffset(az);

    // Set gyro offsets.
    std::cout << "Setting X gyro offset: " << gx << std::endl;
    mpu.setXGyroOffset(gx);
    std::cout << "Setting Y gyro offset: " << gy << std::endl;
    mpu.setYGyroOffset(gy);
    std::cout << "Setting Z gyro offset: " << gz << std::endl;
    mpu.setZGyroOffset(gz);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        printf("Enabling DMP...\n");
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        printf("DMP ready!\n");
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        printf("DMP Initialization failed (code %d)\n", devStatus);
    }

    usleep(100000);

    imu_pub = n.advertise<sensor_msgs::Imu>("imu/data", 10);

    ros::Rate r(sample_rate);
    while(ros::ok()){
        loop(pn, n);
        ros::spinOnce();
        r.sleep();
    }

    std::cout << "Shutdown." << std::endl << std::flush;

    return 0;

}
