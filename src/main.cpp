/*
 * MIT License (MIT)
 *
 * Copyright (c) 2018 Dereck Wonnacott <dereck@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 */
 
 // gps and imu data are saved to csv files
 // files can be checked under .ros folder
 // author : @wntun 
 // gps & imu data saving: 6 March 2019
 // image capturing and saving using libgphoto, gphoto2: 19 May 2019 (For this, libgphoto and gphoto2 are required to install.)
 // 12 June, 2019
 // camera triggering is done with gpio (libgphoto or gphoto are not required anymore) 
 // gps file format and imu file format are updated
 // ku_gps msg file is created for new gps file format
 // major change : it waits for fc signal for triggering and ins data saving

#include <iostream>
#include <fstream>
#include <cmath>
#include <inttypes.h>

// ros timestamp to iso-8601
// #include "boost/date_time/posix_time/posix_time.hpp"
#include <ctime>

//thread
#include <chrono>
#include <thread>
#include <functional>

//pwm libraries
#include <stdio.h>
#include <wiringPi.h>
//#include<softPwm.h>

// camera gphoto libraries
/*
#include <fcntl.h>
#include <gphoto2/gphoto2-camera.h>
#include <gphoto2/gphoto2.h>
*/

// ROS Libraries
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/FluidPressure.h"
#include "std_srvs/Empty.h"
#include "vectornav/ku_gps.h"


#define PI 3.14159265358979323846  /* pi */
#define CAMERAPIN 31
#define FCPIN 29
#define DELAY_MS 100

ros::Publisher pubIMU, pubMag, pubGPS, pubKuGPS, pubOdom, pubTemp, pubPres;
ros::ServiceServer resetOdomSrv;

//Unused covariances initilized to zero's
boost::array<double, 9ul> linear_accel_covariance = { };
boost::array<double, 9ul> angular_vel_covariance = { };
boost::array<double, 9ul> orientation_covariance = { };
XmlRpc::XmlRpcValue rpc_temp;



// Include this header file to get access to VectorNav sensors.
#include "vn/sensors.h"
#include "vn/compositedata.h"
#include "vn/util.h"

using namespace std;
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;


#if !defined (O_BINARY)
    /*To have portable binary open() on *nix and on Windows */
    #define O_BINARY 0
#endif
// Method declarations for future use.
void BinaryAsyncMessageReceived(void* userData, Packet& p, size_t index);
void writeCSVFiles(int);
void takePicture();
//void timer_start(std::function<void(void)> func, unsigned int interval);
//void capture_to_file();
//void takePicture_gphoto();
//GPContext* sample_create_context();

std::ofstream imu_writer;
std::ofstream gps_writer;
std::ofstream img_stamp_writer;
std::string file_folder;

std::string frame_id;
// Boolean to use ned or enu frame. Defaults to enu which is data format from sensor.
bool tf_ned_to_enu;
// Initial position after getting a GPS fix.
vec3d initial_position;
bool initial_position_set = false;

sensor_msgs::Imu msgIMU;
vectornav::ku_gps msgkuGPS;
//camera gphoto
//Camera *camera;
//GPContext *context;


// Basic loop so we can initilize our covariance parameters above
boost::array<double, 9ul> setCov(XmlRpc::XmlRpcValue rpc){
    // Output covariance vector
    boost::array<double, 9ul> output = { 0.0 };

    // Convert the RPC message to array
    ROS_ASSERT(rpc.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for(int i = 0; i < 9; i++){
        ROS_ASSERT(rpc[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        output[i] = (double)rpc[i];
    }
    return output;
}

/*

static void
ctx_error_func (GPContext *context, const char *str, void *data)
{
        fprintf  (stderr, "\n*** Contexterror ***              \n%s\n",str);
        fflush   (stderr);
}

static void
ctx_status_func (GPContext *context, const char *str, void *data)
{
        fprintf  (stderr, "%s\n", str);
        fflush   (stderr);
}

static void errordumper(GPLogLevel level, const char *domain, const char *str,
                 void *data) {
  fprintf(stdout, "%s\n", str);
}

*/
// Reset initial position to current position
bool resetOdom(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
    initial_position_set = false;
    return true;
}

int main(int argc, char *argv[])
{

    // ROS node init
    ros::init(argc, argv, "vectornav");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    pubIMU = n.advertise<sensor_msgs::Imu>("vectornav/IMU", 1000);
    pubMag = n.advertise<sensor_msgs::MagneticField>("vectornav/Mag", 1000);
    pubGPS = n.advertise<sensor_msgs::NavSatFix>("vectornav/GPS", 1000);
    pubKuGPS = n.advertise<vectornav::ku_gps>("vectornav/KU_GPS", 1000);
    pubOdom = n.advertise<nav_msgs::Odometry>("vectornav/Odom", 1000);
    pubTemp = n.advertise<sensor_msgs::Temperature>("vectornav/Temp", 1000);
    pubPres = n.advertise<sensor_msgs::FluidPressure>("vectornav/Pres", 1000);

    resetOdomSrv = n.advertiseService("reset_odom", resetOdom);

    // Serial Port Settings
    string SensorPort;
    int SensorBaudrate;
    int async_output_rate;
    
    //int camera_interval;

    // Load all params
    pn.param<std::string>("frame_id", frame_id, "vectornav");
    pn.param<bool>("tf_ned_to_enu", tf_ned_to_enu, false);
    pn.param<int>("async_output_rate", async_output_rate, 40);
    pn.param<std::string>("serial_port", SensorPort, "/dev/ttyUSB0");
    pn.param<int>("serial_baud", SensorBaudrate, 115200);

    //Call to set covariances
    if(pn.getParam("linear_accel_covariance",rpc_temp))
    {
        linear_accel_covariance = setCov(rpc_temp);
    }
    if(pn.getParam("angular_vel_covariance",rpc_temp))
    {
        angular_vel_covariance = setCov(rpc_temp);
    }
    if(pn.getParam("orientation_covariance",rpc_temp))
    {
        orientation_covariance = setCov(rpc_temp);
    }
    
    //pn.getParam("interval_ms", camera_interval);
    
    pn.getParam("file_folder", file_folder);

    //ROS_INFO("Camera init.");
    if(wiringPiSetupPhys() == -1)
		exit(1);
    // camera context
    /*
    int retval;
    context = sample_create_context();

    // gp_log_add_func(GP_LOG_ERROR, errordumper, NULL);
    gp_camera_new(&camera);

    ROS_INFO("Camera init. Takes about 10 seconds.");

    retval = gp_camera_init(camera, context);
    if(retval != GP_OK){
        ROS_ERROR("Camera init fail!");
        exit(1);
    }
    */
    

    ROS_INFO("Connecting to : %s @ %d Baud", SensorPort.c_str(), SensorBaudrate);

    // Create a VnSensor object and connect to sensor
    VnSensor vs;

    // Default baudrate variable
    int defaultBaudrate;
    // Run through all of the acceptable baud rates until we are connected
    // Looping in case someone has changed the default
    bool baudSet = false;
    while(!baudSet){
        // Make this variable only accessible in the while loop
        static int i = 0;
        defaultBaudrate = vs.supportedBaudrates()[i];
        ROS_INFO("Connecting with default at %d", defaultBaudrate);
        // Default response was too low and retransmit time was too long by default.
        // They would cause errors
        vs.setResponseTimeoutMs(1000); // Wait for up to 1000 ms for response
        vs.setRetransmitDelayMs(50);  // Retransmit every 50 ms

        // Acceptable baud rates 9600, 19200, 38400, 57600, 128000, 115200, 230400, 460800, 921600
        // Data sheet says 128000 is a valid baud rate. It doesn't work with the VN100 so it is excluded.
        // All other values seem to work fine.
        try{
            // Connect to sensor at it's default rate
            if(defaultBaudrate != 128000 && SensorBaudrate != 128000)
            {
                vs.connect(SensorPort, defaultBaudrate);
                // Issues a change baudrate to the VectorNav sensor and then
                // reconnects the attached serial port at the new baudrate.
                vs.changeBaudRate(SensorBaudrate);
                // Only makes it here once we have the default correct
                ROS_INFO("Connected baud rate is %d",vs.baudrate());
                baudSet = true;
            }
        }
        // Catch all oddities
        catch(...){
            // Disconnect if we had the wrong default and we were connected
            vs.disconnect();
            ros::Duration(0.2).sleep();
        }
        // Increment the default iterator
        i++;
        // There are only 9 available data rates, if no connection
        // made yet possibly a hardware malfunction?
        if(i > 8)
        {
            break;
        }
    }

    // Now we verify connection (Should be good if we made it this far)
    if(vs.verifySensorConnectivity())
    {
        ROS_INFO("Device connection established");
    }else{
        ROS_ERROR("No device communication");
        ROS_WARN("Please input a valid baud rate. Valid are:");
        ROS_WARN("9600, 19200, 38400, 57600, 115200, 128000, 230400, 460800, 921600");
        ROS_WARN("With the test IMU 128000 did not work, all others worked fine.");
    }
    // Query the sensor's model number.
    string mn = vs.readModelNumber();
    ROS_INFO("Model Number: %s", mn.c_str());

    // Set Data output Freq [Hz]
    vs.writeAsyncDataOutputFrequency(async_output_rate);

    // Configure binary output message
    BinaryOutputRegister bor(
            ASYNCMODE_PORT1,
            1000 / async_output_rate,  // update rate [ms]
            COMMONGROUP_QUATERNION
            | COMMONGROUP_ANGULARRATE
            | COMMONGROUP_POSITION
            | COMMONGROUP_ACCEL
            | COMMONGROUP_MAGPRES
            | COMMONGROUP_TIMEGPS, // we want GPS time and week
            TIMEGROUP_TIMEGPS,  // we want GPS time and week
            IMUGROUP_NONE,
            GPSGROUP_NONE,
            ATTITUDEGROUP_YPRU, //<-- returning yaw pitch roll uncertainties
            INSGROUP_INSSTATUS
            | INSGROUP_POSLLA
            | INSGROUP_POSECEF
            | INSGROUP_VELBODY
            | INSGROUP_ACCELECEF,
    	    GPSGROUP_NONE);

    vs.writeBinaryOutput1(bor);
    
    // imu & gps writer initialization
    int write_time = (int)ros::Time::now().sec;
    ROS_INFO(" GPS, IMU and Picture info are saving in %s", file_folder.c_str());
    
    std::string imu_file_name = file_folder + "imu_" + std::to_string(write_time) + ".csv"; 
    std::string gps_file_name = file_folder + "gps_" + std::to_string(write_time) + ".csv";
    std::string img_file_name = file_folder + "img_stamp_" + std::to_string(write_time) + ".csv";
    imu_writer.open(imu_file_name);
    gps_writer.open(gps_file_name);
    img_stamp_writer.open(img_file_name);
    
    if(!imu_writer){
        ROS_WARN("IMU file error");
    }
    if(!gps_writer){
        ROS_WARN("GPS file error");
    }
    if(!img_stamp_writer){
        ROS_WARN("Picture info. file error");
    }
    
    
    
    // Set Data output Freq [Hz]
    vs.writeAsyncDataOutputFrequency(async_output_rate);
    vs.registerAsyncPacketReceivedHandler(NULL, BinaryAsyncMessageReceived);
    
    //camera_interval = camera_interval - DELAY_MS;
    //timer_start(takePicture, camera_interval);
    //timer_start(capture_to_file, camera_interval);
    //timer_start(takePicture, camera_interval);
    
    // You spin me right round, baby
    // Right round like a record, baby
    // Right round round round
    
    // counter : gps file format
    int counter = 0;
    pinMode(FCPIN, INPUT);
    bool flag = true;
    while (ros::ok())
    {
        ros::spinOnce();
        
        // fc signal
        //int fc_signal = digitalRead(FCPIN);
        if(digitalRead(FCPIN)==1 && flag){
            flag = false;
            ROS_INFO("Received signal from FC!!");
            counter += 1;
            if(counter ==1){
                imu_writer<<"time stamp, system time, acc_x, acc_y, acc_z, vel_x, vel_y, vel_z, roll, pitch, yaw"<< endl;
                gps_writer<<"type, id, gps time, gps week, latitude, longitude, altitude, relative altitude, gps altitude, roll, pitch, yaw"<<endl;
                img_stamp_writer<<"time stamp, system time"<<endl;
            }
            writeCSVFiles(counter);
            takePicture();
            delay(400); // trial and error
            // if camera triggering is slow, need to reduce delay value
            
        } // if(fc_signal==1 && flag)
        else{
            //ROS_INFO("Waiting for signal from FC!!!");
            flag = true;
            delay(1);
        }
         // Need to make sure we disconnect properly. Check if all ok.
    }

    // Node has been terminated
    vs.unregisterAsyncPacketReceivedHandler();
    ros::Duration(0.5).sleep();
    vs.disconnect();
    ros::Duration(0.5).sleep();
    imu_writer.close();
    gps_writer.close();
    img_stamp_writer.close();
    //gp_camera_exit(camera, context);
    return 0;
}

// imu and gps data to csv files *12/06/2019
void writeCSVFiles(int id)
{
    // std::string iso_time = boost::posix_time::to_iso_extended_string(msgIMU.header.stamp.toBoost());
    auto sys_time = std::chrono::system_clock::to_time_t(chrono::system_clock::now()); //std::chrono::system_clock::now();
    std::string sys_time_temp = std::ctime(&sys_time);
    std::string sys_time_str = sys_time_temp.substr(0, sys_time_temp.size()-1);
        
    // imu 
    imu_writer<< (int)ros::Time::now().sec; //(double)msgIMU.header.stamp.sec;
    imu_writer<< ",";
    imu_writer<< sys_time_str;
    imu_writer<<",";
    imu_writer<< (double)msgIMU.linear_acceleration.x; 
    imu_writer<< ",";
    imu_writer<< (double)msgIMU.linear_acceleration.y;
    imu_writer<< ","; 
    imu_writer<< (double)msgIMU.linear_acceleration.z;
    imu_writer<< ",";
    imu_writer<< (double)msgIMU.angular_velocity.x; 
    imu_writer<< ",";
    imu_writer<< (double)msgIMU.angular_velocity.y;
    imu_writer<< ",";
    imu_writer<< (double)msgIMU.angular_velocity.z;
    imu_writer<< ",";
    imu_writer<< (double)msgIMU.orientation.x;
    imu_writer<< ",";
    imu_writer<< (double)msgIMU.orientation.y; 
    imu_writer<< ",";
    imu_writer<< (double)msgIMU.orientation.z;
    imu_writer<<endl<<std::flush;
        
    // gps
    string gps_file_types[34] = {"BAT", "POWR", "MAG", "MAG2", "MAG3", "BARO", "BAR2", 
        "CTUN", "ATT", "RATE", "NKF1", "NKF2", "NKF3", "NKF4", "NKF5", "NKQ1", "NKF6", "NKF7", 
        "NKF8", "NKF9", "NKQ2", "AHR2", "POS", "MOTB", "RCIN", "RCOU", "PSC", "VIBE", "CTRL", "IMU", "IMU2", "IMU3", "GPS", "GPA"};
    for(int i=0; i<34; i++){
        gps_writer<<gps_file_types[i];
        gps_writer<<",";
        gps_writer<<id;
        gps_writer<<",0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0";
        gps_writer<<endl;
    }
    
    gps_writer<< msgkuGPS.type; 
    gps_writer<< ",";
    gps_writer<< id; 
    gps_writer<< ",";
    gps_writer<<(int)ros::Time::now().sec;
    //gps_writer<< (double)msgkuGPS.gps_time; 
    gps_writer<< ",";
    gps_writer<< (double)msgkuGPS.gps_week; 
    gps_writer<< ",";
    gps_writer<< (double)msgkuGPS.latitude; 
    gps_writer<< ",";
    gps_writer<< (double)msgkuGPS.longitude;
    gps_writer<< ",";
    gps_writer<< (double)msgkuGPS.altitude; 
    gps_writer<< ",";
    gps_writer<< (double)msgkuGPS.relative_altitude; 
    gps_writer<< ",";
    gps_writer<< (double)msgkuGPS.gps_altitude; 
    gps_writer<< ",";
    gps_writer<< (double)msgkuGPS.roll; 
    gps_writer<< ",";
    gps_writer<< (double)msgkuGPS.pitch; 
    gps_writer<< ",";
    gps_writer<< (double)msgkuGPS.yaw; 
    gps_writer<< endl<<std::flush;
}

//
// Callback function to process data packet from sensor
//
void BinaryAsyncMessageReceived(void* userData, Packet& p, size_t index)
{
    vn::sensors::CompositeData cd = vn::sensors::CompositeData::parse(p);

    // IMU
    //sensor_msgs::Imu msgIMU;
    msgIMU.header.stamp = ros::Time::now();
    msgIMU.header.frame_id = frame_id;
    

    if (cd.hasQuaternion() && cd.hasAngularRate() && cd.hasAcceleration())
    {

        vec4f q = cd.quaternion();
        vec3f ar = cd.angularRate();
        vec3f al = cd.acceleration();

        //Quaternion message comes in as a Yaw (z) Pitch (y) Roll (x) format
        if (tf_ned_to_enu)
        {
            // Flip x and y then invert z
            msgIMU.orientation.x = q[1];
            msgIMU.orientation.y = q[0];
            msgIMU.orientation.z = -q[2];
            msgIMU.orientation.w = q[3];

            if (cd.hasAttitudeUncertainty())
            {
                vec3f orientationStdDev = cd.attitudeUncertainty();
                msgIMU.orientation_covariance[0] = orientationStdDev[1]*orientationStdDev[1]*PI/180; // Convert to radians Pitch
                msgIMU.orientation_covariance[4] = orientationStdDev[0]*orientationStdDev[0]*PI/180; // Convert to radians Roll
                msgIMU.orientation_covariance[8] = orientationStdDev[2]*orientationStdDev[2]*PI/180; // Convert to radians Yaw
            }
            // Flip x and y then invert z
            msgIMU.angular_velocity.x = ar[1];
            msgIMU.angular_velocity.y = ar[0];
            msgIMU.angular_velocity.z = -ar[2];
            // Flip x and y then invert z
            msgIMU.linear_acceleration.x = al[1];
            msgIMU.linear_acceleration.y = al[0];
            msgIMU.linear_acceleration.z = -al[2];
        }
        else
        {
            msgIMU.orientation.x = q[0];
            msgIMU.orientation.y = q[1];
            msgIMU.orientation.z = q[2];
            msgIMU.orientation.w = q[3];

            if (cd.hasAttitudeUncertainty())
            {
                vec3f orientationStdDev = cd.attitudeUncertainty();
                msgIMU.orientation_covariance[0] = orientationStdDev[2]*orientationStdDev[2]*PI/180; // Convert to radians Roll
                msgIMU.orientation_covariance[4] = orientationStdDev[1]*orientationStdDev[1]*PI/180; // Convert to radians Pitch
                msgIMU.orientation_covariance[8] = orientationStdDev[0]*orientationStdDev[0]*PI/180; // Convert to radians Yaw
            }
            msgIMU.angular_velocity.x = ar[0];
            msgIMU.angular_velocity.y = ar[1];
            msgIMU.angular_velocity.z = ar[2];
            msgIMU.linear_acceleration.x = al[0];
            msgIMU.linear_acceleration.y = al[1];
            msgIMU.linear_acceleration.z = al[2];
        }
        // Covariances pulled from parameters
        msgIMU.angular_velocity_covariance = angular_vel_covariance;
        msgIMU.linear_acceleration_covariance = linear_accel_covariance;
        pubIMU.publish(msgIMU);
        
    }

    // Magnetic Field
    if (cd.hasMagnetic())
    {
        vec3f mag = cd.magnetic();
        sensor_msgs::MagneticField msgMag;
        msgMag.header.stamp = msgIMU.header.stamp;
        msgMag.header.frame_id = msgIMU.header.frame_id;
        msgMag.magnetic_field.x = mag[0];
        msgMag.magnetic_field.y = mag[1];
        msgMag.magnetic_field.z = mag[2];
        pubMag.publish(msgMag);
    }

    // GPS
    /*
    if(cd.hasTimeGps())
    {
        msgkuGPS.gps_time = cd.timeGps();
            //printf("GPS time:  %d\n", cd.timeGps());
    }

    
    if(cd.hasWeek())
    {
        msgkuGPS.gps_week = cd.week();
            //printf("GPS time week :  %d \n", cd.week());
    }
    //* */
    
    //printf("INS status %d \n", cd.hasInsStatus());
    //ROS_INFO("GPS data problem");
    if (cd.insStatus() == INSSTATUS_GPS_FIX) // || cd.insStatus() ==3)
    {
        //if(cd.hasInsStatus()){
        //    printf("\n %d vs %d \n", cd.insStatus(), INSSTATUS_GPS_FIX);
        //}
        vec3d lla = cd.positionEstimatedLla();

        sensor_msgs::NavSatFix msgGPS;
        msgGPS.header.stamp = msgIMU.header.stamp;
        msgGPS.header.frame_id = msgIMU.header.frame_id;
        msgGPS.latitude = lla[0];
        msgGPS.longitude = lla[1];
        msgGPS.altitude = lla[2];
        pubGPS.publish(msgGPS);
        
        // ku gps
        msgkuGPS.type = "CAM";
        msgkuGPS.roll = msgIMU.orientation.x;
        msgkuGPS.pitch = msgIMU.orientation.y;
        msgkuGPS.yaw = msgIMU.orientation.z;       
        
        msgkuGPS.latitude = lla[0];
        msgkuGPS.longitude = lla[1];
        msgkuGPS.altitude = 0;
        msgkuGPS.relative_altitude = lla[2];
        msgkuGPS.gps_altitude = 0;
        if(cd.hasTimeGps())
        {
            msgkuGPS.gps_time = cd.timeGps();
            //printf("GPS time:  %d\n", cd.timeGps());
        }
    
        if(cd.hasWeek())
        {
            msgkuGPS.gps_week = cd.week();
            //printf("GPS time week :  %d \n", cd.week());
        }
        pubKuGPS.publish(msgkuGPS);
        
        // Odometry
        if (pubOdom.getNumSubscribers() > 0)
        {
            nav_msgs::Odometry msgOdom;
            msgOdom.header.stamp = msgIMU.header.stamp;
            msgOdom.header.frame_id = msgIMU.header.frame_id;
            vec3d pos = cd.positionEstimatedEcef();

            if (!initial_position_set)
            {
                initial_position_set = true;
                initial_position.x = pos[0];
                initial_position.y = pos[1];
                initial_position.z = pos[2];
            }

            msgOdom.pose.pose.position.x = pos[0] - initial_position[0];
            msgOdom.pose.pose.position.y = pos[1] - initial_position[1];
            msgOdom.pose.pose.position.z = pos[2] - initial_position[2];

            if (cd.hasQuaternion())
            {
                vec4f q = cd.quaternion();

                msgOdom.pose.pose.orientation.x = q[0];
                msgOdom.pose.pose.orientation.y = q[1];
                msgOdom.pose.pose.orientation.z = q[2];
                msgOdom.pose.pose.orientation.w = q[2];
            }
            if (cd.hasVelocityEstimatedBody())
            {
                vec3f vel = cd.velocityEstimatedBody();

                msgOdom.twist.twist.linear.x = vel[0];
                msgOdom.twist.twist.linear.y = vel[1];
                msgOdom.twist.twist.linear.z = vel[2];
            }
            if (cd.hasAngularRate())
            {
                vec3f ar = cd.angularRate();

                msgOdom.twist.twist.angular.x = ar[0];
                msgOdom.twist.twist.angular.y = ar[1];
                msgOdom.twist.twist.angular.z = ar[2];
            }
            pubOdom.publish(msgOdom);
        }
    }

    else
    {
        msgkuGPS.type = "CAM";
        msgkuGPS.roll = msgIMU.orientation.x;
        msgkuGPS.pitch = msgIMU.orientation.y;
        msgkuGPS.yaw = msgIMU.orientation.z;       
        
        msgkuGPS.latitude = 0;
        msgkuGPS.longitude = 0;
        msgkuGPS.altitude = 0;
        msgkuGPS.relative_altitude = 0;
        msgkuGPS.gps_altitude = 0;
        if(cd.hasTimeGps())
        {
            msgkuGPS.gps_time = cd.timeGps();
            //printf("GPS time:  %d\n", cd.timeGps());
        }
    
        if(cd.hasWeek())
        {
            msgkuGPS.gps_week = cd.week();
            //printf("GPS time week :  %d \n", cd.week());
        }
        pubKuGPS.publish(msgkuGPS);
    }
    // Temperature
    if (cd.hasTemperature())
    {
        float temp = cd.temperature();

        sensor_msgs::Temperature msgTemp;
        msgTemp.header.stamp = msgIMU.header.stamp;
        msgTemp.header.frame_id = msgIMU.header.frame_id;
        msgTemp.temperature = temp;
        pubTemp.publish(msgTemp);
    }

    // Barometer
    if (cd.hasPressure())
    {
        float pres = cd.pressure();

        sensor_msgs::FluidPressure msgPres;
        msgPres.header.stamp = msgIMU.header.stamp;
        msgPres.header.frame_id = msgIMU.header.frame_id;
        msgPres.fluid_pressure = pres;
        pubPres.publish(msgPres);
    }
}


void takePicture(){
    auto sys_time = std::chrono::system_clock::to_time_t(chrono::system_clock::now()); //std::chrono::system_clock::now();
    std::string sys_time_temp = std::ctime(&sys_time);
    std::string sys_time_str = sys_time_temp.substr(0, sys_time_temp.size()-1);
    
	pinMode(CAMERAPIN, OUTPUT);
	//softPwmCreate(CAMERAPIN, 0, 100);
	digitalWrite(CAMERAPIN, HIGH);
	delay(200);
	digitalWrite(CAMERAPIN, LOW);
	
    
    //img_stamp_writer<<(double)sys_time;
    img_stamp_writer<<(int)ros::Time::now().sec;
    img_stamp_writer<<",";
    img_stamp_writer<<sys_time_str;
    img_stamp_writer<<endl;
    delay(400);
}

// timer to capture image in regular interval
void timer_start(std::function<void(void)> func, unsigned int interval){
	std::thread([func, interval](){
		while(true){
			func();
			std::this_thread::sleep_for(std::chrono::milliseconds(interval));
		}
	}).detach();
}

/*
// capture image and save it in specified folder yaml with name as sec sequence
// use gphoto2 command
void takePicture_gphoto(){
	ROS_INFO("capturing image");
	std::string time_sec = to_string(ros::Time::now().sec);
	std::string gphoto_command = "gphoto2 --capture-image-and-download --filename " + image_folder + time_sec + ".arw"; //infinity frames 30s interval
	system(gphoto_command.c_str());
}

// ref https://github.com/gphoto/libgphoto2/blob/master/examples/sample-capture.c
// capture image and save it in specified folder yaml with name as sec sequence
// use libgphoto library
void capture_to_file(){
    ROS_INFO("capturing image");

    int fd, retval;
    CameraFile *file;
    CameraFilePath camera_file_path;
    char *data;
    unsigned long size;

    std::string time_sec = to_string(ros::Time::now().sec) + ".arw";


    strcpy(camera_file_path.folder,"/");
    strcpy(camera_file_path.name, time_sec.c_str());

    retval = gp_camera_capture(camera, GP_CAPTURE_IMAGE, &camera_file_path, context);
    retval = gp_file_new(&file);

    retval = gp_camera_file_get(camera, camera_file_path.folder, camera_file_path.name, GP_FILE_TYPE_NORMAL, file, context);
    gp_file_get_data_and_size (file, (const char**)&data, &size);

    FILE *f;

    std::string fn = image_folder+time_sec; //+".arw";

    f = fopen(fn.c_str(), "wb");
    if(f){
        retval = fwrite(data, size, 1, f);
        if(retval != size){
            printf("  fwrite size %ld, written %d\n", size, retval);
        }
        fclose(f);
    }
    else
        printf("  fopen foo2.jpg failed.\n");

    gp_file_free(file);
    retval = gp_camera_file_delete(camera, camera_file_path.folder, camera_file_path.name, context);
}

GPContext* sample_create_context() {
    GPContext *context;

    // This is the mandatory part 
    context = gp_context_new();

    // All the parts below are optional! 
        gp_context_set_error_func (context, ctx_error_func, NULL);
        gp_context_set_status_func (context, ctx_status_func, NULL);

    // also:
    //gp_context_set_cancel_func    (p->context, ctx_cancel_func,  p);
        //gp_context_set_message_func   (p->context, ctx_message_func, p);
        //if (isatty (STDOUT_FILENO))
                //gp_context_set_progress_funcs (p->context, ctx_progress_start_func, ctx_progress_update_func,ctx_progress_stop_func, p);
    return context;
}
*/
