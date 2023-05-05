#ifndef GPS_DRIVER_GPS_DRIVER
#define GPS_DRIVER_GPS_DRIVER

#pragma once
#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <serial/serial.h>
#include <thread>
#include <mutex>
#include <memory>
#include <boost/bind.hpp>
#include <regex>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>

#define ROS_PI 3.1415926535

class GPSDriver
{
public:
    GPSDriver(ros::NodeHandle &nh);
    ~GPSDriver();

private:
    bool init();
    void create_publisher();
    void DriverThread();
    void DealRawData(std::string &data);
    void Publish();
    Eigen::Quaternionf FromEuler2Quaternion(const float &yaw, const float &pitch, const float &roll);
    void Publish(const sensor_msgs::NavSatFix &navsatfix_msg, const geometry_msgs::TwistStamped &twist_msg, const geometry_msgs::Quaternion &gps_imu_msg);
    void SendFalseData();

private:
    ros::NodeHandle nh_;
    bool thread_flag_;
    std::string gps_frame_id_;
    std::string driver_port_;
    int driver_baud_;
    serial::Serial sp_;
    std::vector<std::string> buf_;

    ros::Publisher gnss_publisher_;
    ros::Publisher vel_publisher_;
    ros::Publisher gpsimu_publisher_;
};

#endif