#ifndef GPS_DRIVER_GPS_DRIVER_PARSE_HPP //如果这个头文件没有定义过，则执行#ifndef 和#endif之间内容
#define GPS_DRIVER_GPS_DRIVER_PARSE_HPP

#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <string>
#include <ctime>
#include <thread>
#include <mutex>
#include <regex>// 正则化
#include <boost/bind.hpp>
#include <Eigen/Dense>

#define ROS_PI 3.1415926535

class GPSDriverParse{
public:
    GPSDriverParse(ros::NodeHandle& nh);
    ~GPSDriverParse();

private:
    bool Init();
    void ROSRegister();
    void DriverThread();
    void Publish(const std::vector<std::string>& buf);
    void Publish(const sensor_msgs::NavSatFix& navsatfix_msg, const geometry_msgs::TwistStamped& twist_msg, const geometry_msgs::Quaternion& gps_imu_msg);
    void SendFalseData();
    void DealRawData(std::string& data);
    Eigen::Quaternionf FromEuler2Quaternion(const float& yaw, const float& pitch, const float& roll);

private:
    ros::NodeHandle nh_;
    ros::Publisher  gnss_publisher_;
    ros::Publisher  vel_publisher_;
    ros::Publisher  gpsimu_publisher_;

    std::string     driver_port_;
    std::string     gps_frame_id_;
    int             driver_baud_;

    serial::Serial  sp_;//创建串口对象
    bool            thread_flag_;

    std::string     gnss_read_data_;
    std::mutex      mtx_;
};

#endif