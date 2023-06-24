#include "ros/ros.h"
#include <iostream>
#include <ctime>
#include <memory>
#include <thread>
#include <string>
#include "def_gpsmsg/gps.h"

using namespace std;

void publichdate(def_gpsmsg::gps& msg)
{
    msg.Header = "Header_name";
    msg.GPSWeek = "GPSWeek_name";
    msg.GPSTime = "GPSTime_name";
    msg.Heading = "Heading";
    msg.Pitch = "Pitch";
    msg.Roll = "Roll";
    msg.Lattitude = "100.99";
    msg.Longitude = "128.32";
    msg.Altitude = "11.34";
    msg.Ve = "Ve";
    msg.Vn = "Vn";
    msg.Vu = "Vu";
    msg.Baseline = "Baseline";
    msg.NSV1 = "NSV1";
    msg.NSV2 = "NSV2";
    msg.Status = "Status";
    msg.Cs = "Cs";
    msg.CrLf = "CrLf";
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle nh("~");
    ros::Publisher gps_pub = nh.advertise<def_gpsmsg::gps>("gps_date", 1000);
    ros::Rate rate(100);
    def_gpsmsg::gps gpsmsgs;
    publichdate(gpsmsgs);
    while(ros::ok())
    {
        gps_pub.publish(gpsmsgs);
        rate.sleep();
        

    }

    ros::spin();
    
    return 0;
}
