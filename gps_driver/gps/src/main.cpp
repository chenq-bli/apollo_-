#include <ros/ros.h>
#include <memory>
#include "gps/gps_driver.hpp"

int main(int argc,char ** argv)
{
    ros::init(argc,argv,"gps_driver_node");
    ros::NodeHandle nh("~");

    std::shared_ptr<GPSDriver> ptr_=std::make_shared<GPSDriver>(nh);
    ros::spin();
    
    return 0;

}