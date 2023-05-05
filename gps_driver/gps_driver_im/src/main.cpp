#include <ros/ros.h>
#include <memory>
#include "gps_driver/gps_driver_parse.hpp"

int main(int argc,char ** argv) {

    ros::init(argc, argv, "gps_driver_node");
    ros::NodeHandle nh("~");

    std::shared_ptr<GPSDriverParse> ptr_ = std::make_shared<GPSDriverParse>(nh);//创建GPSDriverParse对象，并利用智能指针指向，将句柄引用传入。
    //构造函数内部创建了新的线程来解析数据。
    ros::spin();

    return 0;
}