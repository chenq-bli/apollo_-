#include "ros/ros.h"
#include "def_gpsmsg/gps.h"

using namespace std;

void gpsCallback(const def_gpsmsg::gps::ConstPtr& msg)
{
  cout<< stod(msg->Lattitude) <<endl;
  cout<< stod(msg->Longitude) <<endl;
  cout<< stod(msg->Altitude) <<endl;
}

int main(int argc, char **argv) 
{

  ros::init(argc, argv, "listener");
  ros::NodeHandle nh; 
  ros::Subscriber sub = nh.subscribe("talker/gps_date", 10, &gpsCallback); 
  ros::spin();
  return 0;
}