#include "ros/ros.h"
#include "gps/gps.h"
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/NavSatFix.h>
using namespace std;

double Latt;
double lon;
double alt;
double powff(float x, int n);
double Str2Num(char *pStr);
int state;



double powff(float x, int n)  //  x的n次方
{
  double result = 1;
 
while(n--)
{
   result *= x;
}
return result;
}


double Str2Num(char *pStr)  // 字符串转double型
{
  double  Num = 0;
  unsigned char BitBehidePoint = 0, leng=0, PointCount = 0;
  while(pStr[leng] != '\0')  // 遍历
  {
     if ((pStr[leng] >= '0' && pStr[leng] <= '9') || pStr[leng] == '.') // 合法字符判定
     {
      if(pStr[leng] != '.' && !PointCount )      // 整数部分处理
      {
        Num = Num*10 + (pStr[leng]-'0');
      }
      else if(PointCount )  // 小数部分处理
      {
        BitBehidePoint ++;
        Num += (pStr[leng] - '0') * powff(0.1, BitBehidePoint);
      }
      if(pStr[leng] == '.') // 计数.的数量，若大于1个则为非法字符串
      {
        PointCount ++;
        if(PointCount  >= 2)
        {
          ROS_INFO("More point error");
          return 0;
        }
      }
}
else // 字符串中含有非法字符
{
  ROS_INFO(" error sysm type");
  return 0;
}
leng ++;  // 计算字符串长度
  }
  return Num;
}


void gpsCallback(const gps::gps::ConstPtr &msg)
{
  


  Latt=Str2Num((char *)msg->Lattitude.c_str());
  lon=Str2Num((char *)msg->Longitude.c_str());
  alt=Str2Num((char *)msg->Altitude.c_str());
  state=(int)Str2Num((char *)msg->Status.c_str());



  ROS_INFO("Header:%s",msg->Header.c_str());
  ROS_INFO("GPSWeek:%s",msg->GPSWeek.c_str());
  ROS_INFO("GPSTime:%s",msg->GPSTime.c_str());
  ROS_INFO("Heading:%s",msg->Heading.c_str());
  ROS_INFO("Pitch:%s",msg->Pitch.c_str());
  ROS_INFO("Roll:%s",msg->Roll.c_str());
  ROS_INFO("Lattitude:%lf",Latt);
  ROS_INFO("Longitude:%lf",lon);
  ROS_INFO("Altitude:%lf",alt);
  ROS_INFO("Ve:%s",msg->Ve.c_str());
  ROS_INFO("Vn:%s",msg->Vn.c_str());
  ROS_INFO("Vu:%s",msg->Vu.c_str());
  ROS_INFO("Baseline:%s",msg->Baseline.c_str());
  ROS_INFO("NSV1:%s",msg->NSV1.c_str());
  ROS_INFO("NSV2:%s",msg->NSV2.c_str());
  ROS_INFO("Status:%d",state);
  ROS_INFO("Cs:%s",msg->Cs.c_str());
  ROS_INFO("CrLf:%s",msg->CrLf.c_str());
}

int main(int argc, char **argv) 
{
   
  ros::init(argc, argv, "fix_topic_publisher"); //
  ros::NodeHandle n;        
  ros::Publisher fix_pub = n.advertise<sensor_msgs::NavSatFix>("fix",1); 
  ros::Subscriber sub = n.subscribe("gps_info", 1, gpsCallback); //创建subscriber
  
  while(ros::ok)
  {
  sensor_msgs::NavSatFix fix_data;
  fix_data.header.stamp=ros::Time::now();
  fix_data.header.frame_id ="base_link";
  fix_data.latitude=Latt;
  fix_data.longitude=lon;
  fix_data.altitude= alt;
  fix_pub.publish(fix_data);

  ros::spinOnce();  //The loop waits for the callback function //循环等待回调函数

  }

  
  return 0;
}
