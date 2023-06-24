/***********************************************************/
/*
    Function:ROS parsing GPS agreement(GPFPD)，Post to topic.
    Author:YuTeng
    Date:2019/10/14
    Version: V1.0
*/
/***********************************************************/
#include "ros/ros.h"
#include "gps/gps.h"
#include <serial/serial.h>
#include <iostream>
#include <string>
#include <ctime>
using namespace std; 
int GPS_Data_Check(uint8_t a[],int length);
int GPS_Data_Check(string hex_n,int dex);
int main(int argc,char ** argv)
{
    ros::init(argc,argv,"talker"); //解析参数，命名节点
    ros::NodeHandle nh; //创建句柄，实例化node
    gps::gps msg; //创建gps消息
    serial::Serial sp;
    ros::Publisher pub = nh.advertise<gps::gps>("gps_info",1); //创建publisherW  
    ros::Rate loop_rate(1.0); //定义循环发布的频率
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    //设置要打开的串口名称
    sp.setPort("/dev/rtk_ch340");
    //设置串口通信的波特率
    sp.setBaudrate(115200);
    //窗口设置timeout
    sp.setTimeout(to);
    try
    {
         //打开串口
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    //判断串口是否打开成功
    if(sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    }
    else
    {
        return -1;
    }
    
    clock_t startTime,endTime;    
    while(ros::ok())
    {
        startTime = clock();//计时开始
        size_t n = sp.available();
        string str;
        string Header;
        string GPSWeek;
        string GPSTime;
        string Heading;
        string Pitch;
        string Roll;
        string Lattitude;
        string Longitude;
        string Altitude;
        string Ve;
        string Vn;
        string Vu;
        string Baseline;
        string NSV1;
        string NSV2;
        string Status;
        string Cs;
        string CrLf;
        uint8_t buffer[1024];
        int num=0;
        int flag=0;
        n = sp.read(buffer, n);
        if(n>0)
        {
            if(GPS_Data_Check(buffer,n)==0)//如果校验不通过则跳过
            {
                continue;
            }
            for(int i=0; i<n; i++)
            {
                if(buffer[i]==',')
                {
                    num++;
                }
                else 
                {
                    if(num==0)
                    {
                        Header+=buffer[i];   
                    }
                    else if(num==1)
                    {
                        GPSWeek+=buffer[i];
                    }
                    else if(num==2)
                    {
                        GPSTime+=buffer[i];
                    }
                    else if(num==3)
                    {
                        Heading+=buffer[i];
                    }
                    else if(num==4)
                    {
                        Pitch+=buffer[i];
                    }
                    else if(num==5)
                    {
                        Roll+=buffer[i];
                    }
                    else if(num==6)
                    {
                        Lattitude+=buffer[i];
                    }
                    else if(num==7)
                    {
                        Longitude+=buffer[i];
                    }
                    else if(num==8)
                    {
                        Altitude+=buffer[i];
                    }
                    else if(num==9)
                    {
                        Ve+=buffer[i];
                    }
                    else if(num==10)
                    {
                        Vn+=buffer[i];
                    }
                    else if(num==11)
                    {
                        Vu+=buffer[i];
                    }
                    else if(num==12)
                    {
                        Baseline+=buffer[i];
                    }
                    else if(num==13)
                    {
                        NSV1+=buffer[i];
                    }
                    else if(num==14)
                    {
                        NSV2+=buffer[i];
                    }
                    else if(num==15)
                    {
                        if(flag<2)
                        {
                            Status+=buffer[i];
                        }
                        else if(flag>=2&&flag<5)
                        {
                            Cs+=buffer[i];
                        }
                        else if(flag>=5)
                        {
                            CrLf+=buffer[i];
                        }
                        flag++;
                    }
                }
                str+=buffer[i];
            }
            msg.Header=Header;
            msg.GPSWeek=GPSWeek;
            msg.GPSTime=GPSTime;
            msg.Heading=Heading;
            msg.Pitch=Pitch;
            msg.Roll=Roll;
            msg.Lattitude=Lattitude;
            msg.Longitude=Longitude;
            msg.Altitude=Altitude;
            msg.Ve=Ve;
            msg.Vn=Vn;
            msg.Vu=Vu;
            msg.Baseline=Baseline;
            msg.NSV1=NSV1;
            msg.NSV2=NSV2;
            msg.Status=Status;
            msg.Cs=Cs;
            msg.CrLf=CrLf;
            std::cout <<"GPS:   "<<str<< std::endl;
            ROS_INFO("Header:%s",msg.Header.c_str());
            ROS_INFO("GPSWeek:%s",msg.GPSWeek.c_str());
            ROS_INFO("GPSTime:%s",msg.GPSTime.c_str());
            ROS_INFO("Heading:%s",msg.Heading.c_str());
            ROS_INFO("Pitch:%s",msg.Pitch.c_str());
            ROS_INFO("Roll:%s",msg.Roll.c_str());
            ROS_INFO("Lattitude:%s",msg.Lattitude.c_str());
            ROS_INFO("Longitude:%s",msg.Longitude.c_str());
            ROS_INFO("Altitude:%s",msg.Altitude.c_str());
            ROS_INFO("Ve:%s",msg.Ve.c_str());
            ROS_INFO("Vn:%s",msg.Vn.c_str());
            ROS_INFO("Vu:%s",msg.Vu.c_str());
            ROS_INFO("Baseline:%s",msg.Baseline.c_str());
            ROS_INFO("NSV1:%s",msg.NSV1.c_str());
            ROS_INFO("NSV2:%s",msg.NSV2.c_str());
            ROS_INFO("Status:%s",msg.Status.c_str());
            ROS_INFO("Cs:%s",msg.Cs.c_str());
            ROS_INFO("CrLf:%s",msg.CrLf.c_str());
            pub.publish(msg); //发布消息
            sp.write(buffer, n);
            endTime = clock();//计时结束
            std::cout << "The run time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << std::endl;
        }
        n=0;
        loop_rate.sleep(); //根据定义的发布频率，sleep
    }
    sp.close();
    return 0;
}
int GPS_Data_Check(uint8_t a[],int length)//GPS数据校验
{
    int head_n=6;
    string head_str; //帧头
    string CS_str; //数据帧校验位
    int flag=1; //数据标志位
    int CS_dex; 
    unsigned char CS=0;
    int chang=0;
    for(int i=0;i<length-2;i++)
    {
        if(i<head_n)
        {
            head_str+=a[i];
        }
        if(flag==3)
        {
            CS_str+=a[i];//在一帧数据中本来存在的校验位
        }
        if(a[i]=='*')
        {
            flag=3;
        }
        if(flag==2)
        {
            CS^=a[i];//异或出来的校验位
            chang++;
        }
        if(a[i]=='$')
        {
            flag=2;
        }
    }
    CS_dex=CS;  //十进制ASCLL码
    if((head_str=="$GPFPD")&&(GPS_Data_Check(CS_str,CS_dex)))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
int GPS_Data_Check(string hex_n,int dex)
{
    string dex_con_hex;
    string temp;
    int j=0;
    if(dex<=15)
    {
        string temp="";
        if(dex<10)
        {
            temp+=dex+'0';
        }
        else
        {
            temp+=dex-10+'A';
        }
        dex_con_hex=temp;
        for(int i=temp.length()-1;i>=0;i--)
        {
            dex_con_hex[j]=temp[i];
            j++;
        }
    }
    else
    {
        string temp="";
        do
        {
            int dex_tmp=dex%16;
            if(dex_tmp<10)
            {
                temp+=dex_tmp+'0';
            }
            else
            {
                temp+=dex_tmp-10+'A';
            }
            dex/=16;
        }while(dex>=16);
        if(dex<10)
        {
            temp+=dex+'0';
        }
        else
        {
            temp+=dex-10+'A';
        }
        dex_con_hex=temp;
        for(int i=temp.length()-1;i>=0;i--)
        {
            dex_con_hex[j]=temp[i];
            j++;
        }
    }
    if(dex_con_hex.compare(hex_n))//如果两个字符串相等则为0
    {
        return 0;
    }
    else
    {
        return 1;
        
    }
}
