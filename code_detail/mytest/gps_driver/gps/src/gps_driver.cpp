#include "gps/gps_driver.hpp"

GPSDriver::GPSDriver(ros::NodeHandle &nh) : nh_(nh), thread_flag_(false)
{
    if (!init())
    {
        nh_.shutdown();
        ros::shutdown();
    }

    this->create_publisher();

    std::thread driver_thread(boost::bind(&GPSDriver::DriverThread, this));
    driver_thread.detach();
}

GPSDriver::~GPSDriver()
{
    thread_flag_ = true;
}

bool GPSDriver::init()
{
    nh_.param<std::string>("gps_frame_id", gps_frame_id_, "gps");
    nh_.param<std::string>("driver_port", driver_port_, "/dev/rtk_ch340");
    nh_.param<int>("driver_baud", driver_baud_, 230400);
    serial::Timeout timeout_ = serial::Timeout::simpleTimeout(100);
    sp_.setPort(driver_port_.c_str());
    sp_.setBaudrate(driver_baud_);
    sp_.setTimeout(timeout_);
    try
    {
        sp_.open();
        if (sp_.isOpen())
        {
            std::string stemp;
            stemp = driver_port_ + "is opened.";
            ROS_INFO_STREAM(stemp);
        }
        else
        {
            return false;
        }
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port_micheal Chen.");
        return false;
    }
    return true;
}

void GPSDriver::create_publisher()
{
    gnss_publisher_ = nh_.advertise<sensor_msgs::NavSatFix>("gps/fix", 100);
    vel_publisher_ = nh_.advertise<geometry_msgs::TwistStamped>("/gps/vel", 1000);
    gpsimu_publisher_ = nh_.advertise<geometry_msgs::Quaternion>("/gps/imu", 100);
}

void GPSDriver::DriverThread()
{
    std::string deal_date = "";
    while (true)
    {
        if (thread_flag_)
            break;

        auto feasiable_bytes = sp_.available();
        std::string str_received;
        if (feasiable_bytes)
        {
            sp_.read(str_received, feasiable_bytes);
            if (str_received.find("$GPFPD") == 0)
            {
                if (deal_date.size() != 0)
                {
                    DealRawData(deal_date);
                    deal_date = "";
                    deal_date += str_received;
                }
                else
                {
                    deal_date = deal_date + str_received;
                }
            }
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

void GPSDriver::DealRawData(std::string &data)
{
    data.erase(data.begin() + data.size() - 1, data.end());
    data[data.size() - 1] = ',';

    std::string buff_regex = data;
    std::regex pattern("([a-zA-Z0-9.-/*/$]+),");
    std::smatch capture;
    buf_.clear(); // 必须清空

    for (std::sregex_iterator it(buff_regex.begin(), buff_regex.end(), pattern), end_it; it != end_it; ++it)
    {
        std::string mstem = it->str();
        std::regex_search(mstem, capture, pattern);
        buf_.push_back(std::string(capture[1]));
    }
    for (int i = 0; i < buf_.size(); i++)
    {
        std::cout << buf_[i] << " ";
    }
    std::cout << std::endl;

    Publish();
}

void GPSDriver::Publish()
{
    std::vector<std::string> parse_data = buf_;
    if (parse_data.size() == 16)
    {
        sensor_msgs::NavSatFix navsatfix_msg;
        geometry_msgs::TwistStamped twist_msg;
        geometry_msgs::Quaternion gps_imu_msg;

        ros::Time now_time = ros::Time::now();
        navsatfix_msg.header.stamp = now_time;
        navsatfix_msg.header.frame_id = "gps";
        navsatfix_msg.latitude = std::stof(parse_data[6]);
        navsatfix_msg.longitude = std::stof(parse_data[7]);
        navsatfix_msg.altitude = std::stof(parse_data[8]);

        twist_msg.header.stamp = now_time;
        twist_msg.header.frame_id = "gps";
        twist_msg.twist.linear.x = std::stof(parse_data[9]);
        twist_msg.twist.linear.y = std::stof(parse_data[10]);
        twist_msg.twist.linear.z = std::stof(parse_data[11]);
        twist_msg.twist.angular.x = 0;
        twist_msg.twist.angular.y = 0;
        twist_msg.twist.angular.z = 0;

        static bool init_flag = true;
        static float yaw_history = 0;
        static ros::Time history_time;
        if (init_flag == true)
        {
            init_flag = false;
            yaw_history = std::stof(parse_data[10]);
            twist_msg.twist.angular.z = 0;
        }
        else
        {
            float delta_time = (now_time - history_time).toSec();
            float delta_yaw = yaw_history - std::stof(parse_data[10]);
            if (delta_yaw < 30.0 && delta_yaw > -30.0)
                twist_msg.twist.angular.z = (yaw_history - std::stof(parse_data[10])) * ROS_PI / 180.00 / delta_time;
            else if (delta_yaw >= 30)
            {
                twist_msg.twist.angular.z = (yaw_history - std::stof(parse_data[10]) - 360.0) * ROS_PI / 180.00 / delta_time;
            }
            else if (delta_yaw <= -30)
            {
                twist_msg.twist.angular.z = (yaw_history - std::stof(parse_data[10]) + 360.0) * ROS_PI / 180.00 / delta_time;
            }
        }
        static double hangx;
        hangx = std::stof(parse_data[3]);
        if (hangx < 180 && hangx >= 0)
        {
            hangx = -hangx * 3.1415926 / 180;
        }
        else
        {
            hangx = (360 - hangx) * 3.1415926 / 180;
        }
        Eigen::Quaternionf rot_q = FromEuler2Quaternion(hangx, 0, 0);
        gps_imu_msg.w = rot_q.w();
        gps_imu_msg.x = rot_q.x();
        gps_imu_msg.y = rot_q.y();
        gps_imu_msg.z = rot_q.z();

        if (parse_data[15].find("4B") == 0 || parse_data[15].find("5B") == 0)
        {
            Publish(navsatfix_msg, twist_msg, gps_imu_msg);
            history_time = now_time;
        }
        else
        {
            SendFalseData();
        }
        yaw_history = std::stof(parse_data[10]);
    }

    else
    {
        SendFalseData();
    }
}

Eigen::Quaternionf GPSDriver::FromEuler2Quaternion(const float &yaw, const float &pitch, const float &roll)
{

    Eigen::Quaternionf q = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());

    return q;
}

void GPSDriver::Publish(const sensor_msgs::NavSatFix &navsatfix_msg, const geometry_msgs::TwistStamped &twist_msg, const geometry_msgs::Quaternion &gps_imu_msg)
{
    gnss_publisher_.publish(navsatfix_msg);
    vel_publisher_.publish(twist_msg);
    gpsimu_publisher_.publish(gps_imu_msg);
}

void GPSDriver::SendFalseData()
{
    std::vector<std::string> parse_data = buf_;
    for (int i = 0; i < 19; i++)
    {
        parse_data.push_back("0");
    }

    if (parse_data.size() != 0)
    {
        sensor_msgs::NavSatFix navsatfix_msg;
        geometry_msgs::TwistStamped twist_msg;
        geometry_msgs::Quaternion gps_imu_msg;
        ros::Time now_time = ros::Time::now();
        // navsatfix_msg 填充
        navsatfix_msg.header.stamp = now_time;
        navsatfix_msg.header.frame_id = "gps";
        navsatfix_msg.latitude = std::stof(parse_data[2]);
        navsatfix_msg.longitude = std::stof(parse_data[3]);
        navsatfix_msg.altitude = std::stof(parse_data[4]);
        // twist_msg 填充
        twist_msg.header.stamp = now_time;
        twist_msg.header.frame_id = "gps";
        twist_msg.twist.linear.x = std::stof(parse_data[5]);
        twist_msg.twist.linear.y = std::stof(parse_data[6]);
        twist_msg.twist.linear.z = std::stof(parse_data[7]);
        twist_msg.twist.angular.x = 0;
        twist_msg.twist.angular.y = 0;
        static bool init_flag = true;
        static float yaw_history = 0;
        static ros::Time history_time;
        if (init_flag == true)
        {
            init_flag = false;
            yaw_history = std::stof(parse_data[10]);
            twist_msg.twist.angular.z = 0;
        }
        else
        {
            float delta_time = (now_time - history_time).toSec();
            float delta_yaw = yaw_history - std::stof(parse_data[10]);
            if (delta_yaw < 30.0 && delta_yaw > -30.0)
                twist_msg.twist.angular.z = (yaw_history - std::stof(parse_data[10])) * ROS_PI / 180.00 / delta_time;
            else if (delta_yaw >= 30)
            {
                twist_msg.twist.angular.z = (yaw_history - std::stof(parse_data[10]) - 360.0) * ROS_PI / 180.00 / delta_time;
            }
            else if (delta_yaw <= -30)
            {
                twist_msg.twist.angular.z = (yaw_history - std::stof(parse_data[10]) + 360.0) * ROS_PI / 180.00 / delta_time;
            }
        }
        // gps_imu_msg 填充，如果信号不好，全部设置为-1
        gps_imu_msg.x = -1;
        gps_imu_msg.y = -1;
        gps_imu_msg.z = -1;
        gps_imu_msg.w = -1;
        Publish(navsatfix_msg, twist_msg, gps_imu_msg);
        history_time = now_time;
        yaw_history = std::stof(parse_data[10]);
    }
}