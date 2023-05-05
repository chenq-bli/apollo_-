#include <Eigen/Dense>
#include "gps_driver/gps_driver_parse.hpp"


GPSDriverParse::GPSDriverParse(ros::NodeHandle& nh) : nh_(nh), thread_flag_(false) {
    if(!Init()){//初始化
        nh_.shutdown();
        ros::shutdown();
        
    }//如果初始化，则关闭
    

    ROSRegister();//创建话题发布器

    std::thread driver_thread(boost::bind(&GPSDriverParse::DriverThread, this));//创建一个驱动器线程，并把当前对象作为值传入
    driver_thread.detach();//分离线程，该线程资源被释放，无法被自动回收，使其在后台运行而不阻塞主线程。
}

GPSDriverParse::~GPSDriverParse(){
    thread_flag_ = true;
}


bool GPSDriverParse::Init(){//为类成员进行基本填充
    //参数服务器获取参数
    nh_.param<std::string>("gps_frame_id",gps_frame_id_, "gps");
    nh_.param<std::string>("driver_port", driver_port_,  "/dev/rtk_ch340");
    // nh_.param<std::string>("driver_port", driver_port_,  "/dev/ttyUSB0");
    nh_.param<int>("driver_baud", driver_baud_,  230400);
    
    //创建超时对象，用于设置串口通信过程中的超时参数，例如读取数据时的超时时间，如果在指定的时间内没有读取到数据，则会触发超时错误。
    serial::Timeout timeout_ = serial::Timeout::simpleTimeout(100);//simpleTimeout是serial空间下Timeout结构体静态成员函数

    sp_.setPort(driver_port_.c_str());   //设置串口通信的波特率,并将 driver_port_ 字符串转换为 C 风格的字符串
    sp_.setBaudrate(driver_baud_);
    sp_.setTimeout(timeout_);            //超时触发
    try{
        sp_.open();                      //打开串口
        if(sp_.isOpen())
            ROS_INFO_STREAM("/dev/rtk_ch340 is opened.");
        else
            return false;
    }
    catch(serial::IOException& e){//捕获可能抛出的 serial::IOException 异常。如果在尝试打开串口设备时发生异常，程序将跳转到 catch 块，并执行相应的异常处理代码。
        ROS_ERROR_STREAM("Unable to open port.");
        return false;
    }

    return true;
}

Eigen::Quaternionf GPSDriverParse::FromEuler2Quaternion(const float& yaw, const float& pitch, const float& roll){
    Eigen::Quaternionf q = Eigen::AngleAxisf(yaw,   Eigen::Vector3f::UnitZ())//Eigen::AngleAxisf为旋转向量类型
                           * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
                           * Eigen::AngleAxisf(roll,  Eigen::Vector3f::UnitX());//新的Eigen::AngleAxisf量自动被转换为对应的 Quaternionf 类型的对象 q，表示绕一个新的轴旋转的旋转操作。
    return q;
}

void GPSDriverParse::ROSRegister(){
    //创建/gps/fix话题发布器，两个参数分别是话题名称和队列大小，队列大小为1是指队列中最多只能保存一个待发布的消息，当队列中已经有一个消息时，如果有新的消息要发布，
    //那么新的消息会覆盖队列中原有的消息，然后被发布出去。
    //队列太大不会使得订阅的消息不是最新的，因为在 ROS 中订阅者会从发布者的队列中取出最新的消息，取得都是最新传入的，但是设置太大会浪费内存，太小也不行，如果满了，发布者的最新消息无法进入队列，会丢失最新数据，直到消息队列有空余。
    gnss_publisher_ = nh_.advertise<sensor_msgs::NavSatFix>("/gps/fix",1);
    vel_publisher_  = nh_.advertise<geometry_msgs::TwistStamped>("/gps/vel",1000);
    gpsimu_publisher_= nh_.advertise<geometry_msgs::Quaternion>("/gps/imu",1);
}

void GPSDriverParse::DriverThread(){
    std::string deal_data = "";//储存
    while(true){//循环必须的
        if(thread_flag_) break;
        auto available_bytes = sp_.available();//获取串口中可用的字节数
        std::string str_received;
        if (available_bytes){
            sp_.read(str_received, available_bytes);//然后读取这些字节并存储到字符串 str_received 中
             if(!str_received.find("$GPFPD")){//如果读取到的字符串中包含 "$GPFPD"，则表示读取到了一条完整的 GPS 数据，需要对这条数据进行处理。
                if(deal_data.size() != 0){
                    DealRawData(deal_data);
                }
                deal_data = "";
                deal_data += str_received;
             }
             else
                deal_data += str_received;//如果读取到的字符串中不包含 "$GPFPD"，则表示这条数据还不完整，需要将其与之前读取到的数据拼接起来，直到读取到完整的一条数据为止。
        }
        else
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void GPSDriverParse::DealRawData(std::string& data){

    // data += ",";
    //这两行代码的作用是将字符串 data 转换为以逗号结尾的格式，以便后续的处理或输出
    data.erase(data.begin()+data.size()-1, data.end());//删除字符串 data 的最后一个字符
    data[data.size()-1] = ',';//最后一个字符赋值为','，这里是为了后面使用正则化来处理。
    // if(!data.find("$GPFPD"))
    //     return;
    // std::cout << data << std::endl;

    std::string buff_regex = data;
    std::string pattern = "([a-zA-Z0-9.-/*/$]+),";
    std::regex r(pattern);//定义正则化规则，用于匹配以逗号结尾的字符串，字符串以字母、数字、点号、减号、斜杠、星号和美元符号组成子串。()是进行捕获
    std::smatch mas;
    std::vector<std::string> buf;
    //std::sregex_iterator it(buff_regex.begin(), buff_regex.end(), r)作用是：遍历一个字符串 buff_regex 中所有匹配正则表达式 r 的子串，并将它们保存到一个迭代器 it 中
    for(std::sregex_iterator it(buff_regex.begin(), buff_regex.end(), r), end_it; it != end_it; ++it){
        std::string msg = it->str();
        std::regex_search(msg,mas,r);//std::regex_search与std::regex_match不同的是 前者只需msg的子串可以匹配r即可，后者需要满足整个msg完全匹配r才可。
        buf.push_back(std::string(mas[1]));//每次提取第一个子匹配项，并放入buf
    }
   for(int i = 0; i < buf.size(); i++){
       std::cout << buf[i] << " ";//循环输出buf
   }
   std::cout << std::endl;
    Publish(buf);//获取每个部分后，然后进行数据处理，发布所需话题
}

void GPSDriverParse::Publish(const std::vector<std::string>& buf){

    auto parse_data = buf;//复制数据给parse_data

    if(buf.size() == 16 ){//按理来说是16个字节
        sensor_msgs::NavSatFix navsatfix_msg;
        geometry_msgs::TwistStamped twist_msg;
        ros::Time now_time = ros::Time::now();
        //navsatfix_msg 填充
        navsatfix_msg.header.stamp = now_time;
        navsatfix_msg.header.frame_id = "gps";
        navsatfix_msg.latitude  = std::stof(parse_data[6]);//将字符串转化为单精度浮点型，填充纬度
        navsatfix_msg.longitude = std::stof(parse_data[7]);
        navsatfix_msg.altitude =  std::stof(parse_data[8]);
        //twist_msg 填充
        twist_msg.header.stamp = now_time;
        twist_msg.header.frame_id = "gps";
        twist_msg.twist.linear.x = std::stof(parse_data[9]);
        twist_msg.twist.linear.y = std::stof(parse_data[10]);
        twist_msg.twist.linear.z = std::stof(parse_data[11]);
        twist_msg.twist.angular.x = 0;
        twist_msg.twist.angular.y = 0;
        twist_msg.twist.angular.z = 0;
        static bool init_flag = true;//静态成员，只会被初始化一次
        static float yaw_history = 0;
        static ros::Time history_time;
        if(init_flag == true){//静态成员，只会被初始化一次，意味着这里只会进入一次
            init_flag = false;
            yaw_history = std::stof(parse_data[10]);
            twist_msg.twist.angular.z = 0;
        }
        else{//第二次循环以后，init_flag一直是false，
            float delta_time = (now_time - history_time).toSec();
            float delta_yaw  = yaw_history - std::stof(parse_data[10]);
            if(delta_yaw < 30.0 && delta_yaw > -30.0)
                twist_msg.twist.angular.z = (yaw_history - std::stof(parse_data[10])) * ROS_PI / 180.00 / delta_time;
            else if(delta_yaw >= 30){
                twist_msg.twist.angular.z = (yaw_history - std::stof(parse_data[10]) - 360.0) * ROS_PI / 180.00 / delta_time;
            }
            else if(delta_yaw <= -30){
                twist_msg.twist.angular.z = (yaw_history - std::stof(parse_data[10]) + 360.0) * ROS_PI / 180.00 / delta_time;
            }
        }

        geometry_msgs::Quaternion gps_imu_msg;//创建一个四元素对象
        //gps_imu_msg 填充
        static double hangx;//静态变量，设置为静态变量可以，但是不能每次都初始化，可赋值。
        hangx=std::stof(parse_data[3]);//第三个部分是横向角
        if(hangx<180 && hangx>=0)
        {
          hangx=-hangx*3.1415926/180;//转化为-pi/2到0
        }
        else
        {
          hangx=(360-hangx)*3.1415926/180;//转化为0到pi/2
        }
        Eigen::Quaternionf rot_q = FromEuler2Quaternion(hangx, 0, 0);//欧拉角转化为4元素
        gps_imu_msg.x = rot_q.x();
        gps_imu_msg.y = rot_q.y();
        gps_imu_msg.z = rot_q.z();
        gps_imu_msg.w = rot_q.w();


        //std::cout << parse_data[15].find("4B") << ", " << parse_data[15] << std::endl;
        /*if(parse_data[15].find("4B") == 0 || parse_data[15].find("5B") == 0|| parse_data[15].find("04") == 0)*/
        if(parse_data[15].find("4B") == 0 || parse_data[15].find("5B") == 0){//find()返回值是子串在原串的位置，这里有的话会返回0，不然会返回td::string::npos。
            this->Publish(navsatfix_msg, twist_msg, gps_imu_msg);//真正发布，引用传递
            history_time = now_time;
        }
        else{//如果信号不好，全部设置为-1.
            SendFalseData();
        }
        yaw_history = std::stof(parse_data[10]);
    }
    else{
        SendFalseData();
    }
}

void GPSDriverParse::SendFalseData(){
    std::vector<std::string> parse_data;
    for(int i = 0; i < 19; i++){
        parse_data.push_back("0");
    }
    if(parse_data.size() !=0 ){
        sensor_msgs::NavSatFix navsatfix_msg;
        geometry_msgs::TwistStamped twist_msg;
        geometry_msgs::Quaternion gps_imu_msg;
        ros::Time now_time = ros::Time::now();
        //navsatfix_msg 填充
        navsatfix_msg.header.stamp = now_time;
        navsatfix_msg.header.frame_id = "gps";
        navsatfix_msg.latitude  = std::stof(parse_data[2]);
        navsatfix_msg.longitude = std::stof(parse_data[3]);
        navsatfix_msg.altitude =  std::stof(parse_data[4]);
        //twist_msg 填充
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
        if(init_flag == true){
            init_flag = false;
            yaw_history = std::stof(parse_data[10]);
            twist_msg.twist.angular.z = 0;
        }
        else{
            float delta_time = (now_time - history_time).toSec();
            float delta_yaw  = yaw_history - std::stof(parse_data[10]);
            if(delta_yaw < 30.0 && delta_yaw > -30.0)
                twist_msg.twist.angular.z = (yaw_history - std::stof(parse_data[10])) * ROS_PI / 180.00 / delta_time;
            else if(delta_yaw >= 30){
                twist_msg.twist.angular.z = (yaw_history - std::stof(parse_data[10]) - 360.0) * ROS_PI / 180.00 / delta_time;
            }
            else if(delta_yaw <= -30){
                twist_msg.twist.angular.z = (yaw_history - std::stof(parse_data[10]) + 360.0) * ROS_PI / 180.00 / delta_time;
            }
        }
        //gps_imu_msg 填充，如果信号不好，全部设置为-1
        gps_imu_msg.x = -1;
        gps_imu_msg.y = -1;
        gps_imu_msg.z = -1;
        gps_imu_msg.w = -1;
        this->Publish(navsatfix_msg,twist_msg,gps_imu_msg);
        history_time = now_time;
        yaw_history = std::stof(parse_data[10]);
    }
}

void GPSDriverParse::Publish(const sensor_msgs::NavSatFix& navsatfix_msg, const geometry_msgs::TwistStamped& twist_msg, const geometry_msgs::Quaternion& gps_imu_msg){
    gnss_publisher_.publish(navsatfix_msg);
    vel_publisher_.publish(twist_msg);
    gpsimu_publisher_.publish(gps_imu_msg);
}