/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/control/controller/lat_controller.h"

#include <algorithm>
#include <iomanip>
#include <utility>
#include <vector>

#include "Eigen/LU"
#include "absl/strings/str_cat.h"

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/math/linear_quadratic_regulator.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/quaternion.h"
#include "modules/control/common/control_gflags.h"

namespace apollo
{
  namespace control
  {

    using apollo::common::ErrorCode;
    using apollo::common::Status;
    using apollo::common::TrajectoryPoint;
    using apollo::common::VehicleStateProvider;
    using Matrix = Eigen::MatrixXd;
    using apollo::cyber::Clock;

    namespace
    {

      std::string GetLogFileName()
      {
        time_t raw_time;
        char name_buffer[80];
        std::time(&raw_time);
        std::tm time_tm;
        localtime_r(&raw_time, &time_tm);
        strftime(name_buffer, 80, "/tmp/steer_log_simple_optimal_%F_%H%M%S.csv",
                 &time_tm);
        return std::string(name_buffer);
      }

      void WriteHeaders(std::ofstream &file_stream)
      {
        file_stream << "current_lateral_error,"
                    << "current_ref_heading,"
                    << "current_heading,"
                    << "current_heading_error,"
                    << "heading_error_rate,"
                    << "lateral_error_rate,"
                    << "current_curvature,"
                    << "steer_angle,"
                    << "steer_angle_feedforward,"
                    << "steer_angle_lateral_contribution,"
                    << "steer_angle_lateral_rate_contribution,"
                    << "steer_angle_heading_contribution,"
                    << "steer_angle_heading_rate_contribution,"
                    << "steer_angle_feedback,"
                    << "steering_position,"
                    << "v" << std::endl;
      }
    } // namespace

    LatController::LatController() : name_("LQR-based Lateral Controller")
    {
      if (FLAGS_enable_csv_debug)
      {
        steer_log_file_.open(GetLogFileName());
        steer_log_file_ << std::fixed;
        steer_log_file_ << std::setprecision(6);
        WriteHeaders(steer_log_file_);
      }
      AINFO << "Using " << name_;
    }

    LatController::~LatController() { CloseLogFile(); }

    bool LatController::LoadControlConf(const ControlConf *control_conf)
    {
      if (!control_conf)
      {
        AERROR << "[LatController] control_conf == nullptr";
        return false;
      }
      // LatController类内成员车辆参数配置vehicle_param_加载车辆参数配置
      vehicle_param_ =
          common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param();
      // LatController类内成员控制周期ts_加载纵向控制配置中的控制周期control_conf.pb.txt--lat_controller_conf--ts
      ts_ = control_conf->lat_controller_conf().ts();
      if (ts_ <= 0.0)
      {
        AERROR << "[MPCController] Invalid control update interval.";
        return false;
      }
      // 将控制配置中的前轮侧偏刚度(左右轮之和)cf读取到LatController类数据成员cf_
      cf_ = control_conf->lat_controller_conf().cf();
      cr_ = control_conf->lat_controller_conf().cr();
      // 将控制配置中的预览窗口大小读取到数据成员preview_window_
      preview_window_ = control_conf->lat_controller_conf().preview_window();
      // 将控制配置的低速预瞄距离读取到LatController类内数据成员lookahead_station_low_speed_
      lookahead_station_low_speed_ =
          control_conf->lat_controller_conf().lookahead_station();
      // lookback是倒车时的预瞄距离
      lookback_station_low_speed_ =
          control_conf->lat_controller_conf().lookback_station();
      // 将控制配置的高速预瞄距离读取到LatController类内数据成员lookahead_station_high_speed_
      lookahead_station_high_speed_ =
          control_conf->lat_controller_conf().lookahead_station_high_speed();
      // lookback是倒车时的预瞄距离
      lookback_station_high_speed_ =
          control_conf->lat_controller_conf().lookback_station_high_speed();
      // 从车辆参数配置vehicle_param_读取轴距到LatController类数据成员wheelbase_（L）
      wheelbase_ = vehicle_param_.wheel_base();
      // 转向传动比 = 方向盘转角/前轮转角
      steer_ratio_ = vehicle_param_.steer_ratio();
      // 从车辆参数配置vehicle_param_读取单边方向盘最大转角转化成deg到LatController类数据成员
      // LatController类数据成员steer_single_direction_max_degree_
      steer_single_direction_max_degree_ =
          vehicle_param_.max_steer_angle() / M_PI * 180;
      max_lat_acc_ = control_conf->lat_controller_conf().max_lateral_acceleration();
      // 从控制配置control_conf_加载低高速边界到LatController类数据成员low_speed_bound_
      low_speed_bound_ = control_conf_->lon_controller_conf().switch_speed();
      // 这个窗口主要应用在低高速切换之间的线性插值，凡是涉及低高速控制切换的，就在这个窗口做线性插值过渡低高速的控制参数
      low_speed_window_ =
          control_conf_->lon_controller_conf().switch_speed_window();

      // 从控制配置control_conf加载左前悬的质量到临时常量变量mass_fl
      const double mass_fl = control_conf->lat_controller_conf().mass_fl();
      const double mass_fr = control_conf->lat_controller_conf().mass_fr();
      const double mass_rl = control_conf->lat_controller_conf().mass_rl();
      const double mass_rr = control_conf->lat_controller_conf().mass_rr();
      const double mass_front = mass_fl + mass_fr;
      const double mass_rear = mass_rl + mass_rr;
      mass_ = mass_front + mass_rear;
      // 计算 前悬长度(前轴到车辆质心的距离) = 轴距 * (1 - 前悬质量 / 车辆总质量) 计算结果存放到LatController类的数据成员 lf_
      lf_ = wheelbase_ * (1.0 - mass_front / mass_);
      // 计算 后悬长度(后轴到车辆质心的距离) = 轴距 * (1 - 后悬质量 / 车辆总质量) 计算结果存放到LatController类的数据成员 lr_
      lr_ = wheelbase_ * (1.0 - mass_rear / mass_);

      // moment of inertia 计算 车辆绕z轴的转动惯量(又叫惯性矩) m*r^2
      iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;

      // 从控制配置control_conf_加载LQR迭代求解精度到LatController类数据成员lqr_eps_
      lqr_eps_ = control_conf->lat_controller_conf().eps();
      // 从控制配置control_conf_加载LQR迭代求解最大次数到LatController类数据成员lqr_max_iteration_
      lqr_max_iteration_ = control_conf->lat_controller_conf().max_iteration();

      // 若打开query_time_nearest_point_only模式，则会用到此参数，但是默认关闭
      // 从控制配置control_conf_加载查询相对时间到LatController类数据成员query_relative_time_
      // 若打开此种模式，则 目标点选为 当前时间+query_relative_time_ 这个时间在参考轨迹上对应的点
      // query_relative_time默认设置为0.8s，若打开此种模式就默认始终用将来0.8s的轨迹点作为目标点驱动当前车产生控制量向前走
      query_relative_time_ = control_conf->query_relative_time();

      // 从控制配置control_conf_加载最小速度保护到LatController类数据成员minimum_speed_protection_
      // minimum_speed_protection的作用可以这样理解，因为车辆状态方程系数矩阵A(离散形式Ad)中均包含v作为分母的元素
      // 若v=0，那么计算控制量时出现0出现在分母的情况，所以在控制配置里设置一个最小速度保护值
      // 在更新系数矩阵A,Ad时，若v<最小保护速度，则v取最小保护速度，避免出现v=0作为分母的情况
      minimum_speed_protection_ = control_conf->minimum_speed_protection();

      return true;
    }

    void LatController::ProcessLogs(const SimpleLateralDebug *debug,
                                    const canbus::Chassis *chassis)
    {
      const std::string log_str = absl::StrCat(
          debug->lateral_error(), ",", debug->ref_heading(), ",", debug->heading(),
          ",", debug->heading_error(), ",", debug->heading_error_rate(), ",",
          debug->lateral_error_rate(), ",", debug->curvature(), ",",
          debug->steer_angle(), ",", debug->steer_angle_feedforward(), ",",
          debug->steer_angle_lateral_contribution(), ",",
          debug->steer_angle_lateral_rate_contribution(), ",",
          debug->steer_angle_heading_contribution(), ",",
          debug->steer_angle_heading_rate_contribution(), ",",
          debug->steer_angle_feedback(), ",", chassis->steering_percentage(), ",",
          injector_->vehicle_state()->linear_velocity());
      if (FLAGS_enable_csv_debug)
      {
        steer_log_file_ << log_str << std::endl;
      }
      ADEBUG << "Steer_Control_Detail: " << log_str;
    }

    void LatController::LogInitParameters()
    {
      AINFO << name_ << " begin.";
      AINFO << "[LatController parameters]"
            << " mass_: " << mass_ << ","
            << " iz_: " << iz_ << ","
            << " lf_: " << lf_ << ","
            << " lr_: " << lr_;
    }

    void LatController::InitializeFilters(const ControlConf *control_conf)
    {
      // Low pass filter
      std::vector<double> den(3, 0.0);
      std::vector<double> num(3, 0.0);
      common::LpfCoefficients(
          ts_, control_conf->lat_controller_conf().cutoff_freq(), &den, &num);
      digital_filter_.set_coefficients(den, num);
      lateral_error_filter_ = common::MeanFilter(static_cast<std::uint_fast8_t>(
          control_conf->lat_controller_conf().mean_filter_window_size()));
      heading_error_filter_ = common::MeanFilter(static_cast<std::uint_fast8_t>(
          control_conf->lat_controller_conf().mean_filter_window_size()));
    }

    Status LatController::Init(std::shared_ptr<DependencyInjector> injector,
                               const ControlConf *control_conf)
    {
      // 将控制器参数配置和车辆状态信息读取到LatController类数据成员control_conf_,injector_里
      // 这样先初始化赋值后，类内其他成员函数可以直接访问控制配置和车辆参数
      control_conf_ = control_conf;
      injector_ = injector;

      // 在if判断条件里调用布尔类型的加载控制配置的函数LoadControlConf()，将控制配置对象control_conf_中的参数再依次加载到
      // LatController类的各个数据成员上
      if (!LoadControlConf(control_conf_))
      {
        AERROR << "failed to load control conf";
        return Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                      "failed to load control_conf");
      }

      // 矩阵初始化操作
      // 车辆状态方程 X = AX + B*delta前轮 + B1*Psi_des'   Psi_des是道路曲率变化率即期望的航向角变化率
      // 车辆状态方程中矩阵的大小 = 基础状态矩阵大小 + 预览窗口大小
      // 基础状态矩阵就是X=[e1 e1' e2 e2']^T   ^T表示转置
      // e1横向误差，e2航向误差
      // 基础状态矩阵大小basic_state_size_已经在声明时被初始化为4
      // 横向控制预览窗口是0，所以matrix_size就是basic_state_size_=4
      const int matrix_size = basic_state_size_ + preview_window_;
      // 初始化系数矩阵A为4*4的零矩阵
      matrix_a_ = Matrix::Zero(basic_state_size_, basic_state_size_);

      // 车辆状态方程是连续形式，计算机控制需要转换成离散的差分方程形式
      // matrix_ad_是系数矩阵A的离散形式，A通过双线性变化法变成Ad
      // 初始化系数矩阵Ad为4*4的零矩阵
      matrix_ad_ = Matrix::Zero(basic_state_size_, basic_state_size_);

      // matrix_adc_是Ad Composed  就是由矩阵Ad和预览窗口一起扩展而成的矩阵
      // 横向控制预览窗口是为0默认关闭preview，所以matrix_adc_就是matrix_ad_,所以这里也初始化为4*4零矩阵
      matrix_adc_ = Matrix::Zero(matrix_size, matrix_size);

      /*
      A matrix (Gear Drive) 系数矩阵A
      [0.0, 1.0, 0.0, 0.0;
       0.0, (-(c_f + c_r) / m) / v, (c_f + c_r) / m,
       (l_r * c_r - l_f * c_f) / m / v;
       0.0, 0.0, 0.0, 1.0;
       0.0, ((lr * cr - lf * cf) / i_z) / v, (l_f * c_f - l_r * c_r) / i_z,
       (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
      */
      // 给矩阵A中的常数项(与v无关的项)进行赋值，A中常数项只有下面4个吗？
      // 不是，因为矩阵A matrix_a_被初始化为4*4的零矩阵，0元素项不用再赋值
      // A矩阵中的非常数项在其他函数中每时刻更新
      // A矩阵第1行第2列是常数1.0
      matrix_a_(0, 1) = 1.0;
      matrix_a_(1, 2) = (cf_ + cr_) / mass_;
      matrix_a_(2, 3) = 1.0;
      matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;

      // matrix_a_coeff_矩阵也初始化为4*4的零矩阵
      matrix_a_coeff_ = Matrix::Zero(matrix_size, matrix_size);

      // A矩阵中跟v相关的时变项，这些时变项分解成"常数项/v"的形式，然后提取出这个与v无关的常数项
      // 放在矩阵matrix_a_coeff_的对应位置(例如若在A中是(1,1),在matrix_a_coeff_也放在(1,1))

      matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
      matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
      matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
      matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;

      /*
      b = [0.0, c_f / m, 0.0, l_f * c_f / i_z]^T
      */
      // 初始化矩阵B为4*1的零矩阵
      matrix_b_ = Matrix::Zero(basic_state_size_, 1);
      // 矩阵B的离散化形式Bd初始化为4*1的0矩阵
      matrix_bd_ = Matrix::Zero(basic_state_size_, 1);
      // bdc就是bd copmposed 就是bd和预览窗口一起组合起来扩展成新的矩阵bdc
      // 但是横向控制中preview窗口大小为0，所以预览功能相当于关闭，那么matrix_bdc_就是matrix_bd_
      matrix_bdc_ = Matrix::Zero(matrix_size, 1);

      matrix_b_(1, 0) = cf_ / mass_;
      matrix_b_(3, 0) = lf_ * cf_ / iz_;
      // 矩阵B的离散形式Bd就等于 B * ts
      matrix_bd_ = matrix_b_ * ts_;
      // 车辆状态矩阵X,[e1 e1' e2 e2']
      // 车辆状态矩阵X初始化为4*1的0矩阵
      matrix_state_ = Matrix::Zero(matrix_size, 1);
      // 初始化K矩阵为1*4的0矩阵
      matrix_k_ = Matrix::Zero(1, matrix_size);

      // 初始化R矩阵为1*1的单位阵，R矩阵就是LQR中目标函数中控制量平方和的权重系数
      // 横向控制只有一个控制量就是前轮转角
      matrix_r_ = Matrix::Identity(1, 1);

      // 初始化Q矩阵为4*4的0矩阵，Q矩阵是LQR中目标函数中各个状态量(X=[e1 e1' e2 e2'])平方和的权重系数
      // Q是一个对角阵，对角线上4个元素分别存放e1 e1' e2 e2'平方和在LQR目标函数中的权重系数，这里只是初始化一下
      matrix_q_ = Matrix::Zero(matrix_size, matrix_size);

      // 定义临时变量q_param_size=control_conf控制配置里的matrix_q_size()，
      // 但是在control_conf.pb.txt里并没有发现matrix_q_size这个参数，但是有matrix_q是repeated数组元素
      // 这个就是Protobuf的用法，XXX_size()就是返回repeated类型数组XXX元素的个数
      // matrix_q在控制配置中有4项，就是Q矩阵对角线上的4个元素，对应车辆各个状态量在LQR目标函数中的权重系数
      // q_param_size默认为4

      int q_param_size = control_conf_->lat_controller_conf().matrix_q_size();
      // 倒车的reverse_q_param_size就是配置文件中reverse_matrix_q元素个数
      int reverse_q_param_size =
          control_conf_->lat_controller_conf().reverse_matrix_q_size();

      if (matrix_size != q_param_size || matrix_size != reverse_q_param_size)
      { // 维度不同报错
        const auto error_msg = absl::StrCat(
            "lateral controller error: matrix_q size: ", q_param_size,
            "lateral controller error: reverse_matrix_q size: ",
            reverse_q_param_size,
            " in parameter file not equal to matrix_size: ", matrix_size);
        AERROR << error_msg;
        return Status(ErrorCode::CONTROL_COMPUTE_ERROR, error_msg);
      }
      // 填充权重矩阵q
      for (int i = 0; i < q_param_size; ++i)
      {
        matrix_q_(i, i) = control_conf_->lat_controller_conf().matrix_q(i);
      }

      matrix_q_updated_ = matrix_q_;

      // 用LatController类数据成员控制配置control_conf_去初始化滤波器
      // 初始化3个滤波器，1个低通滤波是对计算出方向盘转角控制指令进行滤波
      // 另外两个滤波器是横向误差，航向误差的均值滤波器
      InitializeFilters(control_conf_);

      // 定义1个临时变量lat_controller_conf,从控制配置control_conf_将
      // control_conf_里的lat_controller_conf读取到临时变量lat_controller_conf
      // lat_controller_conf实际上就是控制配置中的横向控制配置
      auto &lat_controller_conf = control_conf_->lat_controller_conf();

      // LoadLatGainScheduler加载增益调度表，就是横向误差和航向误差在车速不同时乘上个不同的比例
      // 这个比例决定了实际时的控制效果，根据实际经验低速和高速应该采取不同的比例，低速比例较大，若高速
      // 采用同样比例极有可能导致画龙现象，问题来了，对于一辆给定的车，争议调度表如何确定？
      LoadLatGainScheduler(lat_controller_conf);

      // 这个函数主要是在屏幕上打印一些车辆参数的信息
      LogInitParameters();

      // 是否使能超前滞后控制器从control_conf_里的lat_controller_conf里的enable_reverse_leadlag_compensation参数读取
      // 默认是开启横向控制中的超前滞后控制器的，提升或者降低闭环反馈系统的响应速度
      enable_leadlag_ = control_conf_->lat_controller_conf()
                            .enable_reverse_leadlag_compensation();
      // 若打开，用控制配置里的lat_controller_conf里reverse_leadlag_conf里的参数以及控制周期ts_去设置超前滞后控制器
      if (enable_leadlag_)
      {
        leadlag_controller_.Init(lat_controller_conf.reverse_leadlag_conf(), ts_);
      }
      // 是否使能mrac模型参考自适应控制enable_mrac_从control_conf里的lat_controller_conf里的enable_steer_mrac_control读取
      enable_mrac_ =
          control_conf_->lat_controller_conf().enable_steer_mrac_control();
      // 默认关闭，直接跳过
      if (enable_mrac_)
      {
        mrac_controller_.Init(lat_controller_conf.steer_mrac_conf(),
                              vehicle_param_.steering_latency_param(), ts_);
      }

      // 是否使能前进倒车时的预瞄控制enable_look_ahead_back_control_
      // 这个使能开关从控制配置control_conf_里的lat_controller_conf里的enable_look_ahead_back_control参数读取
      // 默认打开
      // lookahead前进预瞄，lookback倒车预瞄
      enable_look_ahead_back_control_ =
          control_conf_->lat_controller_conf().enable_look_ahead_back_control();

      return Status::OK();
    }

    void LatController::CloseLogFile()
    {
      if (FLAGS_enable_csv_debug && steer_log_file_.is_open())
      {
        steer_log_file_.close();
      }
    }

    void LatController::LoadLatGainScheduler(
        const LatControllerConf &lat_controller_conf)
    {
      const auto &lat_err_gain_scheduler =
          lat_controller_conf.lat_err_gain_scheduler();
      const auto &heading_err_gain_scheduler =
          lat_controller_conf.heading_err_gain_scheduler();
      AINFO << "Lateral control gain scheduler loaded";
      Interpolation1D::DataType xy1, xy2;
      for (const auto &scheduler : lat_err_gain_scheduler.scheduler())
      {
        xy1.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
      }
      for (const auto &scheduler : heading_err_gain_scheduler.scheduler())
      {
        xy2.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
      }

      lat_err_interpolation_.reset(new Interpolation1D);
      ACHECK(lat_err_interpolation_->Init(xy1))
          << "Fail to load lateral error gain scheduler";

      heading_err_interpolation_.reset(new Interpolation1D);
      ACHECK(heading_err_interpolation_->Init(xy2))
          << "Fail to load heading error gain scheduler";
    }

    void LatController::Stop() { CloseLogFile(); }

    std::string LatController::Name() const { return name_; }

    Status LatController::ComputeControlCommand(
        const localization::LocalizationEstimate *localization,
        const canbus::Chassis *chassis,
        const planning::ADCTrajectory *planning_published_trajectory,
        ControlCommand *cmd)
    {
      // 通过LatController类数据成员injector_获取车辆当前状态信息放到临时变量vehicle_state里
      // vehicle_state的类型用auto根据等式右边自动指定
      auto vehicle_state = injector_->vehicle_state();

      // 获取车辆的期望轨迹存放到临时变量target_tracking_trajectory
      auto target_tracking_trajectory = *planning_published_trajectory;

      // FLAGS_use_navigation_mode去modules/common/configs/config_gflags.cc取出
      // 取出use_navigation_mode的值，是否打开导航模式，默认是false关闭的这一段直接跳过
      if (FLAGS_use_navigation_mode &&
          FLAGS_enable_navigation_mode_position_update)
      {
        auto time_stamp_diff =
            planning_published_trajectory->header().timestamp_sec() -
            current_trajectory_timestamp_;

        auto curr_vehicle_x = localization->pose().position().x();
        auto curr_vehicle_y = localization->pose().position().y();

        double curr_vehicle_heading = 0.0;
        const auto &orientation = localization->pose().orientation();
        if (localization->pose().has_heading())
        {
          curr_vehicle_heading = localization->pose().heading();
        }
        else
        {
          curr_vehicle_heading =
              common::math::QuaternionToHeading(orientation.qw(), orientation.qx(),
                                                orientation.qy(), orientation.qz());
        }

        // new planning trajectory
        if (time_stamp_diff > 1.0e-6)
        {
          init_vehicle_x_ = curr_vehicle_x;
          init_vehicle_y_ = curr_vehicle_y;
          init_vehicle_heading_ = curr_vehicle_heading;

          current_trajectory_timestamp_ =
              planning_published_trajectory->header().timestamp_sec();
        }
        else
        {
          auto x_diff_map = curr_vehicle_x - init_vehicle_x_;
          auto y_diff_map = curr_vehicle_y - init_vehicle_y_;
          auto theta_diff = curr_vehicle_heading - init_vehicle_heading_;

          auto cos_map_veh = std::cos(init_vehicle_heading_);
          auto sin_map_veh = std::sin(init_vehicle_heading_);

          auto x_diff_veh = cos_map_veh * x_diff_map + sin_map_veh * y_diff_map;
          auto y_diff_veh = -sin_map_veh * x_diff_map + cos_map_veh * y_diff_map;

          auto cos_theta_diff = std::cos(-theta_diff);
          auto sin_theta_diff = std::sin(-theta_diff);

          auto tx = -(cos_theta_diff * x_diff_veh - sin_theta_diff * y_diff_veh);
          auto ty = -(sin_theta_diff * x_diff_veh + cos_theta_diff * y_diff_veh);

          auto ptr_trajectory_points =
              target_tracking_trajectory.mutable_trajectory_point();
          std::for_each(
              ptr_trajectory_points->begin(), ptr_trajectory_points->end(),
              [&cos_theta_diff, &sin_theta_diff, &tx, &ty,
               &theta_diff](common::TrajectoryPoint &p)
              {
                auto x = p.path_point().x();
                auto y = p.path_point().y();
                auto theta = p.path_point().theta();

                auto x_new = cos_theta_diff * x - sin_theta_diff * y + tx;
                auto y_new = sin_theta_diff * x + cos_theta_diff * y + ty;
                auto theta_new = common::math::NormalizeAngle(theta - theta_diff);

                p.mutable_path_point()->set_x(x_new);
                p.mutable_path_point()->set_y(y_new);
                p.mutable_path_point()->set_theta(theta_new);
              });
        }
      }

      // target_tracking_trajectory是从输入参数传进来的规划轨迹信息
      // 将target_tracking_trajectory对象内容move移动到LatController类数据成员trajectory_analyzer_里

      // 先构造，再用move将其左值，返回其右值（内容）给到trajectory_analyzer_
      trajectory_analyzer_ =
          std::move(TrajectoryAnalyzer(&target_tracking_trajectory));

      // 将规划轨迹从后轴中心变换到质心，如果条件满足的话
      // FLAGS_trajectory_transform_to_com_reverse也是从modules/control/common/control_gflags.cc取出
      // 取出trajectory_transform_to_com_reverse值，倒档是否需要转换到质心坐标的开关,默认false
      // 这个if不满足，可以略过
      //  Transform the coordinate of the planning trajectory from the center of the
      //  rear-axis to the center of mass, if conditions matched
      if (((FLAGS_trajectory_transform_to_com_reverse &&
            vehicle_state->gear() == canbus::Chassis::GEAR_REVERSE) ||
           (FLAGS_trajectory_transform_to_com_drive &&
            vehicle_state->gear() == canbus::Chassis::GEAR_DRIVE)) &&
          enable_look_ahead_back_control_)
      {
        trajectory_analyzer_.TrajectoryTransformToCOM(lr_);
      }

      // Re-build the vehicle dynamic models at reverse driving (in particular,
      // replace the lateral translational motion dynamics with the corresponding
      // kinematic models)倒档时重建车辆的动力学模型，尤其时横向的动力学模型
      if (vehicle_state->gear() == canbus::Chassis::GEAR_REVERSE)
      {
        /*
        A matrix (Gear Reverse)
        [0.0, 0.0, 1.0 * v 0.0;
         0.0, (-(c_f + c_r) / m) / v, (c_f + c_r) / m,
         (l_r * c_r - l_f * c_f) / m / v;
         0.0, 0.0, 0.0, 1.0;
         0.0, ((lr * cr - lf * cf) / i_z) / v, (l_f * c_f - l_r * c_r) / i_z,
         (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
        */
        cf_ = -control_conf_->lat_controller_conf().cf();
        cr_ = -control_conf_->lat_controller_conf().cr();
        matrix_a_(0, 1) = 0.0;
        matrix_a_coeff_(0, 2) = 1.0;
      }
      else
      {
        /*
        A matrix (Gear Drive)
        [0.0, 1.0, 0.0, 0.0;
         0.0, (-(c_f + c_r) / m) / v, (c_f + c_r) / m,
         (l_r * c_r - l_f * c_f) / m / v;
         0.0, 0.0, 0.0, 1.0;
         0.0, ((lr * cr - lf * cf) / i_z) / v, (l_f * c_f - l_r * c_r) / i_z,
         (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
        */
        cf_ = control_conf_->lat_controller_conf().cf();
        cr_ = control_conf_->lat_controller_conf().cr();
        matrix_a_(0, 1) = 1.0;
        matrix_a_coeff_(0, 2) = 0.0;
      }

      // 感觉有点多余，前面不是赋值了？ 不多余
      // 因为万一D档和R档切换，A矩阵的值是在变化的话，所以D档和R档时要按照各自的方式计算一下，因为cf_,cr_也更新了
      matrix_a_(1, 2) = (cf_ + cr_) / mass_;
      matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
      matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
      matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
      matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
      matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;

      /*
      b = [0.0, c_f / m, 0.0, l_f * c_f / i_z]^T
      */
      matrix_b_(1, 0) = cf_ / mass_;
      matrix_b_(3, 0) = lf_ * cf_ / iz_;
      matrix_bd_ = matrix_b_ * ts_;
      // 调用更新驾驶航向函数，也是要满足FLAGS_reverse_heading_control默认关闭，实际这个函数没啥用
      UpdateDrivingOrientation();

      // 定义临时变量SimpleLateralDebug类对象debug
      // SimpleLateralDebug类由modules/control/proto/control_cmd.proto文件生成，参见谷歌Protobuf使用
      // 从名字就可以看出SimpleLateralDebug,"简单横向调试"，这个类就是用来调试横向控制的，里面用来储存一些横向控制过程量
      //  这个临时变量首先取出cmd里的debug?然后又Clear清空？

      SimpleLateralDebug *debug = cmd->mutable_debug()->mutable_simple_lat_debug();
      debug->Clear();

      // 更新车辆状态矩阵X=[e1 e1' e2 e2']
      // 首先该函数UpdateState()内部调用了ComputeLateralErrors()函数得到的各种误差信息存放到debug中
      // 该函数的输入SimpleLateralDebug类型指针*debug，其实是要填充这个指针传出来debug信息
      UpdateState(debug);

      // 更新矩阵？更新什么矩阵？
      // 主要是更新车辆状态方程系数矩阵A及其离散形式中与速度相关的时变项，得到matrix_ad_
      UpdateMatrix();

      // Compound discrete matrix with road preview model
      // 更新以及组装离散矩阵Ad,Bd和预览窗口模型，预览窗在横向控制中都关闭了，控制配置里preview_window为0，得到matrix_adc_和matrix_bdc_
      UpdateMatrixCompound();

      // 当在R档时调整矩阵Q也就是LatControllr类成员matrix_q_
      // 将控制配置里的reverse_matrix_q写入LatControllr类成员matrix_q_
      int q_param_size = control_conf_->lat_controller_conf().matrix_q_size();
      int reverse_q_param_size =
          control_conf_->lat_controller_conf().reverse_matrix_q_size();

      if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE)
      {
        for (int i = 0; i < reverse_q_param_size; ++i)
        { // R档加载控制配置里的reverse_matrix_q
          matrix_q_(i, i) =
              control_conf_->lat_controller_conf().reverse_matrix_q(i);
        }
      }
      else
      {
        for (int i = 0; i < q_param_size; ++i)
        { // 非R档加载控制配置里的matrix_q
          matrix_q_(i, i) = control_conf_->lat_controller_conf().matrix_q(i);
        }
      }

      // Add gain scheduler for higher speed steering
      // 对于更高速度的转向增加增益调度表
      // FLAGS_enable_gain_scheduler去modules/control/common/control_gflags.cc取出
      // 取出control_gflags.cc中enable_gain_scheduler的值，默认是false，但是实际上很有用的
      // 这里介绍一下
      if (FLAGS_enable_gain_scheduler) // 如果打开高速度的转向增加增益调度表
      {
        // Q矩阵的(1,1)也就是横向误差平方和的权重系数, 第1行第1列
        // Q(1,1)=Q(1,1)*(用之前加载的横向误差调度增益表根据当前车速插值得到的ratio)
        matrix_q_updated_(0, 0) =
            matrix_q_(0, 0) * lat_err_interpolation_->Interpolate(
                                  std::fabs(vehicle_state->linear_velocity()));
        // Q矩阵的(3,3)也就是航向误差平方和的权重系数, 第3行第3列
        // Q(3,3)=Q(3,3)*(用之前加载的航向误差调度增益表根据当前车速插值得到的ratio)
        matrix_q_updated_(2, 2) =
            matrix_q_(2, 2) * heading_err_interpolation_->Interpolate(
                                  std::fabs(vehicle_state->linear_velocity()));
        common::math::SolveLQRProblem(matrix_adc_, matrix_bdc_, matrix_q_updated_,
                                      matrix_r_, lqr_eps_, lqr_max_iteration_,
                                      &matrix_k_);
      }
      else
      {
        // 调用common::math命名空间下的SolvelLQRProblem()函数
        // 求解LQR问题，求解到的最优状态反馈矩阵K放入matrix_k_中，最后一个引用变量作参数，明摆着就是要用对形参的修改改变实参，常用来存放函数计算结果
        // 参数：矩阵Ad,Bd,Q,R,lqr_eps_是LQR求解精度，lqr_max_iteration_是求解的最大迭代次数
        // 参数：matrix_k_用来求解LQR控制方法计算出的最优状态反馈矩阵k
        // 这个if和else都是调用SolveLQRProblem函数，其中不同就只有一个是matrix_q_updated_是考虑调度增益表的Q，Q与车速有关
        // 一个是matrix_q_不考虑增益调度表的Q矩阵，Q与车速无关，看你是否打开调度增益表
        // 根据经验打开的话更容易获得更好的控制性能，否则低速适用的Q用到高速往往容易出现画龙
        common::math::SolveLQRProblem(matrix_adc_, matrix_bdc_, matrix_q_,
                                      matrix_r_, lqr_eps_, lqr_max_iteration_,
                                      &matrix_k_);
      }

      // feedback = - K * state
      // Convert vehicle steer angle from rad to degree and then to steer degree
      // then to 100% ratio
      const double steer_angle_feedback = -(matrix_k_ * matrix_state_)(0, 0) * 180 /
                                          M_PI * steer_ratio_ /
                                          steer_single_direction_max_degree_ * 100; // 计算反馈，方向盘转角百分数

      const double steer_angle_feedforward = ComputeFeedForward(debug->curvature()); // 计算前馈，方向盘转角百分数

      double steer_angle = 0.0;
      double steer_angle_feedback_augment = 0.0;
      // Augment the feedback control on lateral error at the desired speed domain
      if (enable_leadlag_)
      { // enable_leadlag_=control_conf->lat_controller_conf()->enable_reverse_leadlag_compensation()
        // 见控制配置文件control_conf.pb.txt里enable_reverse_leadlag_compensation默认设置true
        // 如果打开leadlag超前滞后控制器
        if (FLAGS_enable_feedback_augment_on_high_speed ||
            std::fabs(vehicle_state->linear_velocity()) < low_speed_bound_)
        {
          // 如果车辆打开高速的反馈增强控制 或 车速小于低高速边界速度
          // low_speed_bound_是控制配置文件里的switch_speed()默认设置为3.0 m/s
          // enable_feedback_augment_on_high_speed的值见control_gflags.cc里，默认设置为false，高速不打开
          // 满足上述条件,实际上就是低速时打开超前滞后控制器，然后这个超前滞后控制器只对横向误差进行增强控制
          // 这一段不详解，计算出反馈增强控制方向盘转角百分数steer_angle_feedback_augment
          steer_angle_feedback_augment =
              leadlag_controller_.Control(-matrix_state_(0, 0), ts_) * 180 / M_PI *
              steer_ratio_ / steer_single_direction_max_degree_ * 100;
          if (std::fabs(vehicle_state->linear_velocity()) >
              low_speed_bound_ - low_speed_window_)
          {
            // Within the low-high speed transition window, linerly interplolate the
            // augment control gain for "soft" control switch
            steer_angle_feedback_augment = common::math::lerp(
                steer_angle_feedback_augment, low_speed_bound_ - low_speed_window_,
                0.0, low_speed_bound_, std::fabs(vehicle_state->linear_velocity()));
          }
        }
      }
      steer_angle = steer_angle_feedback + steer_angle_feedforward +
                    steer_angle_feedback_augment;

      // Compute the steering command limit with the given maximum lateral
      // acceleration
      // 这一部分计算方向盘转向的限制，根据最大的横向加速度限制计算,这一块主要是限制横向加速度
      // 又涉及FLAGS_set_steer_limit 见control_gflags.cc里，FLAGS_set_steer_limit取出文件里定义的set_steer_limit的值，默认false
      // 若限制横向加速度 最大方向盘转角百分数 = atan(ay_max * L / v^2) * steerratio * 180/pi /max_steer_ang * 100
      // 根据上述公式可以从限制的最大横向加速度计算出最大的方向盘转角控制百分数
      // 若无限制 最大方向盘转角百分数 = 100
      const double steer_limit =
          FLAGS_set_steer_limit ? std::atan(max_lat_acc_ * wheelbase_ /
                                            (vehicle_state->linear_velocity() *
                                             vehicle_state->linear_velocity())) *
                                      steer_ratio_ * 180 / M_PI /
                                      steer_single_direction_max_degree_ * 100
                                : 100.0;
      // 这一部分主要是对方向盘转动速率进行限制，如果FLAGS_enable_maximum_steer_rate_limit打开，默认不打开
      // 如果打开取出车辆参数配置文件中的max_steer_angle_rate转化成百分数 /apollo/modules/common/data/vehicle_param.pb.txt
      // 如果没打开，最大就限制为100
      // 这里计算 一个周期方向盘转角最大增量 = 最大方向盘角速度 * 控制周期
      // 此刻方向盘转角控制量只能在范围内：上一时刻方向盘转角控制量 +/- 一个周期方向盘转角最大增量
      const double steer_diff_with_max_rate =
          FLAGS_enable_maximum_steer_rate_limit
              ? vehicle_param_.max_steer_angle_rate() * ts_ * 180 / M_PI /
                    steer_single_direction_max_degree_ * 100
              : 100.0;
      // 方向盘实际转角
      // 方向盘实际转角从chassis信息读，canbus从车辆can线上读到发出来的
      const double steering_position = chassis->steering_percentage();

      // Re-compute the steering command if the MRAC control is enabled, with steer
      // angle limitation and steer rate limitation
      // 如果打开MRAC模型参考自适应控制 enable_mrac_，重新计算方向盘转角控制量，并用方向盘转角和转速限制对转角控制量进行限幅
      // enable_mrac_ =control_conf_->lat_controller_conf().enable_steer_mrac_control()
      // 也是看控制配置文件里的参数默认是false，略过
      if (enable_mrac_)
      {
        const int mrac_model_order = control_conf_->lat_controller_conf()
                                         .steer_mrac_conf()
                                         .mrac_model_order();
        Matrix steer_state = Matrix::Zero(mrac_model_order, 1);
        steer_state(0, 0) = chassis->steering_percentage();
        if (mrac_model_order > 1)
        {
          steer_state(1, 0) = (steering_position - pre_steering_position_) / ts_;
        }
        if (std::fabs(vehicle_state->linear_velocity()) >
            control_conf_->minimum_speed_resolution())
        {
          mrac_controller_.SetStateAdaptionRate(1.0);
          mrac_controller_.SetInputAdaptionRate(1.0);
        }
        else
        {
          mrac_controller_.SetStateAdaptionRate(0.0);
          mrac_controller_.SetInputAdaptionRate(0.0);
        }
        steer_angle = mrac_controller_.Control(
            steer_angle, steer_state, steer_limit, steer_diff_with_max_rate / ts_);
        // Set the steer mrac debug message
        MracDebug *mracdebug = debug->mutable_steer_mrac_debug();
        Matrix steer_reference = mrac_controller_.CurrentReferenceState();
        mracdebug->set_mrac_model_order(mrac_model_order);
        for (int i = 0; i < mrac_model_order; ++i)
        {
          mracdebug->add_mrac_reference_state(steer_reference(i, 0));
          mracdebug->add_mrac_state_error(steer_state(i, 0) -
                                          steer_reference(i, 0));
          mracdebug->mutable_mrac_adaptive_gain()->add_state_adaptive_gain(
              mrac_controller_.CurrentStateAdaptionGain()(i, 0));
        }
        mracdebug->mutable_mrac_adaptive_gain()->add_input_adaptive_gain(
            mrac_controller_.CurrentInputAdaptionGain()(0, 0));
        mracdebug->set_mrac_reference_saturation_status(
            mrac_controller_.ReferenceSaturationStatus());
        mracdebug->set_mrac_control_saturation_status(
            mrac_controller_.ControlSaturationStatus());
      }
      // 将当前时刻方向盘的转角置为上一时刻的转角
      pre_steering_position_ = steering_position;
      debug->set_steer_mrac_enable_status(enable_mrac_);

      // Clamp the steer angle with steer limitations at current speed
      // 根据当前车速对下发方向盘转角进行限幅，横向加速度的限制转化到此刻方向盘转角限制就会引入车速
      // steer_limit的来源前面已经介绍过，通过横向最大加速度或者方向盘最大转角限制
      // 限幅后的方向盘转角steer_angle_limited
      double steer_angle_limited =
          common::math::Clamp(steer_angle, -steer_limit, steer_limit);
      steer_angle = steer_angle_limited;
      debug->set_steer_angle_limited(steer_angle_limited);

      // Limit the steering command with the designed digital filter
      // 进行滤波
      steer_angle = digital_filter_.Filter(steer_angle);
      steer_angle = common::math::Clamp(steer_angle, -100.0, 100.0);

      // Check if the steer is locked and hence the previous steer angle should be
      // executed
      // 当处于D档或倒档 且 车速小于某一速度 且处于自驾模式时锁定方向盘，方向盘控制转角就保持上一次的命令值
      // FLAGS_lock_steer_speed 意思就是FLAGS_去XXXgflags.cc下取出lock_steer_speed的值
      // lock_steer_speed在modules/control/common/control_gflags.cc里定义为0.081m/s

      if (std::abs(vehicle_state->linear_velocity()) < FLAGS_lock_steer_speed &&
          (vehicle_state->gear() == canbus::Chassis::GEAR_DRIVE ||
           vehicle_state->gear() == canbus::Chassis::GEAR_REVERSE) &&
          chassis->driving_mode() == canbus::Chassis::COMPLETE_AUTO_DRIVE)
      {
        steer_angle = pre_steer_angle_;
      }

      // Set the steer commands
      // 设定转角指令，再通过最大转角速率再次进行限制幅度，最多只能=上次的转角指令+/-最大转角速率 * Ts
      // 目前的代码是处在ComputeCommand函数,控制指令计算出来就放在cmd指针里
      cmd->set_steering_target(common::math::Clamp(
          steer_angle, pre_steer_angle_ - steer_diff_with_max_rate,
          pre_steer_angle_ + steer_diff_with_max_rate));
      cmd->set_steering_rate(FLAGS_steer_angle_rate);
      // 将此刻的方向盘命令值赋给上一时刻方向盘命令值
      pre_steer_angle_ = cmd->steering_target();

      // compute extra information for logging and debugging
      // 为 logging and debugging计算一些额外信息
      const double steer_angle_lateral_contribution =
          -matrix_k_(0, 0) * matrix_state_(0, 0) * 180 / M_PI * steer_ratio_ /
          steer_single_direction_max_degree_ * 100;

      const double steer_angle_lateral_rate_contribution =
          -matrix_k_(0, 1) * matrix_state_(1, 0) * 180 / M_PI * steer_ratio_ /
          steer_single_direction_max_degree_ * 100;

      const double steer_angle_heading_contribution =
          -matrix_k_(0, 2) * matrix_state_(2, 0) * 180 / M_PI * steer_ratio_ /
          steer_single_direction_max_degree_ * 100;

      const double steer_angle_heading_rate_contribution =
          -matrix_k_(0, 3) * matrix_state_(3, 0) * 180 / M_PI * steer_ratio_ /
          steer_single_direction_max_degree_ * 100;

      debug->set_heading(driving_orientation_);
      debug->set_steer_angle(steer_angle);
      debug->set_steer_angle_feedforward(steer_angle_feedforward);
      debug->set_steer_angle_lateral_contribution(steer_angle_lateral_contribution);
      debug->set_steer_angle_lateral_rate_contribution(
          steer_angle_lateral_rate_contribution);
      debug->set_steer_angle_heading_contribution(steer_angle_heading_contribution);
      debug->set_steer_angle_heading_rate_contribution(
          steer_angle_heading_rate_contribution);
      debug->set_steer_angle_feedback(steer_angle_feedback);
      debug->set_steer_angle_feedback_augment(steer_angle_feedback_augment);
      debug->set_steering_position(steering_position);
      debug->set_ref_speed(vehicle_state->linear_velocity());

      ProcessLogs(debug, chassis);
      return Status::OK();
    }

    Status LatController::Reset()
    {
      matrix_state_.setZero();
      if (enable_mrac_)
      {
        mrac_controller_.Reset();
      }
      return Status::OK();
    }

    void LatController::UpdateState(SimpleLateralDebug *debug)
    {
      // 该函数的输入SimpleLateralDebug类型指针*debug，其实是要填充这个指针传出来debug信息
      // 该函数在lat_controller.cc上面的ComputeControlCommand()函数里被调用
      // SimpleLateralDebug是control_cmd.proto下的一个message类型，包含各种调试信息
      // injector_是lat_controller类的数据成员DependencyInjector类对象，vehicle_state()是DependencyInjector成员函数
      // 用于返回车辆当前状态，injector_在control_component.cc里的Status ControlComponent::CheckInput()函数里被更新
      auto vehicle_state = injector_->vehicle_state();

      if (FLAGS_use_navigation_mode)
      { // 是否使用navigation_mode,默认不使用，在config_gflags.cc里设置DEFINE_bool(use_navigation_mode, false,
        // 默认不使用导航模式直接跳过此段代码
        ComputeLateralErrors(
            0.0, 0.0, driving_orientation_, vehicle_state->linear_velocity(),
            vehicle_state->angular_velocity(), vehicle_state->linear_acceleration(),
            trajectory_analyzer_, debug);
      }
      else
      {
        // 不使用导航模式则执行这部分
        // 将车辆状态的坐标系从后轴中心转换到质心如果满足条件，默认倒档不需转换，D档要转换
        // 调用了vehicle_state的成员函数ComputeCOMPosition()，lr_是后轴中心到质心的距离，然后被com引用
        const auto &com = vehicle_state->ComputeCOMPosition(lr_); // 当前中心位置

        // 将x,y,heading航向,v纵向速度，angular_vel航向变化率，纵向加速度，debug是调试信息的待填充指针，就是目前所在函数的参数
        // trajectory_analyzer_是TrajectoryAnalyzer类对象只要把轨迹点传进去，就可以实现轨迹分析的相关功能见trajectory_analyzer.h如
        // 获取轨迹段序号，查找距离当前位置的时间最近点ref和距离最近点match,车辆后轴中心坐标转换到质心等功能
        // trajectory_analyzer_在Status LatController::ComputeControlCommand()函数中被赋值
        // ComputeLateralErrors()函数计算得到的error都放入message debug里，稍后会用debug来更新车辆状态矩阵x
        ComputeLateralErrors(
            com.x(), com.y(), driving_orientation_,
            vehicle_state->linear_velocity(), vehicle_state->angular_velocity(),
            vehicle_state->linear_acceleration(), trajectory_analyzer_, debug);
      }

      // 状态矩阵更新;
      // enable_look_ahead_back_control_=control_conf_->lat_controller_conf().enable_look_ahead_back_control();是在配置文件里设定
      // control_conf.pb.txt里lat_controller_conf里的enable_look_ahead_back_control默认打开

      if (enable_look_ahead_back_control_)
      { // 当打开lookaheadback control，预瞄默认打开
        // 当打开这个e1和e3分别赋值横向反馈误差和航向反馈误差，和下面else两行的区别在哪里？
        // 若打开lookahead（D档）,lookback(R档)则x中的e1,e3就为考虑了lookahead的误差
        // lateral_error_feedback = lateral_error + 参考点到lookahead点的横向误差
        // heading_error_feedback = heading_error + ref_heading - lookahead点的heading 实际上就是匹配点到lookahead点的航向差
        // heading_error = 车辆heading - ref_heading,(这里的预瞄和我小论文里面的预瞄不一样，它这里是通过预瞄距离确定预瞄位置，预瞄位置是参考轨迹上的)
        matrix_state_(0, 0) = debug->lateral_error_feedback();
        matrix_state_(2, 0) = debug->heading_error_feedback();
      }
      else
      {
        matrix_state_(0, 0) = debug->lateral_error();
        matrix_state_(2, 0) = debug->heading_error();
      }
      matrix_state_(1, 0) = debug->lateral_error_rate();
      matrix_state_(3, 0) = debug->heading_error_rate();

      // 这一部分是更新状态矩阵中的preview项，但是preview_window默认为0忽略，此段跳过不管
      for (int i = 0; i < preview_window_; ++i)
      {
        const double preview_time = ts_ * (i + 1);
        const auto preview_point =
            trajectory_analyzer_.QueryNearestPointByRelativeTime(preview_time);

        const auto matched_point = trajectory_analyzer_.QueryNearestPointByPosition(
            preview_point.path_point().x(), preview_point.path_point().y());

        const double dx =
            preview_point.path_point().x() - matched_point.path_point().x();
        const double dy =
            preview_point.path_point().y() - matched_point.path_point().y();

        const double cos_matched_theta =
            std::cos(matched_point.path_point().theta());
        const double sin_matched_theta =
            std::sin(matched_point.path_point().theta());
        const double preview_d_error =
            cos_matched_theta * dy - sin_matched_theta * dx;

        matrix_state_(basic_state_size_ + i, 0) = preview_d_error;
      }
    }

    void LatController::UpdateMatrix()
    {
      double v;
      // At reverse driving, replace the lateral translational motion dynamics with
      // the corresponding kinematic models
      if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE)
      {
        v = std::min(injector_->vehicle_state()->linear_velocity(),
                     -minimum_speed_protection_);
        matrix_a_(0, 2) = matrix_a_coeff_(0, 2) * v;
      }
      else
      {
        v = std::max(injector_->vehicle_state()->linear_velocity(),
                     minimum_speed_protection_);
        matrix_a_(0, 2) = 0.0;
      }
      matrix_a_(1, 1) = matrix_a_coeff_(1, 1) / v;
      matrix_a_(1, 3) = matrix_a_coeff_(1, 3) / v;
      matrix_a_(3, 1) = matrix_a_coeff_(3, 1) / v;
      matrix_a_(3, 3) = matrix_a_coeff_(3, 3) / v;
      // 定义了一个单位阵，A阵列数*A阵列数
      Matrix matrix_i = Matrix::Identity(matrix_a_.cols(), matrix_a_.cols());
      // 计算A矩阵的离散化形式Ad，利用欧拉法，参考 https://blog.csdn.net/YOURUOLI/article/details/129293914
      matrix_ad_ = (matrix_i - ts_ * 0.5 * matrix_a_).inverse() *
                   (matrix_i + ts_ * 0.5 * matrix_a_);
    }

    void LatController::UpdateMatrixCompound()
    {
      // Initialize preview matrix
      // 更新matrix_adc_，如果preview_window_=0，matrix_adc_=matrix_ad_、matrix_bdc_=matrix_bd_
      matrix_adc_.block(0, 0, basic_state_size_, basic_state_size_) = matrix_ad_;
      matrix_bdc_.block(0, 0, basic_state_size_, 1) = matrix_bd_;
      if (preview_window_ > 0)
      {
        matrix_bdc_(matrix_bdc_.rows() - 1, 0) = 1;
        // Update A matrix;
        for (int i = 0; i < preview_window_ - 1; ++i)
        {
          matrix_adc_(basic_state_size_ + i, basic_state_size_ + 1 + i) = 1;
        }
      }
    }

    double LatController::ComputeFeedForward(double ref_curvature) const
    {
      const double kv =
          lr_ * mass_ / 2 / cf_ / wheelbase_ - lf_ * mass_ / 2 / cr_ / wheelbase_;

      // Calculate the feedforward term of the lateral controller; then change it
      // from rad to %
      const double v = injector_->vehicle_state()->linear_velocity();
      double steer_angle_feedforwardterm;
      if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE)
      {
        steer_angle_feedforwardterm = wheelbase_ * ref_curvature * 180 / M_PI *
                                      steer_ratio_ /
                                      steer_single_direction_max_degree_ * 100;
      }
      else
      {
        steer_angle_feedforwardterm =
            (wheelbase_ * ref_curvature + kv * v * v * ref_curvature -
             matrix_k_(0, 2) *
                 (lr_ * ref_curvature -
                  lf_ * mass_ * v * v * ref_curvature / 2 / cr_ / wheelbase_)) *
            180 / M_PI * steer_ratio_ / steer_single_direction_max_degree_ * 100;
      }

      return steer_angle_feedforwardterm;
    }

    void LatController::ComputeLateralErrors(
        const double x, const double y, const double theta, const double linear_v, // linear_v这个是车头方向速度vx
        const double angular_v, const double linear_a,
        const TrajectoryAnalyzer &trajectory_analyzer, SimpleLateralDebug *debug)
    {
      // 参数详解 输入xy坐标，车辆当前heading,纵向速度v,heading变化率，纵向加速度，
      // 轨迹相关信息trajectory_analyzer对象用于提供轨迹的参考点，匹配点，lookahead点等信息
      // debug指针用于存放这里计算到的误差供其他函数调用，
      // SimpleLateralDebug是在control/proto/control_cmd.proto下定义的messgae类

      TrajectoryPoint target_point; // TrajectoryPoint是apollo/modules/common/proto/pnc_point.proto里定义的一个message类
                                    // TrajectoryPoint类包含轨迹点的v,acc,jerk,相对时间，前轮转向角等信息
                                    // 这里定义了一个TrajectoryPoint类对象
                                    // 计算目标点
      if (FLAGS_query_time_nearest_point_only)
      { // 如果只将车辆当前的时间向前加固定时间长度后在轨迹上对应点作为目标点，gflags老套路去
        // modules/control/common/control_gflags.cc路径下取出query_time_nearest_point_only值
        // 默认是false
        target_point = trajectory_analyzer.QueryNearestPointByAbsoluteTime(
            Clock::NowInSeconds() + query_relative_time_);
        // query_relative_time_ = control_conf->query_relative_time();这个值是从控制配置文件中读取，默认0.8秒
        // query_time_nearest_point_only的意识就是始终将车辆当前时间向前加0.8秒在轨迹上对应的点作为目标点
        // 在时域上向前看0.8秒作为目标点
      }
      else
      {

        if (FLAGS_use_navigation_mode &&
            !FLAGS_enable_navigation_mode_position_update)
        {                                                                     // 默认不采用导航模式，直接看else
          target_point = trajectory_analyzer.QueryNearestPointByAbsoluteTime( // 通过时间来寻找目标点
              Clock::NowInSeconds() + query_relative_time_);
        }
        else
        {                                                                       // 如果不采用这种向前看0.8秒作为目标点的方法，默认不采用的
          target_point = trajectory_analyzer.QueryNearestPointByPosition(x, y); // 通过最小距离来寻找目标点，这样最好，也是默认的
        }
      }
      const double dx = x - target_point.path_point().x();
      const double dy = y - target_point.path_point().y();

      // 将目标点的x坐标存入debug指针，debug.current_target_point.path_point.x,指针要用->来赋值成员
      debug->mutable_current_target_point()->mutable_path_point()->set_x(
          target_point.path_point().x());
      debug->mutable_current_target_point()->mutable_path_point()->set_y(
          target_point.path_point().y());

      ADEBUG << "x point: " << x << " y point: " << y;
      ADEBUG << "match point information : " << target_point.ShortDebugString();

      const double cos_target_heading = std::cos(target_point.path_point().theta()); // 计算目标点处的heading角的正余弦值
      const double sin_target_heading = std::sin(target_point.path_point().theta());

      double lateral_error = cos_target_heading * dy - sin_target_heading * dx; // 利用向量，在目标点的法向量上投影，目标点法向量为(cos(pi/2+thetaref),sin(pi/2+thetaref))=(-sin(thetaref),cos(thetaref)),因此 这里为(x-xref,y-yref)*(-sin(thetaref),cos(thetaref))
      if (FLAGS_enable_navigation_mode_error_filter)
      { // 默认时关闭的，导航模式及滤波器都关闭，直接略过
        lateral_error = lateral_error_filter_.Update(lateral_error);
      }

      debug->set_lateral_error(lateral_error);

      debug->set_ref_heading(target_point.path_point().theta()); // 将目标点的heading角填入debug指针的debug.ref_heading
      double heading_error =
          common::math::NormalizeAngle(theta - debug->ref_heading());
      if (FLAGS_enable_navigation_mode_error_filter)
      {
        heading_error = heading_error_filter_.Update(heading_error);
      }
      debug->set_heading_error(heading_error);

      // Within the low-high speed transition window, linerly interplolate the
      // lookahead/lookback station for "soft" prediction window switch

      // 在低-高速切换窗口，现行插值预瞄距离为了更柔和的预测切换？
      double lookahead_station = 0.0;
      double lookback_station = 0.0;
      if (std::fabs(linear_v) >= low_speed_bound_) // 自适应预瞄距离
      {                                            // 如果速度的绝对值>=low_speed_bound_(低速边界)
        // 这个也是在control_conf配置文件中定义的
        lookahead_station = lookahead_station_high_speed_;
        lookback_station = lookback_station_high_speed_;
      }
      else if (std::fabs(linear_v) < low_speed_bound_ - low_speed_window_)
      { // 若纵向速度绝对值小于低速边界-低速窗口
        lookahead_station = lookahead_station_low_speed_;
        lookback_station = lookback_station_low_speed_;
      }
      else
      { // 若纵向速度绝对值小于低速边界又大于（低速边界-低速窗口）就插值计算预瞄距离
        lookahead_station = common::math::lerp(
            lookahead_station_low_speed_, low_speed_bound_ - low_speed_window_,
            lookahead_station_high_speed_, low_speed_bound_, std::fabs(linear_v));
        lookback_station = common::math::lerp(
            lookback_station_low_speed_, low_speed_bound_ - low_speed_window_,
            lookback_station_high_speed_, low_speed_bound_, std::fabs(linear_v));
      }

      // Estimate the heading error with look-ahead/look-back windows as feedback
      // signal for special driving scenarios
      double heading_error_feedback;
      if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE)
      { // 如果倒车
        heading_error_feedback = heading_error;
      }
      else
      {
        // 目标点的相对时间再加上预瞄时间(预瞄距离/车辆纵向速度)作为总相对时间
        // 然后去trajectory_analyzer轨迹信息上根据总相对时间找出预瞄点
        auto lookahead_point = trajectory_analyzer.QueryNearestPointByRelativeTime(
            target_point.relative_time() +
            lookahead_station /
                (std::max(std::fabs(linear_v), 0.1) * std::cos(heading_error))); // 在估计预瞄时间时纵向速度若小于0.1就按0.1
        // heading_error=车辆当前heading-参考点heading
        // heading_error_feedback=heading_error+参考点heading-预瞄点heading=车辆当前heading-预瞄点heading
        heading_error_feedback = common::math::NormalizeAngle(
            heading_error + target_point.path_point().theta() -
            lookahead_point.path_point().theta());
      }
      debug->set_heading_error_feedback(heading_error_feedback);

      // Estimate the lateral error with look-ahead/look-back windows as feedback
      // signal for special driving scenarios
      double lateral_error_feedback;
      if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE)
      {
        lateral_error_feedback =
            lateral_error - lookback_station * std::sin(heading_error);
      }
      else
      { // 前进档的lateral_error_feedback=lateral_error+前进预瞄距离*sin(heading_error)
        lateral_error_feedback =
            lateral_error + lookahead_station * std::sin(heading_error);
      }
      debug->set_lateral_error_feedback(lateral_error_feedback);

      auto lateral_error_dot = linear_v * std::sin(heading_error); // 没问题，从我的图上，几何推导可得
      auto lateral_error_dot_dot = linear_a * std::sin(heading_error);
      if (FLAGS_reverse_heading_control)
      {
        if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE)
        {
          lateral_error_dot = -lateral_error_dot;
          lateral_error_dot_dot = -lateral_error_dot_dot;
        }
      }
      debug->set_lateral_error_rate(lateral_error_dot);
      debug->set_lateral_acceleration(lateral_error_dot_dot);
      debug->set_lateral_jerk(
          (debug->lateral_acceleration() - previous_lateral_acceleration_) / ts_);
      previous_lateral_acceleration_ = debug->lateral_acceleration();

      if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE)
      {
        debug->set_heading_rate(-angular_v);
      }
      else
      {
        debug->set_heading_rate(angular_v);
      }
      debug->set_ref_heading_rate(target_point.path_point().kappa() *
                                  target_point.v());
      debug->set_heading_error_rate(debug->heading_rate() -
                                    debug->ref_heading_rate()); // 航向偏差变化率计算

      debug->set_heading_acceleration(
          (debug->heading_rate() - previous_heading_rate_) / ts_);
      debug->set_ref_heading_acceleration(
          (debug->ref_heading_rate() - previous_ref_heading_rate_) / ts_);
      debug->set_heading_error_acceleration(debug->heading_acceleration() -
                                            debug->ref_heading_acceleration()); // 设置航向偏差变化率倒数
      previous_heading_rate_ = debug->heading_rate();
      previous_ref_heading_rate_ = debug->ref_heading_rate();

      debug->set_heading_jerk(
          (debug->heading_acceleration() - previous_heading_acceleration_) / ts_);
      debug->set_ref_heading_jerk(
          (debug->ref_heading_acceleration() - previous_ref_heading_acceleration_) /
          ts_);
      debug->set_heading_error_jerk(debug->heading_jerk() -
                                    debug->ref_heading_jerk()); // 设置航向偏差加加速度
      previous_heading_acceleration_ = debug->heading_acceleration();
      previous_ref_heading_acceleration_ = debug->ref_heading_acceleration();

      debug->set_curvature(target_point.path_point().kappa()); // 设置参考点曲率
    }

    void LatController::UpdateDrivingOrientation()
    {
      auto vehicle_state = injector_->vehicle_state();
      driving_orientation_ = vehicle_state->heading();
      matrix_bd_ = matrix_b_ * ts_;
      // Reverse the driving direction if the vehicle is in reverse mode
      if (FLAGS_reverse_heading_control)
      {
        if (vehicle_state->gear() == canbus::Chassis::GEAR_REVERSE)
        {
          driving_orientation_ =
              common::math::NormalizeAngle(driving_orientation_ + M_PI);
          // Update Matrix_b for reverse mode
          matrix_bd_ = -matrix_b_ * ts_;
          ADEBUG << "Matrix_b changed due to gear direction";
        }
      }
    }

  } // namespace control
} // namespace apollo
