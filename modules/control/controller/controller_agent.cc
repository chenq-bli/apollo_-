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

#include "modules/control/controller/controller_agent.h"

#include <utility>

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/control/common/control_gflags.h"
#include "modules/control/controller/lat_controller.h"
#include "modules/control/controller/lon_controller.h"
#include "modules/control/controller/mpc_controller.h"

namespace apollo
{
  namespace control
  {

    using apollo::common::ErrorCode;
    using apollo::common::Status;
    using apollo::cyber::Clock;

    void ControllerAgent::RegisterControllers(const ControlConf *control_conf)
    {
      AINFO << "Only support MPC controller or Lat + Lon controllers as of now";
      // control_conf->active_controllers()中要么是一个 MPC controller，要么是Lat + Lon controllers两个控制器
      for (auto active_controller : control_conf->active_controllers())
      {
        switch (active_controller)
        {
        case ControlConf::MPC_CONTROLLER:
          controller_factory_.Register(
              ControlConf::MPC_CONTROLLER,
              []() -> Controller *
              { return new MPCController(); }); // new 构造一个MPCController，存放到producers_成员
          break;
        case ControlConf::LAT_CONTROLLER:
          controller_factory_.Register(
              ControlConf::LAT_CONTROLLER,
              []() -> Controller *
              { return new LatController(); }); // new 构造一个LatController()，存放到producers_成员
          break;
        case ControlConf::LON_CONTROLLER:
          controller_factory_.Register(
              ControlConf::LON_CONTROLLER,
              []() -> Controller *
              { return new LonController(); }); // new 构造一个LonController()，存放到producers_成员，后面会利用producers_成员寻找到所注册的控制器，然后放到控制器列表中
          break;
        default:
          AERROR << "Unknown active controller type:" << active_controller;
        }
      }
    }

    Status ControllerAgent::InitializeConf(const ControlConf *control_conf)
    {
      if (!control_conf)
      {
        AERROR << "control_conf is null";
        return Status(ErrorCode::CONTROL_INIT_ERROR, "Failed to load config");
      }
      control_conf_ = control_conf;
      // control_conf->active_controllers()中要么是一个 MPC controller，要么是Lat + Lon controllers两个控制器
      for (auto controller_type : control_conf_->active_controllers())
      {
        auto controller = controller_factory_.CreateObject(
            static_cast<ControlConf::ControllerType>(controller_type)); // 利用producers_成员寻找到指向所注册的控制器的指针，并创建智能指针指向，并返回
        if (controller)
        {                                                       // 如果智能指针关联的指针不为空
          controller_list_.emplace_back(std::move(controller)); // 取消智能指针与原指针的关联，并将原指针emplace_back()进入controller_list_
        }
        else
        {
          AERROR << "Controller: " << controller_type << "is not supported";
          return Status(ErrorCode::CONTROL_INIT_ERROR,
                        "Invalid controller type:" + controller_type);
        }
      }
      return Status::OK();
    }

    Status ControllerAgent::Init(std::shared_ptr<DependencyInjector> injector,
                                 const ControlConf *control_conf)
    {
      injector_ = injector;
      RegisterControllers(control_conf);                                           // 注册控制器
      ACHECK(InitializeConf(control_conf).ok()) << "Failed to initialize config."; // 进行InitializeConf(control_conf)
      for (auto &controller : controller_list_)
      {
        if (controller == nullptr)
        {
          return Status(ErrorCode::CONTROL_INIT_ERROR, "Controller is null.");
        }
        if (!controller->Init(injector, control_conf_).ok())
        { // InitializeConf(control_conf)中实例化控制器后当然得对各控制器进行初始化
          AERROR << "Controller <" << controller->Name() << "> init failed!";
          return Status(ErrorCode::CONTROL_INIT_ERROR,
                        "Failed to init Controller:" + controller->Name());
        }
        AINFO << "Controller <" << controller->Name() << "> init done!";
      }
      return Status::OK();
    }

    Status ControllerAgent::ComputeControlCommand(
        const localization::LocalizationEstimate *localization,
        const canbus::Chassis *chassis, const planning::ADCTrajectory *trajectory,
        control::ControlCommand *cmd)
    {
      // 对controller_list_中的每一个controller，调用其ComputeControlCommand()方法。controller_list_在ControllerAgent::InitializeConf()中被初始化的：
      for (auto &controller : controller_list_)
      { // controller_list_ 中是lat+lon controller 两个 或者MPC controller 一个
        ADEBUG << "controller:" << controller->Name() << " processing ...";
        double start_timestamp = Clock::NowInSeconds();
        // 前面已经实例化好了控制器对象，这里调用对象的成员函数进行处理。controller_agent是进行一些部署工作，进行控制器的注册，调用等
        // 这个cmd是指针传入，三个控制器共同操作同一块内存，cmd这个类里面包括了三个控制器的控制命令，分别填充即可
        controller->ComputeControlCommand(localization, chassis, trajectory, cmd); // 利用多态分别计算各控制器（lat，lon，mpc controller）的控制命令
        double end_timestamp = Clock::NowInSeconds();
        const double time_diff_ms = (end_timestamp - start_timestamp) * 1000;

        ADEBUG << "controller: " << controller->Name()
               << " calculation time is: " << time_diff_ms << " ms.";
        cmd->mutable_latency_stats()->add_controller_time_ms(time_diff_ms); // 控制器对控制命令解算时间
      }
      return Status::OK();
    }

    Status ControllerAgent::Reset()
    {
      for (auto &controller : controller_list_)
      {
        ADEBUG << "controller:" << controller->Name() << " reset...";
        controller->Reset();
      }
      return Status::OK();
    }

  } // namespace control
} // namespace apollo
