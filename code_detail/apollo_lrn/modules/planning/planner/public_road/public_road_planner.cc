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

#include "modules/planning/planner/public_road/public_road_planner.h"

#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::TrajectoryPoint;

Status PublicRoadPlanner::Init(const PlanningConfig& config) { //有多态，在on_lane_planning.cc中先是父类指针planner_指向了这个子类对象，随即调用planner_->Init(config_)初始化了
  config_ = config;
  scenario_manager_.Init(config);//利用配置文件的参数来初始化，不是实例化，scenario::ScenarioManager scenario_manager_是实例化
  //里面会对场景进行初始化，因此对planner初始化的同时会对场景进行初始化。
  return Status::OK();
}

Status PublicRoadPlanner::Plan(const TrajectoryPoint& planning_start_point,
                               Frame* frame,
                               ADCTrajectory* ptr_computed_trajectory) {
  scenario_manager_.Update(planning_start_point, *frame);
  //对当前场景决策，里面包括对当前场景判别，创建场景（内部嵌套有场景的多态init()函数，在这个init()函数中又有多态createstage()对stage创建，在createstage()里面有对task_的填充）
  //也就是说，这里面更新的东西包括了场景，stage，和task。
  //关于初始化，对planner初始化的同时会对场景进行初始化，这里不是初始化，是更新场景

  //具有多场景，多stage，多任务，设计思路是利用分别写好scenario，stage，task基类，然后各个子类xx_scenario，xx_stage和xx_task分别继承，
  //在场景更新时，强化学习判别当前场景，通过switch使父类scenario指针指向该子类对象创建场景，然后根据参数文件创建父类stage指针指向第一个stage，然后为该stage根据参数文件创建父类task指针列表
  scenario_ = scenario_manager_.mutable_scenario();//获取场景
  
//在执行时，上面完成了场景更新，父类scenario指针指向该场景对象地址，调用其process函数，里面进一步，利用current_stage执行第一个stage的process函数，
//由于父类指针current_stage指向其对象时就根据参数文件创建父类task指针列表，因此利用for循环利用多态逐个执行task，执行完后当前stage结束，紧接着创建第二个stage，并利用父类指针current_stage指向指向
//由于场景不变，不会进行场景更新，因此父类指针current_stage仍然指向下一个stage，重新进入scenario的process函数执行第下一个stage，直到stage全部执行完
   auto result = scenario_->Process(planning_start_point, frame);//处理场景，进行规划，有多态，根据决策的场景进入多态

  if (FLAGS_enable_record_debug) {
    auto scenario_debug = ptr_computed_trajectory->mutable_debug()
                              ->mutable_planning_data()
                              ->mutable_scenario();
    scenario_debug->set_scenario_type(scenario_->scenario_type());
    scenario_debug->set_stage_type(scenario_->GetStage());
    scenario_debug->set_msg(scenario_->GetMsg());
  }

  if (result == scenario::Scenario::STATUS_DONE) {
    // only updates scenario manager when previous scenario's status is
    // STATUS_DONE
    scenario_manager_.Update(planning_start_point, *frame);
  } else if (result == scenario::Scenario::STATUS_UNKNOWN) {
    return Status(common::PLANNING_ERROR, "scenario returned unknown");
  }
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
