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

/**
 * @file
 **/

#pragma once

#include <memory>
#include <string>

#include "modules/common/status/status.h"
#include "modules/planning/common/dependency_injector.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/common_msgs/planning_msgs/planning_config.pb.h"

namespace apollo {
namespace planning {

class Task {
 public:
  explicit Task(const TaskConfig& config);

  Task(const TaskConfig& config,
       const std::shared_ptr<DependencyInjector>& injector);

  virtual ~Task() = default;

  const std::string& Name() const;

  const TaskConfig& Config() const { return config_; }

  virtual common::Status Execute(Frame* frame,
                                 ReferenceLineInfo* reference_line_info);

  virtual common::Status Execute(Frame* frame);

 protected:
  Frame* frame_ = nullptr;
  //超级重要：
  //继承后，每个子类对象都有自己的一个reference_line_info_指针，但指向的地址需要是同一个，确保都可以访问和使用这一块内存东西，使得每个任务之间有数据交互，在这快内存里面亦可保存所规划路径和速度使得我们函数外部可获得，
  //如何确保每个子类的reference_line_info_指针指向的地址需要是同一个，只需要ret = task->Execute(frame, reference_line_info)，执行各任务时，
  //传入reference_line_info这个共用内存，并且在每个子任务中reference_line_info_ = reference_line_info将各reference_line_info_指向同一块内存
  ReferenceLineInfo* reference_line_info_ = nullptr;
  TaskConfig config_;
  std::string name_;

  std::shared_ptr<DependencyInjector> injector_;
};

}  // namespace planning
}  // namespace apollo
