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
 * @file path_optimizer.h
 **/

#pragma once

#include <memory>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/common/status/status.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/tasks/task.h"

namespace apollo {
namespace planning {

class PathOptimizer : public Task {
 public:
  explicit PathOptimizer(const TaskConfig &config);
  PathOptimizer(const TaskConfig &config,
                const std::shared_ptr<DependencyInjector> &injector);
  virtual ~PathOptimizer() = default;
  apollo::common::Status Execute(
      Frame *frame, ReferenceLineInfo *reference_line_info) override;

 protected:
 //注意：reference_line地址不是传入的reference_line_info地址，而是reference_line_info中参考系的地址，因此后面将优化任务的计算结果
 //不会放在reference_line这个地址内存，而是放在reference_line_info_这个地址，因为这个地址是桥梁，进入任务后第一件事情就是
 //让各任务的reference_line_info_指针指向同一块内存，使得各任务间数据交互，外面函数可访问。如下：

  //超级重要：
  //继承后，每个子类对象都有自己的一个reference_line_info_指针，但指向的地址需要是同一个，确保都可以访问和使用这一块内存东西，使得每个任务之间有数据交互，在这快内存里面亦可保存所规划路径和速度使得我们函数外部可获得，
  //如何确保每个子类的reference_line_info_指针指向的地址需要是同一个，只需要ret = task->Execute(frame, reference_line_info)，执行各任务时，
  //传入reference_line_info这个共用内存，并且在每个子任务中reference_line_info_ = reference_line_info将各reference_line_info_指向同一块内存
 
  virtual apollo::common::Status Process(
      const SpeedData &speed_data, const ReferenceLine &reference_line,
      const common::TrajectoryPoint &init_point, const bool path_reusable,
      PathData *const path_data) = 0;

  void RecordDebugInfo(const PathData &path_data);
};

}  // namespace planning
}  // namespace apollo
