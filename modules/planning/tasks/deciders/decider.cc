/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/tasks/deciders/decider.h"

#include <memory>

namespace apollo {
namespace planning {

Decider::Decider(const TaskConfig& config) : Task(config) {}

Decider::Decider(const TaskConfig& config,
                 const std::shared_ptr<DependencyInjector>& injector)
    : Task(config, injector) {}

apollo::common::Status Decider::Execute(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  Task::Execute(frame, reference_line_info);
  return Process(frame, reference_line_info); //注意，这里不是在定义函数了，而是在实际地调用Process函数，由于写了多态，所以这里调用的是对应task类（比如 LANE_CHANGE_DECIDER类）的Process。
}

apollo::common::Status Decider::Execute(Frame* frame) {
  Task::Execute(frame);
  return Process(frame);
}

}  // namespace planning
}  // namespace apollo
