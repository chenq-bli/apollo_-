/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/math/piecewise_jerk/piecewise_jerk_path_problem.h"

#include "cyber/common/log.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

PiecewiseJerkPathProblem::PiecewiseJerkPathProblem(
    const size_t num_of_knots, const double delta_s,
    const std::array<double, 3>& x_init)
    : PiecewiseJerkProblem(num_of_knots, delta_s, x_init) {}

void PiecewiseJerkPathProblem::CalculateKernel(std::vector<c_float>* P_data,
                                               std::vector<c_int>* P_indices,
                                               std::vector<c_int>* P_indptr) {
  const int n = static_cast<int>(num_of_knots_);
  const int num_of_variables = 3 * n; //待优化的变量个数，3是l，dl，dll，n是待优化区域的上一个PATH_BOUNDS_DECIDER task所离散的个数。
  const int num_of_nonzeros = num_of_variables + (n - 1);
  std::vector<std::vector<std::pair<c_int, c_float>>> columns(num_of_variables);
  int value_index = 0;

  // x(i)^2 * (w_x + w_x_ref[i]), w_x_ref might be a uniform value for all x(i)，w_x是统一x的权重，w_x_ref[i]是每一个变量的调整。
  // or piecewise values for different x(i)
  // 接下来建立目标函数，columns[i]存放目标函数，参考https://www.cnblogs.com/liuzubing/p/11051390.html
  //使用了emplace_back，在容器内部进行构造产生std::vector<std::pair<c_int, c_float>>对象
  for (int i = 0; i < n - 1; ++i) {
    columns[i].emplace_back(i, (weight_x_ + weight_x_ref_vec_[i]) /
                                   (scale_factor_[0] * scale_factor_[0]));// emplace_back（）中i是第i个优化，scale_factor_[0]是未知量0阶倒数标记，scale_factor_[1]是未知量1阶倒数标记，scale_factor_[2]是未知量2阶倒数标记，他们都是待优化量。
    ++value_index;
  }//columns[i] 也是一个vector，前面只emplace_back一次，后面2n+1到3n-1会emplace_back二次
  // x(n-1)^2 * (w_x + w_x_ref[n-1] + w_end_x) //
  columns[n - 1].emplace_back(
      n - 1, (weight_x_ + weight_x_ref_vec_[n - 1] + weight_end_state_[0]) /
                 (scale_factor_[0] * scale_factor_[0]));
  ++value_index;

  // x(i)'^2 * w_dx，这是第二部分的累加
  for (int i = 0; i < n - 1; ++i) {
    columns[n + i].emplace_back(
        n + i, weight_dx_ / (scale_factor_[1] * scale_factor_[1]));
    ++value_index;
  }
  // x(n-1)'^2 * (w_dx + w_end_dx)
  columns[2 * n - 1].emplace_back(2 * n - 1,
                                  (weight_dx_ + weight_end_state_[1]) /
                                      (scale_factor_[1] * scale_factor_[1]));
  ++value_index;

  auto delta_s_square = delta_s_ * delta_s_;
  // x(i)''^2 * (w_ddx + 2 * w_dddx / delta_s^2)，这是第三部分的累加
  //原来如此，为什么会有 2 * w_dddx和delta_s^2在columns[2 * n]到columns[3n-1],是因为目标函数最后的一部分展开了，然后进行了同类项合并
  columns[2 * n].emplace_back(2 * n,
                              (weight_ddx_ + weight_dddx_ / delta_s_square) /
                                  (scale_factor_[2] * scale_factor_[2]));
  ++value_index;
  for (int i = 1; i < n - 1; ++i) {
    columns[2 * n + i].emplace_back(
        2 * n + i, (weight_ddx_ + 2.0 * weight_dddx_ / delta_s_square) /
                       (scale_factor_[2] * scale_factor_[2]));
    ++value_index;
  }
  columns[3 * n - 1].emplace_back(
      3 * n - 1,
      (weight_ddx_ + weight_dddx_ / delta_s_square + weight_end_state_[2]) /
          (scale_factor_[2] * scale_factor_[2]));
  ++value_index;

  // -2 * w_dddx / delta_s^2 * x(i)'' * x(i + 1)''
  // x(i)'' * x(i + 1)''的出现也是因为目标函数最后的一部分展开了，这一部分没有合并
  for (int i = 0; i < n - 1; ++i) {
    columns[2 * n + i].emplace_back(2 * n + i + 1,
                                    (-2.0 * weight_dddx_ / delta_s_square) /
                                        (scale_factor_[2] * scale_factor_[2]));
    ++value_index;
  }

  CHECK_EQ(value_index, num_of_nonzeros);

  int ind_p = 0;
  for (int i = 0; i < num_of_variables; ++i) {//对每个变量遍历
    P_indptr->push_back(ind_p);
    for (const auto& row_data_pair : columns[i]) {//对每个变量对应的上面填充的vector进行遍历 
      P_data->push_back(row_data_pair.second * 2.0);
      P_indices->push_back(row_data_pair.first);//记录来自哪个变量
      ++ind_p;
    }
  }
  P_indptr->push_back(ind_p);
}

void PiecewiseJerkPathProblem::CalculateOffset(std::vector<c_float>* q) {
  CHECK_NOTNULL(q);
  const int n = static_cast<int>(num_of_knots_);
  const int kNumParam = 3 * n;
  q->resize(kNumParam, 0.0);

  if (has_x_ref_) {
    for (int i = 0; i < n; ++i) {
      q->at(i) += -2.0 * weight_x_ref_vec_.at(i) * x_ref_[i] / scale_factor_[0];
    }
  }

  if (has_end_state_ref_) {
    q->at(n - 1) +=
        -2.0 * weight_end_state_[0] * end_state_ref_[0] / scale_factor_[0];
    q->at(2 * n - 1) +=
        -2.0 * weight_end_state_[1] * end_state_ref_[1] / scale_factor_[1];
    q->at(3 * n - 1) +=
        -2.0 * weight_end_state_[2] * end_state_ref_[2] / scale_factor_[2];
  }
}

}  // namespace planning
}  // namespace apollo
