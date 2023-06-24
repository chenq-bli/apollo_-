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

/**
 * @file
 **/

#include "modules/planning/math/discretized_points_smoothing/fem_pos_deviation_osqp_interface.h"

#include <limits>

#include "cyber/common/log.h"

namespace apollo {
namespace planning {

bool FemPosDeviationOsqpInterface::Solve() {
  // Sanity Check
  if (ref_points_.empty()) {
    AERROR << "reference points empty, solver early terminates";
    return false;
  }

  if (ref_points_.size() != bounds_around_refs_.size()) {
    AERROR << "ref_points and bounds size not equal, solver early terminates";
    return false;
  }

  if (ref_points_.size() < 3) {
    AERROR << "ref_points size smaller than 3, solver early terminates";
    return false;
  }

  if (ref_points_.size() > std::numeric_limits<int>::max()) {
    AERROR << "ref_points size too large, solver early terminates";
    return false;
  }

  // Calculate optimization states definitions
  num_of_points_ = static_cast<int>(ref_points_.size());
  num_of_variables_ = num_of_points_ * 2;
  num_of_constraints_ = num_of_variables_;

  // Calculate kernel
  std::vector<c_float> P_data;
  std::vector<c_int> P_indices;
  std::vector<c_int> P_indptr;
  CalculateKernel(&P_data, &P_indices, &P_indptr);

  // Calculate affine constraints
  std::vector<c_float> A_data;
  std::vector<c_int> A_indices;
  std::vector<c_int> A_indptr;
  std::vector<c_float> lower_bounds;
  std::vector<c_float> upper_bounds;
  CalculateAffineConstraint(&A_data, &A_indices, &A_indptr, &lower_bounds,
                            &upper_bounds);

  // Calculate offset
  std::vector<c_float> q;//没错，q向量的元素是展开的代价函数中只有一个x的部分的系数，q需要写成一个向量
  CalculateOffset(&q);

  // Set primal warm start
  std::vector<c_float> primal_warm_start;
  SetPrimalWarmStart(&primal_warm_start);

  OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
  OSQPSettings* settings =
      reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));

  // Define Solver settings
  osqp_set_default_settings(settings);
  settings->max_iter = max_iter_;
  settings->time_limit = time_limit_;
  settings->verbose = verbose_;
  settings->scaled_termination = scaled_termination_;
  settings->warm_start = warm_start_;

  OSQPWorkspace* work = nullptr;

  bool res = OptimizeWithOsqp(num_of_variables_, lower_bounds.size(), &P_data,
                              &P_indices, &P_indptr, &A_data, &A_indices,
                              &A_indptr, &lower_bounds, &upper_bounds, &q,
                              &primal_warm_start, data, &work, settings);//根据以上消息，形成A，P，q等，并给到data对象，再然后利用osqp_solve(*work)求解
  if (res == false || work == nullptr || work->solution == nullptr) {
    AERROR << "Failed to find solution.";
    // Cleanup
    osqp_cleanup(work);
    c_free(data->A);
    c_free(data->P);
    c_free(data);
    c_free(settings);

    return false;
  }

  // Extract primal results
  x_.resize(num_of_points_);
  y_.resize(num_of_points_);
  for (int i = 0; i < num_of_points_; ++i) {
    int index = i * 2;
    x_.at(i) = work->solution->x[index];
    y_.at(i) = work->solution->x[index + 1];
  }

  // Cleanup
  osqp_cleanup(work);
  c_free(data->A);
  c_free(data->P);
  c_free(data);
  c_free(settings);

  return true;
}

void FemPosDeviationOsqpInterface::CalculateKernel(
    std::vector<c_float>* P_data, std::vector<c_int>* P_indices,
    std::vector<c_int>* P_indptr) {
  CHECK_GT(num_of_variables_, 4);

  // Three quadratic penalties are involved:
  // 1. Penalty x on distance between middle point and point by finite element
  // estimate;
  // 2. Penalty y on path length;
  // 3. Penalty z on difference between points and reference points

  // General formulation of P matrix is as below(with 6 points as an example):
  // I is a two by two identity matrix, X, Y, Z represents x * I, y * I, z * I
  // 0 is a two by two zero matrix
  //如何构造U=[u0,u1,u2,u3,u4,u5]T  ,其中u1=[x1,y1]T
  //仔细看看，X+Y+Z正是x0方和y0方系数，-2X-Y是x0*x1和y0*y1的系数，因此P矩阵元素位置为(2,3)的元素则代表x1*x2和y1*y2的系数
  //掌握这个规律后，把原代价函数全部展开，找规律写，第一类和第二类展开后都是这种类型，第三类展开后会出现 kx类型的，这类型不能通过构建P矩阵需要放在构建q矩阵来进行
  //先确定每种类型代价的对角线，
  // |X+Y+Z, -2X-Y,   X,       0,       0,       0    |
  // |0,     5X+2Y+Z, -4X-Y,   X,       0,       0    |
  // |0,     0,       6X+2Y+Z, -4X-Y,   X,       0    |
  // |0,     0,       0,       6X+2Y+Z, -4X-Y,   X    |
  // |0,     0,       0,       0,       5X+2Y+Z, -2X-Y|
  // |0,     0,       0,       0,       0,       X+Y+Z|

  //注意，二次型矩阵为对称矩阵，因此一定要构造成为对称矩阵(通过对角线不变，其他的同类系数项相加/2)，因此，这里P_data元素只需要记录上三角元素即可(这是系数矩阵记录方法)，但是A_data需要全部非零元素，因为其不是对称的
  // Only upper triangle needs to be filled
  std::vector<std::vector<std::pair<c_int, c_float>>> columns;
  columns.resize(num_of_variables_);
  int col_num = 0;
 //第1列
  for (int col = 0; col < 2; ++col) {
    columns[col].emplace_back(col, weight_fem_pos_deviation_ +
                                       weight_path_length_ +
                                       weight_ref_deviation_);
    ++col_num;
  }

 //第2列，这里的列是上面注释的矩阵，不是真实P矩阵的列(u是二维的)
 //注释中的-2X-Y其实是
 //    |-2X-Y,      0|
 //    |    0,  -2X-Y|
  for (int col = 2; col < 4; ++col) {
    columns[col].emplace_back(
        col - 2, -2.0 * weight_fem_pos_deviation_ - weight_path_length_);
    columns[col].emplace_back(col, 5.0 * weight_fem_pos_deviation_ +
                                       2.0 * weight_path_length_ +
                                       weight_ref_deviation_);

    //注意到columns[col].emplace_back(col, 实现columns[2].emplace_back(2,和columns[3].emplace_back(3,实现的是5X+2Y+Z部分填写，
    //columns[col].emplace_back(col - 2， 实现的是columns[2].emplace_back(0,和columns[3].emplace_back(1,的填写
    //后面填写规律是一样的，只需要columns[col].emplace_back(col-2   和   columns[col].emplace_back(col -4 即可实现相对于对角线上元素上1个和上2个元素的填写。
    
    ++col_num;
  }

  //从第3列点开始到倒数第3列
  int second_point_from_last_index = num_of_points_ - 2;
  for (int point_index = 2; point_index < second_point_from_last_index;
       ++point_index) {
    int col_index = point_index * 2;
    for (int col = 0; col < 2; ++col) {
      col_index += col;
      columns[col_index].emplace_back(col_index - 4, weight_fem_pos_deviation_);
      columns[col_index].emplace_back(
          col_index - 2,
          -4.0 * weight_fem_pos_deviation_ - weight_path_length_);
      columns[col_index].emplace_back(
          col_index, 6.0 * weight_fem_pos_deviation_ +
                         2.0 * weight_path_length_ + weight_ref_deviation_);
      ++col_num;
    }
  }
  //倒数第2列
  int second_point_col_from_last_col = num_of_variables_ - 4;
  int last_point_col_from_last_col = num_of_variables_ - 2;
  for (int col = second_point_col_from_last_col;
       col < last_point_col_from_last_col; ++col) {
    columns[col].emplace_back(col - 4, weight_fem_pos_deviation_);// col - 2和col - 4是 以对角线上元素填写基准 ，通过找规律得知的。
    columns[col].emplace_back(
        col - 2, -4.0 * weight_fem_pos_deviation_ - weight_path_length_);
    columns[col].emplace_back(col, 5.0 * weight_fem_pos_deviation_ +
                                       2.0 * weight_path_length_ +
                                       weight_ref_deviation_);
    ++col_num;
  }
//倒数第1列 
  for (int col = last_point_col_from_last_col; col < num_of_variables_; ++col) {
    columns[col].emplace_back(col - 4, weight_fem_pos_deviation_);
    columns[col].emplace_back(
        col - 2, -2.0 * weight_fem_pos_deviation_ - weight_path_length_);
    columns[col].emplace_back(col, weight_fem_pos_deviation_ +
                                       weight_path_length_ +
                                       weight_ref_deviation_);
    ++col_num;
  }

  CHECK_EQ(col_num, num_of_variables_);

  int ind_p = 0;
  for (int i = 0; i < col_num; ++i) {
    P_indptr->push_back(ind_p);
    for (const auto& row_data_pair : columns[i]) {
      // Rescale by 2.0 as the quadratic term in osqp default qp problem setup
      // is set as (1/2) * x' * P * x
      P_data->push_back(row_data_pair.second * 2.0);
      P_indices->push_back(row_data_pair.first);
      ++ind_p;
    }
  }
  P_indptr->push_back(ind_p);
}

void FemPosDeviationOsqpInterface::CalculateOffset(std::vector<c_float>* q) {
  for (int i = 0; i < num_of_points_; ++i) {
    const auto& ref_point_xy = ref_points_[i];
    q->push_back(-2.0 * weight_ref_deviation_ * ref_point_xy.first);
    q->push_back(-2.0 * weight_ref_deviation_ * ref_point_xy.second);
  }
}

void FemPosDeviationOsqpInterface::CalculateAffineConstraint(
    std::vector<c_float>* A_data, std::vector<c_int>* A_indices,
    std::vector<c_int>* A_indptr, std::vector<c_float>* lower_bounds,
    std::vector<c_float>* upper_bounds) {
  int ind_A = 0;
  for (int i = 0; i < num_of_variables_; ++i) {
    A_data->push_back(1.0);
    A_indices->push_back(i);
    A_indptr->push_back(ind_A);
    ++ind_A;
  }
  A_indptr->push_back(ind_A);

  for (int i = 0; i < num_of_points_; ++i) {
    const auto& ref_point_xy = ref_points_[i];
    upper_bounds->push_back(ref_point_xy.first + bounds_around_refs_[i]);
    upper_bounds->push_back(ref_point_xy.second + bounds_around_refs_[i]);
    lower_bounds->push_back(ref_point_xy.first - bounds_around_refs_[i]);
    lower_bounds->push_back(ref_point_xy.second - bounds_around_refs_[i]);
  }
}

void FemPosDeviationOsqpInterface::SetPrimalWarmStart(
    std::vector<c_float>* primal_warm_start) {
  CHECK_EQ(ref_points_.size(), static_cast<size_t>(num_of_points_));
  for (const auto& ref_point_xy : ref_points_) {
    primal_warm_start->push_back(ref_point_xy.first);
    primal_warm_start->push_back(ref_point_xy.second);
  }
}

bool FemPosDeviationOsqpInterface::OptimizeWithOsqp(
    const size_t kernel_dim, const size_t num_affine_constraint,
    std::vector<c_float>* P_data, std::vector<c_int>* P_indices,
    std::vector<c_int>* P_indptr, std::vector<c_float>* A_data,
    std::vector<c_int>* A_indices, std::vector<c_int>* A_indptr,
    std::vector<c_float>* lower_bounds, std::vector<c_float>* upper_bounds,
    std::vector<c_float>* q, std::vector<c_float>* primal_warm_start,
    OSQPData* data, OSQPWorkspace** work, OSQPSettings* settings) {
  CHECK_EQ(lower_bounds->size(), upper_bounds->size());

  data->n = kernel_dim;
  data->m = num_affine_constraint;
  data->P = csc_matrix(data->n, data->n, P_data->size(), P_data->data(),
                       P_indices->data(), P_indptr->data());
  data->q = q->data();
  data->A = csc_matrix(data->m, data->n, A_data->size(), A_data->data(),
                       A_indices->data(), A_indptr->data());
  data->l = lower_bounds->data();
  data->u = upper_bounds->data();

  *work = osqp_setup(data, settings);
  // osqp_setup(work, data, settings);

  osqp_warm_start_x(*work, primal_warm_start->data());

  // Solve Problem
  osqp_solve(*work);

  auto status = (*work)->info->status_val;

  if (status < 0) {
    AERROR << "failed optimization status:\t" << (*work)->info->status;
    return false;
  }

  if (status != 1 && status != 2) {
    AERROR << "failed optimization status:\t" << (*work)->info->status;
    return false;
  }

  return true;
}

}  // namespace planning
}  // namespace apollo
