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

#include "modules/common/math/mpc_osqp.h"

namespace apollo {
namespace common {
namespace math {
MpcOsqp::MpcOsqp(const Eigen::MatrixXd &matrix_a,
                 const Eigen::MatrixXd &matrix_b,
                 const Eigen::MatrixXd &matrix_q,
                 const Eigen::MatrixXd &matrix_r,
                 const Eigen::MatrixXd &matrix_initial_x,
                 const Eigen::MatrixXd &matrix_u_lower,
                 const Eigen::MatrixXd &matrix_u_upper,
                 const Eigen::MatrixXd &matrix_x_lower,
                 const Eigen::MatrixXd &matrix_x_upper,
                 const Eigen::MatrixXd &matrix_x_ref, const int max_iter,
                 const int horizon, const double eps_abs)
    : matrix_a_(matrix_a),
      matrix_b_(matrix_b),
      matrix_q_(matrix_q),
      matrix_r_(matrix_r),
      matrix_initial_x_(matrix_initial_x),
      matrix_u_lower_(matrix_u_lower),
      matrix_u_upper_(matrix_u_upper),
      matrix_x_lower_(matrix_x_lower),
      matrix_x_upper_(matrix_x_upper),
      matrix_x_ref_(matrix_x_ref),
      max_iteration_(max_iter),
      horizon_(horizon),
      eps_abs_(eps_abs) {
  state_dim_ = matrix_b.rows();
  control_dim_ = matrix_b.cols();
  ADEBUG << "state_dim" << state_dim_;
  ADEBUG << "control_dim_" << control_dim_;
  num_param_ = state_dim_ * (horizon_ + 1) + control_dim_ * horizon_;
}

void MpcOsqp::CalculateKernel(std::vector<c_float> *P_data,
                              std::vector<c_int> *P_indices,
                              std::vector<c_int> *P_indptr) {
  // col1:(row,val),...; col2:(row,val),....; ...
  std::vector<std::vector<std::pair<c_int, c_float>>> columns;
  //需要先再纸上写出矩阵P，再构造相同列数的columns二维向量存放矩阵P中各列的非零元素的位置和值信息，遍历columns的列和行将P中各列的非零元素的位置和值信息存放到P_indices和P_data，再后面可根据P_data，P_indices，P_indptr把data->P中非0元素修改从而获得data->P
  columns.resize(num_param_);//columns的列数和原矩阵P的列一致，columns是为了保存矩阵P内非0元素的位置和值信息
  size_t value_index = 0;
  // state and terminal state 状态部分的权重填充
  for (size_t i = 0; i <= horizon_; ++i) {
    for (size_t j = 0; j < state_dim_; ++j) {
      // (row, val)
      columns[i * state_dim_ + j].emplace_back(i * state_dim_ + j,
                                               matrix_q_(j, j));//为columns的每一列填充矩阵P相同列的非零元素，前
      ++value_index;
    }
  }
  // control 控制部分的权重填充
  const size_t state_total_dim = state_dim_ * (horizon_ + 1);
  for (size_t i = 0; i < horizon_; ++i) {
    for (size_t j = 0; j < control_dim_; ++j) {
      // (row, val)
      columns[i * control_dim_ + j + state_total_dim].emplace_back(
          state_total_dim + i * control_dim_ + j, matrix_r_(j, j));//为columns的每一列填充矩阵P相同列的非零元素，后
      ++value_index;
    }
  }
  CHECK_EQ(value_index, num_param_);

  int ind_p = 0;
  for (size_t i = 0; i < num_param_; ++i) {//每一列
    // TODO(SHU) Check this
    P_indptr->emplace_back(ind_p);
    for (const auto &row_data_pair : columns[i]) {//columns中第i列的每一行
      P_data->emplace_back(row_data_pair.second);    // val，P_data存放columns中元素，从columns的第一列开始向下，再第二列向下，再第三列向下
      P_indices->emplace_back(row_data_pair.first);  // row，
      ++ind_p;
    }
  }
  P_indptr->emplace_back(ind_p);//P_indptr[i]-P_indptr[i-1]表示第i-1列中非零元素个数
  //注意：P_indptr[i]-P_indptr[i-1]，P_indices，P_data存放了P矩阵中非零元素的行列位置信息和值信息,后面可根据P_data，P_indices，P_indptr把data->P中非0元素修改从而获得data->P，根据P_indptr[i]-P_indptr[i-1]可知道当前P_indices[i]和P_data[i]是第几列的行和值
}

// reference is always zero
void MpcOsqp::CalculateGradient() {
  // populate the gradient vector
  gradient_ = Eigen::VectorXd::Zero(
      state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
  for (size_t i = 0; i < horizon_ + 1; i++) {
    gradient_.block(i * state_dim_, 0, state_dim_, 1) =
        -1.0 * matrix_q_ * matrix_x_ref_;
  }
  ADEBUG << "Gradient_mat";
  ADEBUG << gradient_;
}

// equality constraints x(k+1) = A*x(k)
void MpcOsqp::CalculateEqualityConstraint(std::vector<c_float> *A_data,
                                          std::vector<c_int> *A_indices,
                                          std::vector<c_int> *A_indptr) {
  static constexpr double kEpsilon = 1e-6;
  // block matrix
  Eigen::MatrixXd matrix_constraint = Eigen::MatrixXd::Zero(
      state_dim_ * (horizon_ + 1) + state_dim_ * (horizon_ + 1) +
          control_dim_ * horizon_,
      state_dim_ * (horizon_ + 1) + control_dim_ * horizon_);//创建一个Ac矩阵
  Eigen::MatrixXd state_identity_mat = Eigen::MatrixXd::Identity(
      state_dim_ * (horizon_ + 1), state_dim_ * (horizon_ + 1));
  ADEBUG << "state_identity_mat" << state_identity_mat;

  matrix_constraint.block(0, 0, state_dim_ * (horizon_ + 1),
                          state_dim_ * (horizon_ + 1)) =
      -1 * state_identity_mat;//填充Ac中Ex块中负单位矩阵
  ADEBUG << "matrix_constraint";
  ADEBUG << matrix_constraint;

  Eigen::MatrixXd control_identity_mat =
      Eigen::MatrixXd::Identity(control_dim_, control_dim_);

  for (size_t i = 0; i < horizon_; i++) {
    matrix_constraint.block((i + 1) * state_dim_, i * state_dim_, state_dim_,
                            state_dim_) = matrix_a_;//填充Ac中Ex块中A矩阵
  }
  ADEBUG << "matrix_constraint with A";
  ADEBUG << matrix_constraint;

  for (size_t i = 0; i < horizon_; i++) {
    matrix_constraint.block((i + 1) * state_dim_,
                            i * control_dim_ + (horizon_ + 1) * state_dim_,
                            state_dim_, control_dim_) = matrix_b_;//填充Ac中Eu块中B矩阵
  }
  ADEBUG << "matrix_constraint with B";
  ADEBUG << matrix_constraint;

  Eigen::MatrixXd all_identity_mat =
      Eigen::MatrixXd::Identity(num_param_, num_param_);

  matrix_constraint.block(state_dim_ * (horizon_ + 1), 0, num_param_,
                          num_param_) = all_identity_mat;//填充IEx和IEu块中单位矩阵
  ADEBUG << "matrix_constraint with I";
  ADEBUG << matrix_constraint;
//需要先再纸上写出矩阵Ac，再构造相同列数的columns二维向量存放矩阵Ac中各列的非零元素的位置和值信息，遍历columns的列和行将Ac中各列的非零元素的位置和值信息存放到A_indices和A_data，再后面可根据A_data，A_indices，A_indptr把data->A中非0元素修改从而获得data->A
  std::vector<std::vector<std::pair<c_int, c_float>>> columns;//其实也是多维的
  columns.resize(num_param_ + 1);
  int value_index = 0;
  // state and terminal state
  for (size_t i = 0; i < num_param_; ++i) {  // col
    for (size_t j = 0; j < num_param_ + state_dim_ * (horizon_ + 1);
         ++j)  // row
      if (std::fabs(matrix_constraint(j, i)) > kEpsilon) {//如果Ac矩阵元素绝对值大于0，存在不为零元素
        // (row, val)
        columns[i].emplace_back(j, matrix_constraint(j, i));//为节省空间，将Ac矩阵转化为二位向量columns，列中的零元素被去除了
        ++value_index;
      }
  }
  ADEBUG << "value_index";
  ADEBUG << value_index;
  int ind_A = 0;
  for (size_t i = 0; i < num_param_; ++i) {//每一列
    A_indptr->emplace_back(ind_A);
    for (const auto &row_data_pair : columns[i]) {//每一列每一行
      A_data->emplace_back(row_data_pair.second);    // value，A_data存放columns中元素，从columns的第一列开始向下，再第二列向下
      A_indices->emplace_back(row_data_pair.first);  // row
      ++ind_A;//ind_A和A_indices是完全不一样的，因为A_indices记录的是Ac矩阵中元素的行，而ind_A是元素个数
    }
  }

   //注意：A_indptr[i]-A_indptr[i-1]，A_indices，A_data存放了Ac矩阵中非零元素的行列位置信息和值信息,后面可根据A_data，A_indices，A_indptr把data->A中非0元素修改从而获得data->A，根据A_indptr[i]-A_indptr[i-1]可知道当前A_indices[i]和A_data[i]是第几列的行和值
  A_indptr->emplace_back(ind_A);
}

void MpcOsqp::CalculateConstraintVectors() {
  // evaluate the lower and the upper inequality vectors
  Eigen::VectorXd lowerInequality = Eigen::MatrixXd::Zero(
      state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
  Eigen::VectorXd upperInequality = Eigen::MatrixXd::Zero(
      state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
  for (size_t i = 0; i < horizon_; i++) {
    //matrix_u_lower_为序列中个控制量的约束，需要被拓展
    lowerInequality.block(control_dim_ * i + state_dim_ * (horizon_ + 1), 0,
                          control_dim_, 1) = matrix_u_lower_;
    upperInequality.block(control_dim_ * i + state_dim_ * (horizon_ + 1), 0,
                          control_dim_, 1) = matrix_u_upper_;
  }
  ADEBUG << " matrix_u_lower_";
  //matrix_x_lower_为序列中个状态量的约束，需要被拓展
  for (size_t i = 0; i < horizon_ + 1; i++) {
    lowerInequality.block(state_dim_ * i, 0, state_dim_, 1) = matrix_x_lower_;
    upperInequality.block(state_dim_ * i, 0, state_dim_, 1) = matrix_x_upper_;
  }
  ADEBUG << " matrix_x_lower_";

  // evaluate the lower and the upper equality vectors
  Eigen::VectorXd lowerEquality =
      Eigen::MatrixXd::Zero(state_dim_ * (horizon_ + 1), 1);
  Eigen::VectorXd upperEquality;
  lowerEquality.block(0, 0, state_dim_, 1) = -1 * matrix_initial_x_;//第一个状态量为-x0,x0为传感器获得的当前状态
  upperEquality = lowerEquality;
  lowerEquality = lowerEquality;//等式约束部分，上下边界一样
  ADEBUG << " matrix_initial_x_";

  // merge inequality and equality vectors
  lowerBound_ = Eigen::MatrixXd::Zero(
      2 * state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
  lowerBound_ << lowerEquality, lowerInequality;
  ADEBUG << " lowerBound_ ";
  upperBound_ = Eigen::MatrixXd::Zero(
      2 * state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
  upperBound_ << upperEquality, upperInequality;
  ADEBUG << " upperBound_";
}

OSQPSettings *MpcOsqp::Settings() {
  // default setting
  OSQPSettings *settings =
      reinterpret_cast<OSQPSettings *>(c_malloc(sizeof(OSQPSettings)));
  if (settings == nullptr) {
    return nullptr;
  } else {
    osqp_set_default_settings(settings);
    settings->polish = true;
    settings->scaled_termination = true;
    settings->verbose = false;
    settings->max_iter = max_iteration_;
    settings->eps_abs = eps_abs_;
    return settings;
  }
}

OSQPData *MpcOsqp::Data() {
  OSQPData *data = reinterpret_cast<OSQPData *>(c_malloc(sizeof(OSQPData)));
  size_t kernel_dim = state_dim_ * (horizon_ + 1) + control_dim_ * horizon_;
  size_t num_affine_constraint =
      2 * state_dim_ * (horizon_ + 1) + control_dim_ * horizon_;
  if (data == nullptr) {
    return nullptr;
  } else {
    data->n = kernel_dim;//待求解向量的维度，解向量x中包括了预测状态和预测控制量，最后只取第一组控制量作为真实控制量。查看文档即可
    data->m = num_affine_constraint;//不等式约束个数+等式约束个数
    std::vector<c_float> P_data;
    std::vector<c_int> P_indices;
    std::vector<c_int> P_indptr;
    ADEBUG << "before CalculateKernel";
    CalculateKernel(&P_data, &P_indices, &P_indptr);//计算存放P矩阵非零元素的位置信息和值信息，从第一列向下开始，再第二列向下，再第三列向下，一直下去
    ADEBUG << "CalculateKernel done";
    data->P =
        csc_matrix(kernel_dim, kernel_dim, P_data.size(), CopyData(P_data),
                   CopyData(P_indices), CopyData(P_indptr));//根据P矩阵非零元素的位置信息和值信息构造data->P
    ADEBUG << "Get P matrix";
    data->q = gradient_.data();
    ADEBUG << "before CalculateEqualityConstraint";
    std::vector<c_float> A_data;
    std::vector<c_int> A_indices;
    std::vector<c_int> A_indptr;
    CalculateEqualityConstraint(&A_data, &A_indices, &A_indptr);//计算存放Ac矩阵非零元素的位置信息和值信息，从第一列向下开始，再第二列向下，再第三列向下，一直下去
    ADEBUG << "CalculateEqualityConstraint done";
    data->A =
        csc_matrix(state_dim_ * (horizon_ + 1) + state_dim_ * (horizon_ + 1) +
                       control_dim_ * horizon_,
                   kernel_dim, A_data.size(), CopyData(A_data),
                   CopyData(A_indices), CopyData(A_indptr));//根据Ac矩阵非零元素的位置信息和值信息构造data->A
    ADEBUG << "Get A matrix";
    data->l = lowerBound_.data();//l下边界
    data->u = upperBound_.data();//u上边界
    return data;
  }
}

void MpcOsqp::FreeData(OSQPData *data) {
  c_free(data->A);
  c_free(data->P);
  c_free(data);
}

bool MpcOsqp::Solve(std::vector<double> *control_cmd) {
  ADEBUG << "Before Calc Gradient";
  //计算梯度
  CalculateGradient();
  ADEBUG << "After Calc Gradient";
  //计算约束向量,为了后面data->l和data->u填充
  CalculateConstraintVectors();
  ADEBUG << "MPC2Matrix";
  //对QP的参数矩阵计算和填充，如P，Ac，l，u
  OSQPData *data = Data();
  ADEBUG << "OSQP data done";
  ADEBUG << "OSQP data n" << data->n;
  ADEBUG << "OSQP data m" << data->m;
  for (int i = 0; i < data->n; ++i) {
    ADEBUG << "OSQP data q" << i << ":" << (data->q)[i];
  }
  ADEBUG << "OSQP data l" << data->l;
  for (int i = 0; i < data->m; ++i) {
    ADEBUG << "OSQP data l" << i << ":" << (data->l)[i];
  }
  ADEBUG << "OSQP data u" << data->u;
  for (int i = 0; i < data->m; ++i) {
    ADEBUG << "OSQP data u" << i << ":" << (data->u)[i];
  }

  OSQPSettings *settings = Settings();
  ADEBUG << "OSQP setting done";
  OSQPWorkspace *osqp_workspace = nullptr;
  // osqp_setup(&osqp_workspace, data, settings);
  osqp_workspace = osqp_setup(data, settings);
  ADEBUG << "OSQP workspace ready";
  osqp_solve(osqp_workspace);//最后所求解的x维度为state_dim_ * (horizon_ + 1) + control_dim_ * horizon_，包括了预测状态和预测控制量

  auto status = osqp_workspace->info->status_val;
  ADEBUG << "status:" << status;
  // check status
  if (status < 0 || (status != 1 && status != 2)) {
    AERROR << "failed optimization status:\t" << osqp_workspace->info->status;
    osqp_cleanup(osqp_workspace);
    FreeData(data);
    c_free(settings);
    return false;
  } else if (osqp_workspace->solution == nullptr) {
    AERROR << "The solution from OSQP is nullptr";
    osqp_cleanup(osqp_workspace);
    FreeData(data);
    c_free(settings);
    return false;
  }

  size_t first_control = state_dim_ * (horizon_ + 1);
  for (size_t i = 0; i < control_dim_; ++i) {//control_dim_=2 包括前轮转角和前进加速度
    control_cmd->at(i) = osqp_workspace->solution->x[i + first_control];
    ADEBUG << "control_cmd:" << i << ":" << control_cmd->at(i);//取解向量x中的第一组控制量作为最终控制量，这里就是控制量，不是控制增量！
  }

  // Cleanup
  osqp_cleanup(osqp_workspace);
  FreeData(data);
  c_free(settings);
  return true;
}

}  // namespace math
}  // namespace common
}  // namespace apollo
