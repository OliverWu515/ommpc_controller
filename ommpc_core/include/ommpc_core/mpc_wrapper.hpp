#ifndef OMMPC_CORE_MPC_WRAPPER_HPP
#define OMMPC_CORE_MPC_WRAPPER_HPP

#include "ommpc_core/types.hpp"

#include <Eigen/Sparse>
#include <osqp/osqp.h>

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <vector>

namespace ommpc_core
{

struct Solution
{
  std::vector<Eigen::VectorXd> delta_u;  // optimal control sequence
  std::vector<Eigen::VectorXd> delta_x;  // optimal state sequence
  double optimal_cost = 0.0;
};

class MpcWrapper
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MpcWrapper()
  {
    // variables: [δx0, δu0, δx1, δu1, ..., δx{nstep-1}, δu{nstep-1}, δx{nstep}]
    total_vars_ = (kDefaultHorizonSteps + 1) * kErrorStateDim + kDefaultHorizonSteps * kInputDim;
    // constraints: error dynamics (nstep) + init (1) + control bounds (nstep)
    total_constraints_ =
        kDefaultHorizonSteps * kErrorStateDim + kErrorStateDim + kDefaultHorizonSteps * kInputDim;
  }

  ~MpcWrapper()
  {
    free(P_data_);
    free(P_indices_);
    free(P_indptr_);
  }

  void setInitValue(const Eigen::VectorXd &x0)
  {
    for (int i = 0; i < kErrorStateDim; ++i)
    {
      l_[i] = x0[i];
      u_[i] = x0[i];
    }
  }

  void setDesiredStart(const Eigen::VectorXd &xdes, const Eigen::VectorXd &udes)
  {
    x_des_start_ = xdes;
    u_des_start_ = udes;
  }

  void getDesiredStart(Eigen::VectorXd &xdes, Eigen::VectorXd &udes) const
  {
    xdes = x_des_start_;
    udes = u_des_start_;
  }

  bool solve(Solution &sol)
  {
    OSQPData osqp_data;
    OSQPSettings osqp_settings;
    OSQPWorkspace *work = nullptr;

    osqp_data.n = total_vars_;
    osqp_data.m = total_constraints_;
    osqp_data.P = csc_matrix(total_vars_, total_vars_, P_nnz_, P_data_, P_indices_, P_indptr_);
    osqp_data.q = q_.data();
    osqp_data.A = csc_matrix(total_constraints_, total_vars_, A_nnz_, A_data_, A_indices_, A_indptr_);
    osqp_data.l = l_.data();
    osqp_data.u = u_.data();

    osqp_set_default_settings(&osqp_settings);
    osqp_settings.polish = true;
    osqp_settings.verbose = false;

    c_int exit_code = osqp_setup(&work, &osqp_data, &osqp_settings);
    if (exit_code != 0)
    {
      free(A_data_);
      free(A_indices_);
      free(A_indptr_);
      A_data_ = nullptr;
      A_indices_ = nullptr;
      A_indptr_ = nullptr;
      return false;
    }

    osqp_solve(work);

    bool success = false;
    if (work != nullptr && work->info != nullptr &&
        (work->info->status_val == OSQP_SOLVED || work->info->status_val == OSQP_SOLVED_INACCURATE))
    {
      success = true;
      extractSolution(work->solution->x, sol);
      sol.optimal_cost = work->info->obj_val;
    }

    if (work != nullptr)
    {
      osqp_cleanup(work);
    }

    free(A_data_);
    free(A_indices_);
    free(A_indptr_);
    A_data_ = nullptr;
    A_indices_ = nullptr;
    A_indptr_ = nullptr;

    return success;
  }

  // Set weight matrix, only once
  void buildHessianMatrix(const Eigen::Matrix<double, kErrorStateDim, kErrorStateDim> &Q_diag,
                          const Eigen::Matrix<double, kInputDim, kInputDim> &R_diag,
                          const double state_cost_exponential,
                          const double input_cost_exponential)
  {
    P_nnz_ = total_vars_;

    P_data_ = static_cast<c_float *>(malloc(P_nnz_ * sizeof(c_float)));
    P_indices_ = static_cast<c_int *>(malloc(P_nnz_ * sizeof(c_int)));
    P_indptr_ = static_cast<c_int *>(malloc((total_vars_ + 1) * sizeof(c_int)));

    int var_idx = 0;
    for (int col = 0; col <= total_vars_; ++col)
    {
      P_indptr_[col] = col;
    }

    for (int k = 0; k < kDefaultHorizonSteps; ++k)
    {
      const double state_decay =
          std::exp(-(static_cast<double>(k) / static_cast<double>(kDefaultHorizonSteps)) *
                   state_cost_exponential);
      for (int i = 0; i < kErrorStateDim; ++i)
      {
        P_indices_[var_idx] = var_idx;
        P_data_[var_idx] = Q_diag(i, i) * state_decay;
        ++var_idx;
      }

      const double input_decay =
          std::exp(-(static_cast<double>(k) / static_cast<double>(kDefaultHorizonSteps)) *
                   input_cost_exponential);
      for (int i = 0; i < kInputDim; ++i)
      {
        P_indices_[var_idx] = var_idx;
        P_data_[var_idx] = R_diag(i, i) * input_decay;
        ++var_idx;
      }
    }

    const Eigen::Matrix<double, kErrorStateDim, kErrorStateDim> P_final_diag =
        Q_diag * std::exp(-state_cost_exponential);

    for (int i = 0; i < kErrorStateDim; ++i)
    {
      P_indices_[var_idx] = var_idx;
      P_data_[var_idx] = P_final_diag(i, i);
      ++var_idx;
    }
  }

  void buildConstraintMatrix(const std::vector<Eigen::SparseMatrix<double>> &Fx,
                             const std::vector<Eigen::SparseMatrix<double>> &Fu)
  {
    // Calculate the number of non-zero elements
    A_nnz_ = kErrorStateDim;  // Initial condition constraints
    // non-zero elements of dynamics constraints
    for (int k = 0; k < kDefaultHorizonSteps; ++k)
    {
      A_nnz_ += Fx[k].nonZeros() + Fu[k].nonZeros() + kErrorStateDim;
    }
    // non-zero elements of control constraint (each constraint has 1 non-zero element)
    A_nnz_ += kDefaultHorizonSteps * kInputDim;

    A_data_ = static_cast<c_float *>(malloc(A_nnz_ * sizeof(c_float)));
    A_indices_ = static_cast<c_int *>(malloc(A_nnz_ * sizeof(c_int)));
    A_indptr_ = static_cast<c_int *>(malloc((total_vars_ + 1) * sizeof(c_int)));

    // First, count non-zero elements per column
    // Initial condition constraints: columns corresponding to δx0
    std::vector<int> col_nnz(total_vars_, 0);
    for (int i = 0; i < kErrorStateDim; ++i)
    {
      ++col_nnz[i];
    }

    // Dynamics constraints
    int constraint_idx = kErrorStateDim;
    for (int k = 0; k < kDefaultHorizonSteps; ++k)
    {
      const int xk_offset = k * (kErrorStateDim + kInputDim);
      const int uk_offset = xk_offset + kErrorStateDim;
      const int xkp1_offset = (k + 1) * (kErrorStateDim + kInputDim);

      // -Fx[k] part (corresponding to δxk)
      for (int j = 0; j < Fx[k].outerSize(); ++j)
      {
        for (Eigen::SparseMatrix<double>::InnerIterator it(Fx[k], j); it; ++it)
        {
          ++col_nnz[xk_offset + it.col()];
        }
      }

      // -Fu[k] part (corresponding to δuk)
      for (int j = 0; j < Fu[k].outerSize(); ++j)
      {
        for (Eigen::SparseMatrix<double>::InnerIterator it(Fu[k], j); it; ++it)
        {
          ++col_nnz[uk_offset + it.col()];
        }
      }

      // I part (corresponding to δx{k+1})
      for (int i = 0; i < kErrorStateDim; ++i)
      {
        ++col_nnz[xkp1_offset + i];
      }

      constraint_idx += kErrorStateDim;
    }

    // Control constraints
    for (int k = 0; k < kDefaultHorizonSteps; ++k)
    {
      const int uk_offset = k * (kErrorStateDim + kInputDim) + kErrorStateDim;
      for (int i = 0; i < kInputDim; ++i)
      {
        ++col_nnz[uk_offset + i];
      }
    }

    A_indptr_[0] = 0;
    for (int col = 0; col < total_vars_; ++col)
    {
      A_indptr_[col + 1] = A_indptr_[col] + col_nnz[col];
    }

    // End of counting the number of non-zero elements per column
    // Next fill in data
    std::vector<int> col_pos(total_vars_, 0);
    std::vector<c_float> temp_data(A_nnz_);
    std::vector<c_int> temp_indices(A_nnz_);

    constraint_idx = 0;
    for (int i = 0; i < kErrorStateDim; ++i)
    {
      const int pos = A_indptr_[i] + col_pos[i];
      temp_data[pos] = 1.0;
      temp_indices[pos] = constraint_idx++;
      ++col_pos[i];
    }

    // Dynamics constraints
    for (int k = 0; k < kDefaultHorizonSteps; ++k)
    {
      const int xk_offset = k * (kErrorStateDim + kInputDim);
      const int uk_offset = xk_offset + kErrorStateDim;
      const int xkp1_offset = (k + 1) * (kErrorStateDim + kInputDim);

      // -Fx[k]
      for (int j = 0; j < Fx[k].outerSize(); ++j)
      {
        for (Eigen::SparseMatrix<double>::InnerIterator it(Fx[k], j); it; ++it)
        {
          const int col = xk_offset + it.col();
          const int pos = A_indptr_[col] + col_pos[col];
          temp_data[pos] = -it.value();
          temp_indices[pos] = constraint_idx + it.row();
          ++col_pos[col];
        }
      }

      // -Fu[k]
      for (int j = 0; j < Fu[k].outerSize(); ++j)
      {
        for (Eigen::SparseMatrix<double>::InnerIterator it(Fu[k], j); it; ++it)
        {
          const int col = uk_offset + it.col();
          const int pos = A_indptr_[col] + col_pos[col];
          temp_data[pos] = -it.value();
          temp_indices[pos] = constraint_idx + it.row();
          ++col_pos[col];
        }
      }

      // I
      for (int i = 0; i < kErrorStateDim; ++i)
      {
        const int col = xkp1_offset + i;
        const int pos = A_indptr_[col] + col_pos[col];
        temp_data[pos] = 1.0;
        temp_indices[pos] = constraint_idx + i;
        ++col_pos[col];
      }

      constraint_idx += kErrorStateDim;
    }

    // Control constraints
    for (int k = 0; k < kDefaultHorizonSteps; ++k)
    {
      const int uk_offset = k * (kErrorStateDim + kInputDim) + kErrorStateDim;
      for (int i = 0; i < kInputDim; ++i)
      {
        const int col = uk_offset + i;
        const int pos = A_indptr_[col] + col_pos[col];
        temp_data[pos] = 1.0;
        temp_indices[pos] = constraint_idx++;
        ++col_pos[col];
      }
    }

    std::memcpy(A_data_, temp_data.data(), A_nnz_ * sizeof(c_float));
    std::memcpy(A_indices_, temp_indices.data(), A_nnz_ * sizeof(c_int));
  }

  // Build constraint right-hand side vectors
  void buildConstraintVectors(const std::vector<Eigen::VectorXd> &u_min,
                              const std::vector<Eigen::VectorXd> &u_max)
  {
    q_.resize(total_vars_, 0.0);
    l_.resize(total_constraints_, 0.0);
    u_.resize(total_constraints_, 0.0);

    int offset = kErrorStateDim;
    std::fill(l_.begin() + offset, l_.begin() + offset + kDefaultHorizonSteps * kErrorStateDim, 0.0);
    std::fill(u_.begin() + offset, u_.begin() + offset + kDefaultHorizonSteps * kErrorStateDim, 0.0);

    offset += kDefaultHorizonSteps * kErrorStateDim;
    for (int k = 0; k < kDefaultHorizonSteps; ++k)
    {
      for (int i = 0; i < kInputDim; ++i)
      {
        l_[offset] = u_min[k][i];
        u_[offset] = u_max[k][i];
        ++offset;
      }
    }
  }

private:
  int total_vars_ = 0;
  int total_constraints_ = 0;

  c_float *P_data_ = nullptr;
  c_int *P_indices_ = nullptr;
  c_int *P_indptr_ = nullptr;
  int P_nnz_ = 0;

  c_float *A_data_ = nullptr;
  c_int *A_indices_ = nullptr;
  c_int *A_indptr_ = nullptr;
  int A_nnz_ = 0;

  std::vector<c_float> q_;
  std::vector<c_float> l_;
  std::vector<c_float> u_;

  Eigen::VectorXd x_des_start_;
  Eigen::VectorXd u_des_start_;

  void extractSolution(const c_float *solution, Solution &sol) const
  {
    sol.delta_u.resize(kDefaultHorizonSteps);
    sol.delta_x.resize(kDefaultHorizonSteps + 1);

    for (int k = 0; k < kDefaultHorizonSteps; ++k)
    {
      const int x_offset = k * (kErrorStateDim + kInputDim);
      const int u_offset = x_offset + kErrorStateDim;

      sol.delta_x[k] = Eigen::VectorXd(kErrorStateDim);
      sol.delta_u[k] = Eigen::VectorXd(kInputDim);

      for (int i = 0; i < kErrorStateDim; ++i)
      {
        sol.delta_x[k][i] = solution[x_offset + i];
      }

      for (int i = 0; i < kInputDim; ++i)
      {
        sol.delta_u[k][i] = solution[u_offset + i];
      }
    }

    const int final_offset = kDefaultHorizonSteps * (kErrorStateDim + kInputDim);
    sol.delta_x[kDefaultHorizonSteps] = Eigen::VectorXd(kErrorStateDim);
    for (int i = 0; i < kErrorStateDim; ++i)
    {
      sol.delta_x[kDefaultHorizonSteps][i] = solution[final_offset + i];
    }
  }
};

} // namespace ommpc_core

#endif
