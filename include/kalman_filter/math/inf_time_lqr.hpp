/**
 * @File: inf_time_lqr.hpp
 * @Date: July 2023
 * @Author: James Swedeen
 *
 * @brief
 * Defines functions used to solve the infinite time LQR problem.
 *
 * @Cite
 * This implementation barrows code from ethz-adrl/control-toolbox.
 * See https://github.com/ethz-adrl/control-toolbox/blob/v3.0.2/ct_optcon/include/ct/optcon/lqr/riccati/CARE-impl.hpp
 **/

#ifndef KALMAN_FILTER_MATH_INF_TIME_LQR_HPP
#define KALMAN_FILTER_MATH_INF_TIME_LQR_HPP

/* C++ Headers */
#include<execution>

/* Eigen Headers */
#include<Eigen/Dense>

// Schur reordering from Lapack
#ifndef EIGEN_USE_LAPACKE
extern "C" void dtrsen_(const char* JOB,
                        const char* COMPQ,
                        const int* SELECT,
                        const int* N,
                        const double* T,
                        const int* LDT,
                        const double* Q,
                        const int* LDQ,
                        double* WR,
                        double* WI,
                        int* M,
                        double* S,
                        double* SEP,
                        double* WORK,
                        const int* LWORK,
                        int* IWORK,
                        const int* LIWORK,
                        int* INFO);
#endif

namespace kf
{
namespace math
{
/**
 * @InfTimeLQRMemoryBuffer
 *
 * @brief
 * An object of buffers that are used in infTimeLQR.
 **/
struct InfTimeLQRMemoryBuffer
{
public:
  int LWORK_;
  int LIWORK_;
  Eigen::VectorXd WORK_;
  Eigen::VectorXi IWORK_;
};
/**
 * @generateInfTimeLQRMemoryBuffer
 *
 * @brief
 * Helper function used to generate the memory buffer for infTimeLQR.
 **/
template<Eigen::Index STATE_DIM, Eigen::Index CONTROL_DIM>
inline InfTimeLQRMemoryBuffer generateInfTimeLQRMemoryBuffer();
/**
 * @infTimeLQR
 *
 * @brief
 * Used to solve the infinite time LQR problem.
 *
 * @templates
 * STATE_DYNAMICS_DERIVED: The matrix type for a n by n matrix where n is the number of states
 * CONTROL_MATRIX_DERIVED: The matrix type of the control matrix n by u where u is the number of controls
 * STATE_COST_DERIVED: The matrix type for the state cost matrix n by n
 * CONTROL_COST_DERIVED: The matrix type for the control cost matrix u by u
 *
 * @parameters
 * state_dynamics: The state dynamics matrix
 * control: The control matrix
 * state_cost: The state cost matrix
 * control_cost: The control cost matrix
 * memory_buffer: Buffer used in the internal computation
 *
 * @return
 * The optimal feedback control gain.
 **/
template<Eigen::Index          STATE_DIM,
         Eigen::Index          CONTROL_DIM,
         typename              SCALAR,
         Eigen::StorageOptions OPTIONS,
         typename              STATE_DYNAMICS_DERIVED,
         typename              CONTROL_MATRIX_DERIVED,
         typename              STATE_COST_DERIVED,
         typename              CONTROL_COST_DERIVED>
inline Eigen::Matrix<SCALAR,CONTROL_DIM,STATE_DIM,OPTIONS>
  infTimeLQR(const Eigen::MatrixBase<STATE_DYNAMICS_DERIVED>& state_dynamics,
             const Eigen::MatrixBase<CONTROL_MATRIX_DERIVED>& control,
             const Eigen::MatrixBase<STATE_COST_DERIVED>&     state_cost,
             const Eigen::MatrixBase<CONTROL_COST_DERIVED>&   control_cost);
template<Eigen::Index          STATE_DIM,
         Eigen::Index          CONTROL_DIM,
         typename              SCALAR,
         Eigen::StorageOptions OPTIONS,
         typename              STATE_DYNAMICS_DERIVED,
         typename              CONTROL_MATRIX_DERIVED,
         typename              STATE_COST_DERIVED,
         typename              CONTROL_COST_DERIVED>
inline Eigen::Matrix<SCALAR,CONTROL_DIM,STATE_DIM,OPTIONS>
  infTimeLQR(const Eigen::MatrixBase<STATE_DYNAMICS_DERIVED>& state_dynamics,
             const Eigen::MatrixBase<CONTROL_MATRIX_DERIVED>& control,
             const Eigen::MatrixBase<STATE_COST_DERIVED>&     state_cost,
             const Eigen::MatrixBase<CONTROL_COST_DERIVED>&   control_cost,
             InfTimeLQRMemoryBuffer&                          memory_buffer);
} // namespace math

template<Eigen::Index STATE_DIM, Eigen::Index CONTROL_DIM>
inline math::InfTimeLQRMemoryBuffer math::generateInfTimeLQRMemoryBuffer()
{
  InfTimeLQRMemoryBuffer output;

  Eigen::Matrix<double,2*STATE_DIM,2*STATE_DIM,Eigen::ColMajor> U;
  Eigen::Matrix<double,2*STATE_DIM,2*STATE_DIM,Eigen::ColMajor> T;
  int SELECT[2 * STATE_DIM];
  int N = 2 * STATE_DIM;
  double WR[2*STATE_DIM];
  double WI[2*STATE_DIM];
  int MS;
  double S;
  double SEP;
  double WORKDUMMY[1];
  int LWORK = -1;
  int IWORKQUERY[1];
  int LIWORK = -1;
  int INFO = 0;
  int TCols = 2*STATE_DIM;
  char JOB = 'N';
  char COMPQ = 'V';

  // Find the optimal work size of schur reordering
  dtrsen_(&JOB, &COMPQ, &SELECT[0], &TCols, T.data(), &N, U.data(), &N, &WR[0], &WI[0], &MS, &S, &SEP, WORKDUMMY, &LWORK,
          &IWORKQUERY[0], &LIWORK, &INFO);
  assert(0 == INFO);

  output.LWORK_ = WORKDUMMY[0] + 32;
  output.LIWORK_ = IWORKQUERY[0] + 32;

  output.WORK_.resize(output.LWORK_);
  output.IWORK_.resize(output.LIWORK_);

  return output;
}

template<Eigen::Index          STATE_DIM,
         Eigen::Index          CONTROL_DIM,
         typename              SCALAR,
         Eigen::StorageOptions OPTIONS,
         typename              STATE_DYNAMICS_DERIVED,
         typename              CONTROL_MATRIX_DERIVED,
         typename              STATE_COST_DERIVED,
         typename              CONTROL_COST_DERIVED>
inline Eigen::Matrix<SCALAR,CONTROL_DIM,STATE_DIM,OPTIONS>
  math::infTimeLQR(const Eigen::MatrixBase<STATE_DYNAMICS_DERIVED>& state_dynamics,
                   const Eigen::MatrixBase<CONTROL_MATRIX_DERIVED>& control,
                   const Eigen::MatrixBase<STATE_COST_DERIVED>&     state_cost,
                   const Eigen::MatrixBase<CONTROL_COST_DERIVED>&   control_cost)
{
  return infTimeLQR<STATE_DIM,CONTROL_DIM,SCALAR,OPTIONS>(state_dynamics, control, state_cost, control_cost, generateInfTimeLQRMemoryBuffer<STATE_DIM,CONTROL_DIM>());
}

template<Eigen::Index          STATE_DIM,
         Eigen::Index          CONTROL_DIM,
         typename              SCALAR,
         Eigen::StorageOptions OPTIONS,
         typename              STATE_DYNAMICS_DERIVED,
         typename              CONTROL_MATRIX_DERIVED,
         typename              STATE_COST_DERIVED,
         typename              CONTROL_COST_DERIVED>
inline Eigen::Matrix<SCALAR,CONTROL_DIM,STATE_DIM,OPTIONS>
  math::infTimeLQR(const Eigen::MatrixBase<STATE_DYNAMICS_DERIVED>& state_dynamics,
                   const Eigen::MatrixBase<CONTROL_MATRIX_DERIVED>& control,
                   const Eigen::MatrixBase<STATE_COST_DERIVED>&     state_cost,
                   const Eigen::MatrixBase<CONTROL_COST_DERIVED>&   control_cost,
                   InfTimeLQRMemoryBuffer&                          memory_buffer)
{
  static_assert((int(STATE_DYNAMICS_DERIVED::RowsAtCompileTime) == STATE_DIM)   or (int(STATE_DYNAMICS_DERIVED::RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(STATE_DYNAMICS_DERIVED::ColsAtCompileTime) == STATE_DIM)   or (int(STATE_DYNAMICS_DERIVED::ColsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(CONTROL_MATRIX_DERIVED::RowsAtCompileTime) == CONTROL_DIM) or (int(CONTROL_MATRIX_DERIVED::RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(CONTROL_MATRIX_DERIVED::ColsAtCompileTime) == STATE_DIM)   or (int(CONTROL_MATRIX_DERIVED::ColsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(STATE_COST_DERIVED::    RowsAtCompileTime) == STATE_DIM)   or (int(STATE_COST_DERIVED::    RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(STATE_COST_DERIVED::    ColsAtCompileTime) == STATE_DIM)   or (int(STATE_COST_DERIVED::    ColsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(CONTROL_COST_DERIVED::  RowsAtCompileTime) == CONTROL_DIM) or (int(CONTROL_COST_DERIVED::  RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(CONTROL_COST_DERIVED::  ColsAtCompileTime) == CONTROL_DIM) or (int(CONTROL_COST_DERIVED::  ColsAtCompileTime) == Eigen::Dynamic));
  assert(state_dynamics.rows() == STATE_DIM);
  assert(state_dynamics.cols() == STATE_DIM);
  assert(control.       rows() == CONTROL_DIM);
  assert(control.       cols() == STATE_DIM);
  assert(state_cost.    rows() == STATE_DIM);
  assert(state_cost.    cols() == STATE_DIM);
  assert(control_cost.  rows() == CONTROL_DIM);
  assert(control_cost.  cols() == CONTROL_DIM);

  // Helpers
  const typename CONTROL_COST_DERIVED::PlainMatrix control_cost_inv = control_cost.colPivHouseholderQr().inverse();

  // Find Hamiltonian
  Eigen::Matrix<double,2*STATE_DIM,2*STATE_DIM,Eigen::ColMajor> hamiltonian_mat;
  hamiltonian_mat.template topLeftCorner<    STATE_DIM,STATE_DIM>() = state_dynamics;
  hamiltonian_mat.template bottomLeftCorner< STATE_DIM,STATE_DIM>() = -state_cost;
  hamiltonian_mat.template topRightCorner<   STATE_DIM,STATE_DIM>() = -control.transpose() * control_cost_inv * control;
  hamiltonian_mat.template bottomRightCorner<STATE_DIM,STATE_DIM>() = -state_dynamics.transpose();

  // Perform Schur decomposition
  const Eigen::RealSchur<Eigen::Matrix<double,2*STATE_DIM,2*STATE_DIM,Eigen::ColMajor>> schur_solver(hamiltonian_mat);
  assert(Eigen::ComputationInfo::Success == schur_solver.info());

  Eigen::Matrix<double,2*STATE_DIM,2*STATE_DIM,Eigen::ColMajor> U = schur_solver.matrixU();
  Eigen::Matrix<double,2*STATE_DIM,2*STATE_DIM,Eigen::ColMajor> T = schur_solver.matrixT();

  // Reorder U and T entries with LAPACK
  {
    int SELECT[2 * STATE_DIM];
    int N = 2 * STATE_DIM;
    double WR[2*STATE_DIM];
    double WI[2*STATE_DIM];
    int MS;
    double S;
    double SEP;
    int INFO = 0;
    char JOB = 'N';
    char COMPQ = 'V';

    // Perform reordering
    for(size_t i = 0; i < (2 * STATE_DIM); ++i)
    {
      // Check if last row or eigenvalue is complex (2x2 block)
      if((i == ((2 * STATE_DIM) - 1)) or (std::abs(T(i + 1, i)) < 1e-12))
      {
        SELECT[i] = static_cast<int>(T(i, i) < 0);
      }
      else
      {
        // We have a complex block
        SELECT[i] = static_cast<int>(((T(i, i) + T(i + 1, i + 1)) / 2.0) < 0);
        SELECT[i + 1] = SELECT[i];
        i++;
      }
    }
    dtrsen_(&JOB, &COMPQ, &SELECT[0], &N, T.data(), &N, U.data(), &N, &WR[0], &WI[0], &MS, &S, &SEP, memory_buffer.WORK_.data(),
            &memory_buffer.LWORK_, memory_buffer.IWORK_.data(), &memory_buffer.LIWORK_, &INFO);
    assert(0 == INFO);
  }

  // Compute steady state solution
  const Eigen::Matrix<SCALAR,STATE_DIM,STATE_DIM,OPTIONS> steady_state_sol =
//    U.template block<STATE_DIM,STATE_DIM>(STATE_DIM, 0) *
//    U.template block<STATE_DIM,STATE_DIM>(0,         0).colPivHouseholderQr().inverse();
    U.template block<STATE_DIM,STATE_DIM>(0, 0).transpose().colPivHouseholderQr().solve(U.template block<STATE_DIM,STATE_DIM>(STATE_DIM, 0).transpose()).transpose();

  return -control_cost_inv * control * steady_state_sol;
}
} // namespace kf

#endif
/* inf_time_lqr.hpp */
