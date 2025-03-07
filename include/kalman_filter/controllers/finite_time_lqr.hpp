/**
 * @File: finite_time_lqr.hpp
 * @Date: April 2023
 * @Author: James Swedeen
 *
 * @brief
 * Implements the continuous-time dynamics finite-time horizon formulation of the Linear Quadratic Regulator (LQR)
 * controller.
 **/

#ifndef KALMAN_FILTER_CONTROLLERS_FINITE_TIME_LQR_HPP
#define KALMAN_FILTER_CONTROLLERS_FINITE_TIME_LQR_HPP

/* C++ Headers */
#include<cstdint>
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<kalman_filter/helpers/versions.hpp>
#include<kalman_filter/helpers/integrator_step.hpp>
#include<kalman_filter/mappings/mappings_base.hpp>
#include<kalman_filter/controllers/controller_base.hpp>

namespace kf
{
namespace control
{
template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
class FiniteTimeLQR;

template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using FiniteTimeLQRPtr = std::shared_ptr<FiniteTimeLQR<DIM_S,SCALAR,OPTIONS>>;

/**
 * @DIM_S
 * The type of a Dimensions object or an inheriting object that has information about the size of the state vectors.
 *
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options.
 **/
template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class FiniteTimeLQR
 : public ControllerBase<DIM_S,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  FiniteTimeLQR() = delete;
  /**
   * @Copy Constructor
   **/
  FiniteTimeLQR(const FiniteTimeLQR&) noexcept = default;
  /**
   * @Move Constructor
   **/
  FiniteTimeLQR(FiniteTimeLQR&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * Initializes the object for use.
   *
   * @parameters
   * start_time: The start time of the simulation
   * end_time: The end time of the simulation
   * time_step: The time step that the integration of P will be calculated at
   * A: The state dynamics matrix of the controlled systems dynamics
   * Bt: The transpose of the control coupling matrix of the controlled systems dynamics
   * Q: The state weighting matrix of the quadratic running cost
   * R: The control weighting matrix of the quadratic running cost
   * S: The terminal state weighting matrix of the quadratic terminal cost
   * mappings: A helper object that maps one state vector to another
   **/
  template<typename A_DERIVED,
           typename B_DERIVED,
           typename Q_DERIVED,
           typename R_DERIVED,
           typename S_DERIVED>
  FiniteTimeLQR(const SCALAR                                      start_time,
                const SCALAR                                      end_time,
                const SCALAR                                      time_step,
                const Eigen::MatrixBase<A_DERIVED>&               A,
                const Eigen::MatrixBase<B_DERIVED>&               Bt,
                const Eigen::MatrixBase<Q_DERIVED>&               Q,
                const Eigen::MatrixBase<R_DERIVED>&               R,
                const Eigen::MatrixBase<S_DERIVED>&               S,
                const map::MappingsBasePtr<DIM_S,SCALAR,OPTIONS>& mappings);
  /**
   * @Deconstructor
   **/
  ~FiniteTimeLQR() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  FiniteTimeLQR& operator=(const FiniteTimeLQR&)  noexcept = default;
  FiniteTimeLQR& operator=(      FiniteTimeLQR&&) noexcept = default;
  /**
   * @getControl
   *
   * @brief
   * Used to find the control input for this step in the simulation.
   *
   * @parameters
   * time: The current simulation time
   * nav_state: The current navigation state vector
   * ref_state: The current reference state vector
   * next_ref_state: The next reference state vector
   * sim_dt: The time delta between the current and next reference states
   *
   * @return
   * The control for this time step.
   **/
  inline Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS>
    getControl(const SCALAR                                                            time,
               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& nav_state,
               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,OPTIONS>>& ref_state,
               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,OPTIONS>>& next_ref_state,
               const SCALAR                                                            sim_dt) override;
  /**
   * @getControlPDWRErrorState
   *
   * @brief
   * Used to find the partial derivative of the control with respect to the error state.
   *
   * @parameters
   * time: The current simulation time
   * nav_state: The current navigation state vector
   * ref_state: The current reference state vector
   *
   * @return
   * The control for this time step.
   **/
  inline Eigen::Matrix<SCALAR,DIM_S::CONTROL_DIM,DIM_S::ERROR_DIM,OPTIONS>
    getControlPDWRErrorState(const SCALAR                                                            time,
                             const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& nav_state,
                             const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,OPTIONS>>& ref_state) override;
private:
  map::MappingsBasePtr<DIM_S,SCALAR,OPTIONS>                                          mappings;
  SCALAR                                                                              end_time;
  SCALAR                                                                              time_step;
  Eigen::Index                                                                        num_sim_steps;
                 Eigen::Matrix<SCALAR,DIM_S::TRUTH_DIM,  DIM_S::TRUTH_DIM,  OPTIONS>  A;
                 Eigen::Matrix<SCALAR,DIM_S::CONTROL_DIM,DIM_S::TRUTH_DIM,  OPTIONS>  Bt;
                 Eigen::Matrix<SCALAR,DIM_S::TRUTH_DIM,  DIM_S::TRUTH_DIM,  OPTIONS>  Q;
  Eigen::Inverse<Eigen::Matrix<SCALAR,DIM_S::CONTROL_DIM,DIM_S::CONTROL_DIM,OPTIONS>> R_inv;
  std::vector<   Eigen::Matrix<SCALAR,DIM_S::TRUTH_DIM,  DIM_S::TRUTH_DIM,  OPTIONS>> P_vec;
  /**
   * @pDynamics
   *
   * @brief
   * Calculates the time derivative of P.
   *
   * @parameters
   * P: The current P matrix
   *
   * @return
   * The time derivative of P.
   **/
  inline Eigen::Matrix<SCALAR,DIM_S::TRUTH_DIM,DIM_S::TRUTH_DIM,OPTIONS>
    pDynamics(const Eigen::Matrix<SCALAR,DIM_S::TRUTH_DIM,DIM_S::TRUTH_DIM,OPTIONS>& P) const noexcept;
  /**
   * @findP
   *
   * @brief
   * Finds the value of P at a specific point in time.
   *
   * @parameters
   * time: The time that P is need
   *
   * @return
   * P at the given time.
   **/
  inline Eigen::Matrix<SCALAR,DIM_S::TRUTH_DIM,DIM_S::TRUTH_DIM,OPTIONS> findP(const SCALAR time) const noexcept;
};

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename A_DERIVED,
         typename B_DERIVED,
         typename Q_DERIVED,
         typename R_DERIVED,
         typename S_DERIVED>
FiniteTimeLQR<DIM_S,SCALAR,OPTIONS>::FiniteTimeLQR(const SCALAR                                      start_time,
                                                   const SCALAR                                      end_time,
                                                   const SCALAR                                      time_step,
                                                   const Eigen::MatrixBase<A_DERIVED>&               A,
                                                   const Eigen::MatrixBase<B_DERIVED>&               Bt,
                                                   const Eigen::MatrixBase<Q_DERIVED>&               Q,
                                                   const Eigen::MatrixBase<R_DERIVED>&               R,
                                                   const Eigen::MatrixBase<S_DERIVED>&               S,
                                                   const map::MappingsBasePtr<DIM_S,SCALAR,OPTIONS>& mappings)
 : ControllerBase<DIM_S,SCALAR,OPTIONS>(),
   mappings(mappings),
   end_time(end_time),
   time_step(time_step),
   num_sim_steps(std::ceil((end_time-start_time)/time_step)),
   A(A),
   Bt(Bt),
   Q(Q),
   R_inv(R.inverse())
{
  // Check for the problem being well posed
  static_assert((int(S_DERIVED::ColsAtCompileTime) == DIM_S::TRUTH_DIM) or (int(S_DERIVED::ColsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(S_DERIVED::RowsAtCompileTime) == DIM_S::TRUTH_DIM) or (int(S_DERIVED::RowsAtCompileTime) == Eigen::Dynamic));
  assert(S.cols() == DIM_S::TRUTH_DIM);
  assert(S.rows() == DIM_S::TRUTH_DIM);
  assert(0 != R.determinant());
  assert(R.isApprox(R.transpose()));
  assert((Eigen::NumericalIssue != Eigen::LLT<R_DERIVED,Eigen::Upper>(R).info()));
  #ifndef NDEBUG
    assert(Q.isApprox(Q.transpose()));
    const Eigen::LDLT<Q_DERIVED,Eigen::Upper> Q_decomp(Q);
    assert(Eigen::NumericalIssue != Q_decomp.info());
    assert(Q_decomp.isPositive());

    assert(S.isApprox(S.transpose()));
    const Eigen::LDLT<S_DERIVED,Eigen::Upper> S_decomp(S);
    assert(Eigen::NumericalIssue != S_decomp.info());
    assert(S_decomp.isPositive());
  #endif
  assert(start_time < end_time);

  // Calculate P over time
  this->P_vec.resize(this->num_sim_steps);
  this->P_vec.back() = S;
  for(Eigen::Index time_it = this->num_sim_steps-2; time_it >= 0; --time_it)
  {
    this->P_vec[time_it] = integratorStep<Versions::ODE45_INTEGRATION,SCALAR,Eigen::Matrix<SCALAR,DIM_S::TRUTH_DIM,DIM_S::TRUTH_DIM,OPTIONS>>(
      std::bind(&FiniteTimeLQR<DIM_S,SCALAR,OPTIONS>::pDynamics, this, std::placeholders::_1),
      this->P_vec[time_it+1],
      -time_step);
    #ifndef NDEBUG
      assert(this->P_vec[time_it].isApprox(this->P_vec[time_it].transpose()));
      const Eigen::LDLT<Eigen::Matrix<SCALAR,DIM_S::TRUTH_DIM,DIM_S::TRUTH_DIM,OPTIONS>,Eigen::Upper> P_decomp(this->P_vec[time_it]);
      assert(Eigen::NumericalIssue != P_decomp.info());
      assert(P_decomp.isPositive());
    #endif
  }
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS> FiniteTimeLQR<DIM_S,SCALAR,OPTIONS>::
  getControl(const SCALAR                                                            time,
             const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& nav_state,
             const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,OPTIONS>>& ref_state,
             const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,OPTIONS>>& /* next_ref_state */,
             const SCALAR                                                            /* sim_dt */)
{
  const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS> error_vec = nav_state - this->mappings->mapRefNav(ref_state);
  return error_vec * this->getControlPDWRErrorState(time, nav_state, ref_state).transpose();
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,DIM_S::CONTROL_DIM,DIM_S::ERROR_DIM,OPTIONS> FiniteTimeLQR<DIM_S,SCALAR,OPTIONS>::
  getControlPDWRErrorState(const SCALAR                                                            time,
                           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& /* nav_state */,
                           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,OPTIONS>>& /* ref_state */)
{
  return -this->R_inv*this->Bt*this->findP(time);
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,DIM_S::TRUTH_DIM,DIM_S::TRUTH_DIM,OPTIONS> FiniteTimeLQR<DIM_S,SCALAR,OPTIONS>::
  pDynamics(const Eigen::Matrix<SCALAR,DIM_S::TRUTH_DIM,DIM_S::TRUTH_DIM,OPTIONS>& P) const noexcept
{
  return -this->A.transpose()*P - P*this->A - this->Q + P*this->Bt.transpose()*this->R_inv*this->Bt*P;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,DIM_S::TRUTH_DIM,DIM_S::TRUTH_DIM,OPTIONS> FiniteTimeLQR<DIM_S,SCALAR,OPTIONS>::
  findP(const SCALAR time) const noexcept
{
  // Find nearest point in P_vec to current time
  Eigen::Index nearest_P_diff_fe = 0;
  if(time < this->end_time)
  {
    nearest_P_diff_fe = std::min<Eigen::Index>(this->num_sim_steps-1, std::round((this->end_time - time)/this->time_step));
  }
  // Find time difference from nearest P to current time
  const SCALAR nearest_P_time = this->end_time - (SCALAR(nearest_P_diff_fe) * this->time_step);
  // Integrate P to current time
  const Eigen::Matrix<SCALAR,DIM_S::TRUTH_DIM,DIM_S::TRUTH_DIM,OPTIONS> P =
    integratorStep<Versions::ODE45_INTEGRATION,SCALAR,Eigen::Matrix<SCALAR,DIM_S::TRUTH_DIM,DIM_S::TRUTH_DIM,OPTIONS>>(
      std::bind(&FiniteTimeLQR<DIM_S,SCALAR,OPTIONS>::pDynamics, this, std::placeholders::_1),
      this->P_vec[this->num_sim_steps - 1 - nearest_P_diff_fe],
      time - nearest_P_time);
  #ifndef NDEBUG
    assert(P.isApprox(P.transpose()));
    const Eigen::LDLT<Eigen::Matrix<SCALAR,DIM_S::TRUTH_DIM,DIM_S::TRUTH_DIM,OPTIONS>,Eigen::Upper> P_decomp(P);
    assert(Eigen::NumericalIssue != P_decomp.info());
    assert(P_decomp.isPositive());
  #endif
  return P;
}
} // namespace control
} // namespace kf

#endif
/* finite_time_lqr.hpp */
