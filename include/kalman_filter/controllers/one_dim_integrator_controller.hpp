/**
 * @File: one_dim_integrator_controller.hpp
 * @Date: March 2023
 * @Author: James Swedeen
 *
 * @brief
 * Simple feedback controller for the OneDimIntegrator dynamics class.
 **/

#ifndef KALMAN_FILTER_CONTROLLERS_ONE_DIM_INTEGRATOR_CONTROLLER_HPP
#define KALMAN_FILTER_CONTROLLERS_ONE_DIM_INTEGRATOR_CONTROLLER_HPP

/* C++ Headers */
#include<cstdint>
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<kalman_filter/controllers/controller_base.hpp>
#include<kalman_filter/dynamics/one_dim_integrator.hpp>

namespace kf
{
namespace control
{
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
class OneDimIntegratorController;

template<typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using OneDimIntegratorControllerPtr = std::shared_ptr<OneDimIntegratorController<SCALAR,OPTIONS>>;

/**
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options.
 **/
template<typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class OneDimIntegratorController
 : public ControllerBase<dynamics::OneDimIntegratorDim,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  OneDimIntegratorController() = delete;
  /**
   * @Copy Constructor
   **/
  OneDimIntegratorController(const OneDimIntegratorController&) noexcept = default;
  /**
   * @Move Constructor
   **/
  OneDimIntegratorController(OneDimIntegratorController&&) noexcept = default;
  /**
   * @Constructor
   **/
  template<typename FEEDBACK_GAIN_TYPE>
  explicit OneDimIntegratorController(const Eigen::MatrixBase<FEEDBACK_GAIN_TYPE>& feedback_gain);
  /**
   * @Deconstructor
   **/
  ~OneDimIntegratorController() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  OneDimIntegratorController& operator=(const OneDimIntegratorController&)  noexcept = default;
  OneDimIntegratorController& operator=(      OneDimIntegratorController&&) noexcept = default;
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
  inline Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::CONTROL_DIM,OPTIONS>
    getControl(const SCALAR                                                                                    time,
               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::NAV_DIM,OPTIONS>>& nav_state,
               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::REF_DIM,OPTIONS>>& ref_state,
               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::REF_DIM,OPTIONS>>& next_ref_state,
               const SCALAR                                                                                    sim_dt) override;
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
  inline Eigen::Matrix<SCALAR,dynamics::OneDimIntegratorDim::CONTROL_DIM,dynamics::OneDimIntegratorDim::ERROR_DIM,OPTIONS>
    getControlPDWRErrorState(const SCALAR                                                                                    time,
                             const Eigen::Ref<const Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::NAV_DIM,OPTIONS>>& nav_state,
                             const Eigen::Ref<const Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::REF_DIM,OPTIONS>>& ref_state) override;
private:
  const Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::NAV_DIM,OPTIONS> feedback_gain;
};


template<typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename FEEDBACK_GAIN_TYPE>
OneDimIntegratorController<SCALAR,OPTIONS>::OneDimIntegratorController(const Eigen::MatrixBase<FEEDBACK_GAIN_TYPE>& feedback_gain)
 : feedback_gain(feedback_gain)
{}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::CONTROL_DIM,OPTIONS>
  OneDimIntegratorController<SCALAR,OPTIONS>::getControl(
    const SCALAR                                                                                    /* time */,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::NAV_DIM,OPTIONS>>& nav_state,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::REF_DIM,OPTIONS>>& ref_state,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::REF_DIM,OPTIONS>>& /* next_ref_state */,
    const SCALAR                                                                                    /* sim_dt */)
{
  Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::NAV_DIM,OPTIONS> des_vec(
    Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::NAV_DIM,OPTIONS>::Zero());
  des_vec[dynamics::OneDimIntegratorDim::NAV::POS_IND] = ref_state[dynamics::OneDimIntegratorDim::REF::DISIRED_POS_IND];
  des_vec[dynamics::OneDimIntegratorDim::NAV::VEL_IND] = ref_state[dynamics::OneDimIntegratorDim::REF::DISIRED_VEL_IND];

  const Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::NAV_DIM,OPTIONS> error_vec =
    des_vec - nav_state;

  return this->feedback_gain * error_vec.transpose();
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,dynamics::OneDimIntegratorDim::CONTROL_DIM,dynamics::OneDimIntegratorDim::ERROR_DIM,OPTIONS>
  OneDimIntegratorController<SCALAR,OPTIONS>::getControlPDWRErrorState(
    const SCALAR                                                                                    /* time */,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::NAV_DIM,OPTIONS>>& /* nav_state */,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::REF_DIM,OPTIONS>>& /* ref_state */)
{
  return -this->feedback_gain;
}
} // namespace control
} // namespace kf

#endif
/* one_dim_integrator_controller.hpp */
