/**
 * @File: dynamics_base.hpp
 * @Date: April 2022
 * @Author: James Swedeen
 *
 * @brief
 * A base class for defining dynamic systems.
 **/

#ifndef KALMAN_FILTER_DYNAMICS_DYNAMICS_BASE_HPP
#define KALMAN_FILTER_DYNAMICS_DYNAMICS_BASE_HPP

/* C++ Headers */
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */

namespace kf
{
namespace dynamics
{
template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
class DynamicsBase;

template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using DynamicsBasePtr = std::shared_ptr<DynamicsBase<DIM_S,SCALAR,OPTIONS>>;

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
class DynamicsBase
{
public:
  /**
   * @Default Constructor
   **/
  DynamicsBase() noexcept = default;
  /**
   * @Copy Constructor
   **/
  DynamicsBase(const DynamicsBase&) noexcept = default;
  /**
   * @Move Constructor
   **/
  DynamicsBase(DynamicsBase&&) noexcept = default;
  /**
   * @Deconstructor
   **/
  virtual ~DynamicsBase() noexcept = default;
  /**
   * @Assignment Operators
   **/
  DynamicsBase& operator=(const DynamicsBase&)  noexcept = default;
  DynamicsBase& operator=(      DynamicsBase&&) noexcept = default;
  /**
   * @getTruthStateDynamics
   *
   * @brief
   * Finds the time derivative of the truth state with additive process noise.
   *
   * @parameters
   * time: The current simulation time
   * truth_state: The current truth state vector
   * control_input: The current control vector
   * process_noise: The process noise on the truth state dynamics
   *
   * @return
   * The time derivative of the truth state vector.
   **/
  inline virtual Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>
    getTruthStateDynamics(const SCALAR                                                                    time,
                          const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,      OPTIONS>>& truth_state,
                          const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,    OPTIONS>>& control_input,
                          const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_NOISE_DIM,OPTIONS>>& process_noise) = 0;
  /**
   * @getNavStateDynamics
   *
   * @brief
   * Finds the time derivative of the navigation state.
   *
   * @parameters
   * time: The current simulation time
   * nav_state: The current navigation state vector
   * ref_state: The reference state of the current time step
   * inertial_reading: The inertial measurements with biases and noise
   *
   * @return
   * The time derivative of the navigation state vector.
   **/
  inline virtual Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>
    getNavStateDynamics(const SCALAR                                                                  time,
                        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,      OPTIONS>>& nav_state,
                        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,      OPTIONS>>& ref_state,
                        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::INER_MEAS_DIM,OPTIONS>>& inertial_reading) = 0;
  /**
   * @getTruthStateDynamicsPDWRDispersionState
   *
   * @brief
   * Finds the derivative of the truth state dynamics with respect to the truth dispersion state.
   *
   * @parameters
   * time: The current simulation time
   * truth_state: The current truth state vector
   * control_input: The current control vector
   *
   * @return
   * The derivative of the truth state dynamics with respect to the truth dispersion state.
   **/
  inline virtual Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS>
    getTruthStateDynamicsPDWRDispersionState(
      const SCALAR                                                                time,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,  OPTIONS>>& truth_state,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS>>& control_input) = 0;
  /**
   * @getTruthStateDynamicsPDWRControl
   *
   * @brief
   * Finds the derivative of the truth state dynamics with respect to the control.
   *
   * @parameters
   * time: The current simulation time
   * truth_state: The current truth state vector
   * control_input: The current control vector
   *
   * @return
   * The derivative of the truth state dynamics with respect to the control.
   **/
  inline virtual Eigen::Matrix<SCALAR,DIM_S::CONTROL_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS>
    getTruthStateDynamicsPDWRControl(
      const SCALAR                                                                time,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,  OPTIONS>>& truth_state,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS>>& control_input) = 0;
  /**
   * @getTruthStateDynamicsPDWRNoise
   *
   * @brief
   * Finds the derivative of the truth state dynamics with respect to the process noise vector.
   *
   * @parameters
   * time: The current simulation time
   * truth_state: The current truth state vector
   * control_input: The current control vector
   *
   * @return
   * The derivative of the truth state dynamics with respect to the process noise vector.
   **/
  inline virtual Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_NOISE_DIM,OPTIONS>
    getTruthStateDynamicsPDWRNoise(
      const SCALAR                                                                time,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,  OPTIONS>>& truth_state,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS>>& control_input) = 0;
  /**
   * @getNavStateDynamicsPDWRErrorState
   *
   * @brief
   * Finds the derivative of the navigation state dynamics with respect to the navigation error state.
   *
   * @parameters
   * time: The current simulation time
   * nav_state: The current navigation state vector
   * ref_state: The reference state of the current time step
   * inertial_reading: The inertial measurements with biases and noise
   *
   * @return
   * The derivative of the navigation state dynamics with respect to the navigation error state.
   **/
  inline virtual Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS>
    getNavStateDynamicsPDWRErrorState(
      const SCALAR                                                                  time,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,      OPTIONS>>& nav_state,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,      OPTIONS>>& ref_state,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::INER_MEAS_DIM,OPTIONS>>& inertial_reading) = 0;
  /**
   * @getNavStateDynamicsPDWRInertialMeasurement
   *
   * @brief
   * Finds the derivative of the navigation state dynamics with respect to the inertial measurement vector.
   *
   * @parameters
   * time: The current simulation time
   * nav_state: The current navigation state vector
   * ref_state: The reference state of the current time step
   * inertial_reading: The inertial measurements with biases and noise
   *
   * @return
   * The derivative of the navigation state dynamics with respect to the navigation error state.
   **/
  inline virtual Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,DIM_S::ERROR_DIM,OPTIONS>
    getNavStateDynamicsPDWRInertialMeasurement(
      const SCALAR                                                                  time,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,      OPTIONS>>& nav_state,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,      OPTIONS>>& ref_state,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::INER_MEAS_DIM,OPTIONS>>& inertial_reading) = 0;
  /**
   * @findTimeStep
   *
   * @brief
   * Used to calculate the time that is spanned between two state vectors along the simulation vector.
   *
   * @parameters
   * prev_ref_state: The reference state of the previous time step
   * cur_ref_state: The reference state of the current time step
   *
   * @return
   * The time that is spanned between prev_ref_state and cur_ref_state in seconds.
   **/
  inline virtual SCALAR findTimeStep(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,OPTIONS>>& prev_ref_state,
                                     const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,OPTIONS>>& cur_ref_state) = 0;
};
} // namespace dynamics
} // namespace kf

#endif
/* dynamics_base.hpp */
