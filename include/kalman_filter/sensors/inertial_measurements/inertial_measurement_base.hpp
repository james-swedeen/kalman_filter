/**
 * @File: inertial_measurement_base.hpp
 * @Date: April 2022
 * @Author: James Swedeen
 *
 * @brief
 * A base class for inertial measurements.
 **/

#ifndef KALMAN_FILTER_SENSORS_INERTIAL_MEASUREMENTS_INERTIAL_MEASUREMENT_BASE_HPP
#define KALMAN_FILTER_SENSORS_INERTIAL_MEASUREMENTS_INERTIAL_MEASUREMENT_BASE_HPP

/* C++ Headers */
#include<cstdint>
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */

namespace kf
{
namespace sensors
{
template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
class InertialMeasurementBase;

template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using InertialMeasurementBasePtr = std::shared_ptr<InertialMeasurementBase<DIM_S,SCALAR,OPTIONS>>;

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
class InertialMeasurementBase
{
public:
  /**
   * @Default Constructor
   **/
  InertialMeasurementBase() noexcept = default;
  /**
   * @Copy Constructor
   **/
  InertialMeasurementBase(const InertialMeasurementBase&) noexcept = default;
  /**
   * @Move Constructor
   **/
  InertialMeasurementBase(InertialMeasurementBase&&) noexcept = default;
  /**
   * @Deconstructor
   **/
  virtual ~InertialMeasurementBase() noexcept = default;
  /**
   * @Assignment Operators
   **/
  InertialMeasurementBase& operator=(const InertialMeasurementBase&)  noexcept = default;
  InertialMeasurementBase& operator=(      InertialMeasurementBase&&) noexcept = default;
  /**
   * @getMeasurement
   *
   * @brief
   * Used to synthesize a measurement with noise.
   *
   * @parameters
   * time: The current simulation time
   * time_step: The time difference between the last state in the simulation and the current one
   * truth_state: The current truth state vector
   * control_input: The current control vector
   * inertial_noise: The noise for the initial measurements
   *
   * @return
   * The vector of inertial measurements.
   **/
  inline virtual Eigen::Matrix<SCALAR,1,DIM_S::INER_MEAS_DIM,OPTIONS>
    getMeasurement(const SCALAR                                                                        time,
                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,  OPTIONS>>&         truth_state,
                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS>>&         control_input,
                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::INER_MEAS_NOISE_DIM,OPTIONS>>& inertial_noise) = 0;
  /**
   * @getMeasurementPDWRDispersionState
   *
   * @brief
   * Finds the derivative of the inertial measurement function with respect to the truth dispersion state.
   *
   * @parameters
   * time: The current simulation time
   * truth_state: The current truth state vector
   * control_input: The current control vector
   *
   * @return
   * The time derivative of the truth dispersion state vector.
   **/
  inline virtual Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS>
    getMeasurementPDWRDispersionState(
      const SCALAR                                                                time,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,  OPTIONS>>& truth_state,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS>>& control_input) = 0;
  /**
   * @getMeasurementPDWRControl
   *
   * @brief
   * Finds the derivative of the inertial measurement function with respect to the control.
   *
   * @parameters
   * time: The current simulation time
   * truth_state: The current truth state vector
   * control_input: The current control vector
   *
   * @return
   * The time derivative of the control vector.
   **/
  inline virtual Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,DIM_S::CONTROL_DIM,OPTIONS>
    getMeasurementPDWRControl(
      const SCALAR                                                                time,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,  OPTIONS>>& truth_state,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS>>& control_input) = 0;
  /**
   * @getMeasurementPDWRNoise
   *
   * @brief
   * Finds the derivative of the inertial measurement function with respect to the noise vector.
   *
   * @parameters
   * time: The current simulation time
   * truth_state: The current truth state vector
   * control_input: The current control vector
   *
   * @return
   * The time derivative of the noise vector.
   **/
  inline virtual Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,DIM_S::INER_MEAS_NOISE_DIM,OPTIONS>
    getMeasurementPDWRNoise(
      const SCALAR                                                                time,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,  OPTIONS>>& truth_state,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS>>& control_input) = 0;
};
} // namespace sensors
} // namespace kf

#endif
/* inertial_measurement_base.hpp */
