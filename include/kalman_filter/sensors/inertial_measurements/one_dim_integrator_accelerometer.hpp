/**
 * @File: one_dim_integrator_accelerometer.hpp
 * @Date: March 2023
 * @Author: James Swedeen
 *
 * @brief
 **/

#ifndef KALMAN_FILTER_SENSORS_INERTIAL_MEASUREMENTS_ONE_DIM_INTEGRATOR_ACCELEROMETER_HPP
#define KALMAN_FILTER_SENSORS_INERTIAL_MEASUREMENTS_ONE_DIM_INTEGRATOR_ACCELEROMETER_HPP

/* C++ Headers */
#include<cstdint>
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<kalman_filter/sensors/inertial_measurements/inertial_measurement_base.hpp>
#include<kalman_filter/dynamics/one_dim_integrator.hpp>

namespace kf
{
namespace sensors
{
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
class OneDimIntegratorAccelerometer;

template<typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using OneDimIntegratorAccelerometerPtr = std::shared_ptr<OneDimIntegratorAccelerometer<SCALAR,OPTIONS>>;

/**
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options.
 **/
template<typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class OneDimIntegratorAccelerometer
 : public InertialMeasurementBase<dynamics::OneDimIntegratorDim,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  OneDimIntegratorAccelerometer() noexcept = default;
  /**
   * @Copy Constructor
   **/
  OneDimIntegratorAccelerometer(const OneDimIntegratorAccelerometer&) noexcept = default;
  /**
   * @Move Constructor
   **/
  OneDimIntegratorAccelerometer(OneDimIntegratorAccelerometer&&) noexcept = default;
  /**
   * @Deconstructor
   **/
  ~OneDimIntegratorAccelerometer() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  OneDimIntegratorAccelerometer& operator=(const OneDimIntegratorAccelerometer&)  noexcept = default;
  OneDimIntegratorAccelerometer& operator=(      OneDimIntegratorAccelerometer&&) noexcept = default;
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
  inline Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::INER_MEAS_DIM,OPTIONS>
    getMeasurement(const SCALAR                                                                                                time,
                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::TRUTH_DIM,  OPTIONS>>&         truth_state,
                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::CONTROL_DIM,OPTIONS>>&         control_input,
                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::INER_MEAS_NOISE_DIM,OPTIONS>>& inertial_noise) override;
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
  inline Eigen::Matrix<SCALAR,dynamics::OneDimIntegratorDim::INER_MEAS_DIM,dynamics::OneDimIntegratorDim::TRUTH_DISP_DIM,OPTIONS>
    getMeasurementPDWRDispersionState(
      const SCALAR                                                                                        time,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::TRUTH_DIM,  OPTIONS>>& truth_state,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::CONTROL_DIM,OPTIONS>>& control_input) override;
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
  inline Eigen::Matrix<SCALAR,dynamics::OneDimIntegratorDim::INER_MEAS_DIM,dynamics::OneDimIntegratorDim::CONTROL_DIM,OPTIONS>
    getMeasurementPDWRControl(
      const SCALAR                                                                                        time,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::TRUTH_DIM,  OPTIONS>>& truth_state,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::CONTROL_DIM,OPTIONS>>& control_input) override;
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
  inline Eigen::Matrix<SCALAR,dynamics::OneDimIntegratorDim::INER_MEAS_DIM,dynamics::OneDimIntegratorDim::INER_MEAS_NOISE_DIM,OPTIONS>
    getMeasurementPDWRNoise(
      const SCALAR                                                                                        time,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::TRUTH_DIM,  OPTIONS>>& truth_state,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::CONTROL_DIM,OPTIONS>>& control_input) override;
};

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::INER_MEAS_DIM,OPTIONS>
  OneDimIntegratorAccelerometer<SCALAR,OPTIONS>::getMeasurement(
    const SCALAR                                                                                                /* time */,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::TRUTH_DIM,  OPTIONS>>&         truth_state,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::CONTROL_DIM,OPTIONS>>&         control_input,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::INER_MEAS_NOISE_DIM,OPTIONS>>& inertial_noise)
{
  return Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::INER_MEAS_DIM,OPTIONS>(
           (SCALAR(1) + truth_state[dynamics::OneDimIntegratorDim::TRUTH::ACCEL_SCALE_FACTOR_IND]) *
           (truth_state[dynamics::OneDimIntegratorDim::TRUTH::DISTURBANCE_ACCEL_IND] +
            control_input[dynamics::OneDimIntegratorDim::CONTROL::ACCEL_IND] +
            truth_state[dynamics::OneDimIntegratorDim::TRUTH::ACTUATOR_BIAS_IND] +
            truth_state[dynamics::OneDimIntegratorDim::TRUTH::ACCEL_BIAS_IND] +
            inertial_noise[dynamics::OneDimIntegratorDim::INER_MEAS_NOISE::ACCEL_IND]));
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,dynamics::OneDimIntegratorDim::INER_MEAS_DIM,dynamics::OneDimIntegratorDim::TRUTH_DISP_DIM,OPTIONS>
  OneDimIntegratorAccelerometer<SCALAR,OPTIONS>::getMeasurementPDWRDispersionState(
    const SCALAR                                                                                        /* time */,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::TRUTH_DIM,  OPTIONS>>& truth_state,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::CONTROL_DIM,OPTIONS>>& control_input)
{
  Eigen::Matrix<SCALAR,dynamics::OneDimIntegratorDim::INER_MEAS_DIM,dynamics::OneDimIntegratorDim::TRUTH_DISP_DIM,OPTIONS> output;

  output[dynamics::OneDimIntegratorDim::TRUTH_DISP::POS_IND] = 0;
  output[dynamics::OneDimIntegratorDim::TRUTH_DISP::VEL_IND] = 0;
  output[dynamics::OneDimIntegratorDim::TRUTH_DISP::DISTURBANCE_ACCEL_IND] = SCALAR(1) + truth_state[dynamics::OneDimIntegratorDim::TRUTH::ACCEL_SCALE_FACTOR_IND];
  output[dynamics::OneDimIntegratorDim::TRUTH_DISP::ACCEL_BIAS_IND] = SCALAR(1) + truth_state[dynamics::OneDimIntegratorDim::TRUTH::ACCEL_SCALE_FACTOR_IND];
  output[dynamics::OneDimIntegratorDim::TRUTH_DISP::ACCEL_SCALE_FACTOR_IND] =
    truth_state[dynamics::OneDimIntegratorDim::TRUTH::DISTURBANCE_ACCEL_IND] +
    control_input[dynamics::OneDimIntegratorDim::CONTROL::ACCEL_IND] +
    truth_state[dynamics::OneDimIntegratorDim::TRUTH::ACTUATOR_BIAS_IND] +
    truth_state[dynamics::OneDimIntegratorDim::TRUTH::ACCEL_BIAS_IND];
  output[dynamics::OneDimIntegratorDim::TRUTH_DISP::ACTUATOR_BIAS_IND] = SCALAR(1) + truth_state[dynamics::OneDimIntegratorDim::TRUTH::ACCEL_SCALE_FACTOR_IND];

  return output;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,dynamics::OneDimIntegratorDim::INER_MEAS_DIM,dynamics::OneDimIntegratorDim::CONTROL_DIM,OPTIONS>
  OneDimIntegratorAccelerometer<SCALAR,OPTIONS>::getMeasurementPDWRControl(
    const SCALAR                                                                                        /* time */,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::TRUTH_DIM,  OPTIONS>>& truth_state,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::CONTROL_DIM,OPTIONS>>& /* control_input */)
{
  Eigen::Matrix<SCALAR,dynamics::OneDimIntegratorDim::INER_MEAS_DIM,dynamics::OneDimIntegratorDim::CONTROL_DIM,OPTIONS> output;

  output[0] = SCALAR(1) + truth_state[dynamics::OneDimIntegratorDim::TRUTH::ACCEL_SCALE_FACTOR_IND];

  return output;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,dynamics::OneDimIntegratorDim::INER_MEAS_DIM,dynamics::OneDimIntegratorDim::INER_MEAS_NOISE_DIM,OPTIONS>
  OneDimIntegratorAccelerometer<SCALAR,OPTIONS>::getMeasurementPDWRNoise(
    const SCALAR                                                                                        /* time */,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::TRUTH_DIM,  OPTIONS>>& truth_state,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::CONTROL_DIM,OPTIONS>>& /* control_input */)
{
  Eigen::Matrix<SCALAR,dynamics::OneDimIntegratorDim::INER_MEAS_DIM,dynamics::OneDimIntegratorDim::INER_MEAS_NOISE_DIM,OPTIONS> output;

  output[0] = SCALAR(1) + truth_state[dynamics::OneDimIntegratorDim::TRUTH::ACCEL_SCALE_FACTOR_IND];

  return output;
}
} // namespace sensors
} // namespace kf

#endif
/* one_dim_integrator_accelerometer.hpp */
