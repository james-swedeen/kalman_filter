/**
 * @File: open_loop_imu.hpp
 * @Date: May 2022
 * @Author: James Swedeen
 *
 * @brief
 * Defines a class for simulation a IMU unit in open loop analysis.
 **/

#ifndef KALMAN_FILTER_SENSORS_INERTIAL_MEASUREMENTS_OPEN_LOOP_IMU_HPP
#define KALMAN_FILTER_SENSORS_INERTIAL_MEASUREMENTS_OPEN_LOOP_IMU_HPP

/* C++ Headers */
#include<cstdint>
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<kalman_filter/noise/noise_base.hpp>
#include<kalman_filter/sensors/inertial_measurements/inertial_measurement_base.hpp>

namespace kf
{
namespace sensors
{
template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
class OpenLoopIMU;

template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using OpenLoopIMUPtr = std::shared_ptr<OpenLoopIMU<DIM_S,SCALAR,OPTIONS>>;

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
class OpenLoopIMU
 : public InertialMeasurementBase<DIM_S,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  OpenLoopIMU() = default;
  /**
   * @Copy Constructor
   **/
  OpenLoopIMU(const OpenLoopIMU&) = default;
  /**
   * @Move Constructor
   **/
  OpenLoopIMU(OpenLoopIMU&&) = default;
  /**
   * @Deconstructor
   **/
  ~OpenLoopIMU() override = default;
  /**
   * @Assignment Operators
   **/
  OpenLoopIMU& operator=(const OpenLoopIMU&)  = default;
  OpenLoopIMU& operator=(      OpenLoopIMU&&) = default;
  /**
   * @getMeasurement
   *
   * @brief
   * Used to synthesize a measurement with noise.
   *
   * @parameters
   * time: The current simulation time
   * truth_state: The current truth state vector
   * control_input: The current control vector
   *
   * @return
   * The vector of inertial measurements.
   **/
  inline Eigen::Matrix<SCALAR,1,6,OPTIONS>
    getMeasurement(const SCALAR                                                                        time,
                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,          OPTIONS>>& truth_state,
                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,        OPTIONS>>& control_input,
                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::INER_MEAS_NOISE_DIM,OPTIONS>>& inertial_noise) override;
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
  inline Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS>
    getMeasurementPDWRDispersionState(
      const SCALAR                                                                time,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,  OPTIONS>>& truth_state,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS>>& control_input) override;
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
  inline Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,DIM_S::CONTROL_DIM,OPTIONS>
    getMeasurementPDWRControl(
      const SCALAR                                                                time,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,  OPTIONS>>& truth_state,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS>>& control_input) override;
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
  inline Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,DIM_S::INER_MEAS_NOISE_DIM,OPTIONS>
    getMeasurementPDWRNoise(
      const SCALAR                                                                time,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,  OPTIONS>>& truth_state,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS>>& control_input) override;
};

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,6,OPTIONS> OpenLoopIMU<DIM_S,SCALAR,OPTIONS>::
  getMeasurement(const SCALAR                                                                        /* time */,
                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,          OPTIONS>>& truth_state,
                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,        OPTIONS>>& control_input,
                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::INER_MEAS_NOISE_DIM,OPTIONS>>& inertial_noise)
{
  Eigen::Matrix<SCALAR,1,6,OPTIONS> output;

  // Set nominal values
  output = control_input;
  // Add biases
  output.template middleCols<3>(DIM_S::INER_MEAS::GYRO_START_IND)  += truth_state.template middleCols<3>(DIM_S::TRUTH::GYRO_BIAS_START_IND);
  output.template middleCols<3>(DIM_S::INER_MEAS::ACCEL_START_IND) += truth_state.template middleCols<3>(DIM_S::TRUTH::ACCEL_BIAS_START_IND);
  // Add Noise
  output += inertial_noise;

  return output;
}


template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS> OpenLoopIMU<DIM_S,SCALAR,OPTIONS>::
  getMeasurementPDWRDispersionState(
    const SCALAR                                                                /* time */,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,  OPTIONS>>& /* truth_state */,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS>>& /* control_input */)
{
  Eigen::Matrix<SCALAR,6,DIM_S::TRUTH_DISP_DIM,OPTIONS> output;

  output.setZero();
  output.template block<3,3>(DIM_S::INER_MEAS::GYRO_START_IND, DIM_S::TRUTH_DISP::GYRO_BIAS_START_IND). setIdentity();
  output.template block<3,3>(DIM_S::INER_MEAS::ACCEL_START_IND,DIM_S::TRUTH_DISP::ACCEL_BIAS_START_IND).setIdentity();

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,DIM_S::CONTROL_DIM,OPTIONS> OpenLoopIMU<DIM_S,SCALAR,OPTIONS>::
  getMeasurementPDWRControl(
    const SCALAR                                                                /* time */,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,  OPTIONS>>& /* truth_state */,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS>>& /* control_input */)
{
  Eigen::Matrix<SCALAR,6,DIM_S::CONTROL_DIM,OPTIONS> output;

  output.setZero();
  output.template block<3,3>(DIM_S::INER_MEAS::GYRO_START_IND, DIM_S::CONTROL::GYRO_START_IND). setIdentity();
  output.template block<3,3>(DIM_S::INER_MEAS::ACCEL_START_IND,DIM_S::CONTROL::ACCEL_START_IND).setIdentity();

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,DIM_S::INER_MEAS_NOISE_DIM,OPTIONS> OpenLoopIMU<DIM_S,SCALAR,OPTIONS>::
  getMeasurementPDWRNoise(
    const SCALAR                                                                /* time */,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,  OPTIONS>>& /* truth_state */,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS>>& /* control_input */)
{
  Eigen::Matrix<SCALAR,6,DIM_S::INER_MEAS_NOISE_DIM,OPTIONS> output;

  output.setZero();
  output.template block<3,3>(DIM_S::INER_MEAS::GYRO_START_IND, DIM_S::INER_MEAS_NOISE::GYRO_START_IND). setIdentity();
  output.template block<3,3>(DIM_S::INER_MEAS::ACCEL_START_IND,DIM_S::INER_MEAS_NOISE::ACCEL_START_IND).setIdentity();

  return output;
}
} // namespace sensors
} // namespace kf

#endif
/* open_loop_imu.hpp */
