/**
 * @File: null_inertial_measurement.hpp
 * @Date: June 2023
 * @Author: James Swedeen
 *
 * @brief
 * A helper class can be used in the place of the initial measurements assuming the system of interest does not use
 * model replacement.
 **/

#ifndef KALMAN_FILTER_SENSORS_INERTIAL_MEASUREMENTS_NULL_INERTIAL_MEASUREMENT_HPP
#define KALMAN_FILTER_SENSORS_INERTIAL_MEASUREMENTS_NULL_INERTIAL_MEASUREMENT_HPP

/* C++ Headers */
#include<cstdint>
#include<limits>
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<kalman_filter/sensors/inertial_measurements/inertial_measurement_base.hpp>

namespace kf
{
namespace sensors
{
template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
class NullInertialMeasurement;

template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using NullInertialMeasurementPtr = std::shared_ptr<NullInertialMeasurement<DIM_S,SCALAR,OPTIONS>>;

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
class NullInertialMeasurement
 : public InertialMeasurementBase<DIM_S,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  NullInertialMeasurement() = default;
  /**
   * @Copy Constructor
   **/
  NullInertialMeasurement(const NullInertialMeasurement&) = default;
  /**
   * @Move Constructor
   **/
  NullInertialMeasurement(NullInertialMeasurement&&) = default;
  /**
   * @Deconstructor
   **/
  ~NullInertialMeasurement() override = default;
  /**
   * @Assignment Operators
   **/
  NullInertialMeasurement& operator=(const NullInertialMeasurement&)  = default;
  NullInertialMeasurement& operator=(      NullInertialMeasurement&&) = default;
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
  inline Eigen::Matrix<SCALAR,1,DIM_S::INER_MEAS_DIM,OPTIONS>
    getMeasurement(const SCALAR                                                                        time,
                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,  OPTIONS>>&         truth_state,
                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS>>&         control_input,
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
inline Eigen::Matrix<SCALAR,1,DIM_S::INER_MEAS_DIM,OPTIONS> NullInertialMeasurement<DIM_S,SCALAR,OPTIONS>::
  getMeasurement(const SCALAR                                                                        /* time */,
                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,  OPTIONS>>&         /* truth_state */,
                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS>>&         /* control_input */,
                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::INER_MEAS_NOISE_DIM,OPTIONS>>& /* inertial_noise */)
{
  return Eigen::Matrix<SCALAR,1,DIM_S::INER_MEAS_DIM,OPTIONS>::Constant(std::numeric_limits<SCALAR>::quiet_NaN());
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS> NullInertialMeasurement<DIM_S,SCALAR,OPTIONS>::
  getMeasurementPDWRDispersionState(const SCALAR                                                                /* time */,
                                    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,  OPTIONS>>& /* truth_state */,
                                    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS>>& /* control_input */)
{
  return Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS>::Zero();
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,DIM_S::CONTROL_DIM,OPTIONS> NullInertialMeasurement<DIM_S,SCALAR,OPTIONS>::
  getMeasurementPDWRControl(const SCALAR                                                                /* time */,
                            const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,  OPTIONS>>& /* truth_state */,
                            const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS>>& /* control_input */)
{
  return Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,DIM_S::CONTROL_DIM,OPTIONS>::Zero();
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,DIM_S::INER_MEAS_NOISE_DIM,OPTIONS> NullInertialMeasurement<DIM_S,SCALAR,OPTIONS>::
  getMeasurementPDWRNoise(const SCALAR                                                                /* time */,
                          const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,  OPTIONS>>& /* truth_state */,
                          const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS>>& /* control_input */)
{
  return Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,DIM_S::INER_MEAS_NOISE_DIM,OPTIONS>::Zero();
}
} // namespace sensors
} // namespace kf

#endif
/* null_inertial_measurement.hpp */
