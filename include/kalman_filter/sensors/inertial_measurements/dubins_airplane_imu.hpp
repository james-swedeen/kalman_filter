/**
 * @File: dubins_airplane_imu.hpp
 * @Date: July 2023
 * @Author: James Swedeen
 *
 * @brief
 * A helper class that defines the IMU functionality for the Dubin's Airplane model.
 **/

#ifndef KALMAN_FILTER_SENSORS_INERTIAL_MEASUREMENTS_DUBINS_AIRPLANE_IMU_HPP
#define KALMAN_FILTER_SENSORS_INERTIAL_MEASUREMENTS_DUBINS_AIRPLANE_IMU_HPP

/* C++ Headers */
#include<cstdint>
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<kalman_filter/math/helpers.hpp>
#include<kalman_filter/math/quaternion.hpp>
#include<kalman_filter/sensors/inertial_measurements/inertial_measurement_base.hpp>

namespace kf
{
namespace sensors
{
template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
class DubinsAirplaneIMU;

template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using DubinsAirplaneIMUPtr = std::shared_ptr<DubinsAirplaneIMU<DIM_S,SCALAR,OPTIONS>>;

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
class DubinsAirplaneIMU
 : public InertialMeasurementBase<DIM_S,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  DubinsAirplaneIMU() = delete;
  /**
   * @Copy Constructor
   **/
  DubinsAirplaneIMU(const DubinsAirplaneIMU&) noexcept = default;
  /**
   * @Move Constructor
   **/
  DubinsAirplaneIMU(DubinsAirplaneIMU&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * Initializes the class for use.
   *
   * @parameters
   * gravity_accel: The magnitude of the acceleration from gravity
   **/
  DubinsAirplaneIMU(const SCALAR gravity_accel) noexcept;
  /**
   * @Deconstructor
   **/
  ~DubinsAirplaneIMU() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  DubinsAirplaneIMU& operator=(const DubinsAirplaneIMU&)  noexcept = default;
  DubinsAirplaneIMU& operator=(      DubinsAirplaneIMU&&) noexcept = default;
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
private:
  SCALAR gravity_accel;
};


template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
DubinsAirplaneIMU<DIM_S,SCALAR,OPTIONS>::DubinsAirplaneIMU(const SCALAR gravity_accel) noexcept
 : InertialMeasurementBase<DIM_S,SCALAR,OPTIONS>(),
   gravity_accel(gravity_accel)
{}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::INER_MEAS_DIM,OPTIONS> DubinsAirplaneIMU<DIM_S,SCALAR,OPTIONS>::
  getMeasurement(const SCALAR                                                                        /* time */,
                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,  OPTIONS>>&         truth_state,
                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS>>&         control_input,
                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::INER_MEAS_NOISE_DIM,OPTIONS>>& inertial_noise)
{
  Eigen::Matrix<SCALAR,1,DIM_S::INER_MEAS_DIM,OPTIONS> output;

  const Eigen::Matrix<SCALAR,3,3,OPTIONS> ned_to_body_rotation =
    math::quat::rollPitchYawToDirectionCosineMatrix(truth_state.template middleCols<3>(DIM_S::TRUTH::EULER_START_IND)).transpose();

  const SCALAR s_roll  = std::sin(truth_state[DIM_S::TRUTH::ROLL_IND]);
  const SCALAR c_roll  = std::cos(truth_state[DIM_S::TRUTH::ROLL_IND]);
  const SCALAR s_pitch = std::sin(truth_state[DIM_S::TRUTH::PITCH_IND]);
  const SCALAR c_pitch = std::cos(truth_state[DIM_S::TRUTH::PITCH_IND]);

  Eigen::Matrix<SCALAR,3,3,OPTIONS> angular_rotation_ned_to_body;
  angular_rotation_ned_to_body(0, 0) = 1;
  angular_rotation_ned_to_body(1, 0) = 0;
  angular_rotation_ned_to_body(2, 0) = 0;
  angular_rotation_ned_to_body(0, 1) = 0;
  angular_rotation_ned_to_body(1, 1) = c_roll;
  angular_rotation_ned_to_body(2, 1) = -s_roll;
  angular_rotation_ned_to_body(0, 2) = -s_pitch;
  angular_rotation_ned_to_body(1, 2) = c_pitch * s_roll;
  angular_rotation_ned_to_body(2, 2) = c_pitch * c_roll;

  // Angular Rates
  output.template middleCols<3>(DIM_S::INER_MEAS::GYRO_START_IND) =
    angular_rotation_ned_to_body *
    Eigen::Matrix<SCALAR,1,3,OPTIONS>({control_input[DIM_S::CONTROL::ROLL_RATE_IND], control_input[DIM_S::CONTROL::PITCH_RATE_IND], (this->gravity_accel / truth_state[DIM_S::TRUTH::AIR_SPEED_IND]) * std::tan(truth_state[DIM_S::TRUTH::ROLL_IND])}).transpose();

  // Accelerations
  output[DIM_S::INER_MEAS::X_ACCEL_IND] = control_input[DIM_S::CONTROL::AIR_SPEED_RATE_IND];
  output[DIM_S::INER_MEAS::Y_ACCEL_IND] = 0;
  output[DIM_S::INER_MEAS::Z_ACCEL_IND] = 0;
  output.template middleCols<3>(DIM_S::INER_MEAS::ACCEL_START_IND) +=
    math::crossProduct(output.template middleCols<3>(DIM_S::INER_MEAS::GYRO_START_IND),
                       Eigen::Matrix<SCALAR,1,3,OPTIONS>({truth_state[DIM_S::TRUTH::AIR_SPEED_IND], 0, 0})).transpose() +
    (ned_to_body_rotation * Eigen::Matrix<SCALAR,1,3,OPTIONS>({0, 0, -this->gravity_accel}).transpose());
  if constexpr(DIM_S::USE_IMU_BIASES)
  {
    // Add biases
    output.template middleCols<3>(DIM_S::INER_MEAS::GYRO_START_IND)  += truth_state.template middleCols<3>(DIM_S::TRUTH::GYRO_BIAS_START_IND);
    output.template middleCols<3>(DIM_S::INER_MEAS::ACCEL_START_IND) += truth_state.template middleCols<3>(DIM_S::TRUTH::ACCEL_BIAS_START_IND);
  }

  return output + inertial_noise;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS>
  DubinsAirplaneIMU<DIM_S,SCALAR,OPTIONS>::getMeasurementPDWRDispersionState(
    const SCALAR                                                                /* time */,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,  OPTIONS>>& truth_state,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS>>& control_input)
{
  Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS> output;
  output.setZero();

  const SCALAR s_roll  = std::sin(truth_state[DIM_S::TRUTH::ROLL_IND]);
  const SCALAR c_roll  = std::cos(truth_state[DIM_S::TRUTH::ROLL_IND]);
  const SCALAR s_pitch = std::sin(truth_state[DIM_S::TRUTH::PITCH_IND]);
  const SCALAR c_pitch = std::cos(truth_state[DIM_S::TRUTH::PITCH_IND]);

  const SCALAR t_roll    = std::tan(truth_state[DIM_S::TRUTH::ROLL_IND]);
  const SCALAR t_roll_pd = SCALAR(1) + std::pow(t_roll, SCALAR(2));
  const SCALAR yaw_rate  = (this->gravity_accel / truth_state[DIM_S::TRUTH::AIR_SPEED_IND]) * t_roll;

  const Eigen::Matrix<SCALAR,3,3,OPTIONS> truth_vel_cross_mat  = math::crossProductMatrix(Eigen::Matrix<SCALAR,1,3,OPTIONS>({truth_state[DIM_S::TRUTH::AIR_SPEED_IND], 0, 0}));
  const Eigen::Matrix<SCALAR,3,3,OPTIONS> ned_to_body_rotation = math::quat::rollPitchYawToDirectionCosineMatrix(truth_state.template middleCols<3>(DIM_S::TRUTH::EULER_START_IND)).transpose();

  Eigen::Matrix<SCALAR,3,3,OPTIONS> angular_rotation_ned_to_body;
  angular_rotation_ned_to_body(0, 0) = 1;
  angular_rotation_ned_to_body(1, 0) = 0;
  angular_rotation_ned_to_body(2, 0) = 0;
  angular_rotation_ned_to_body(0, 1) = 0;
  angular_rotation_ned_to_body(1, 1) = c_roll;
  angular_rotation_ned_to_body(2, 1) = -s_roll;
  angular_rotation_ned_to_body(0, 2) = -s_pitch;
  angular_rotation_ned_to_body(1, 2) = c_pitch * s_roll;
  angular_rotation_ned_to_body(2, 2) = c_pitch * c_roll;

  Eigen::Matrix<SCALAR,1,3,OPTIONS> angular_rate;
  angular_rate[0] = control_input[DIM_S::CONTROL::ROLL_RATE_IND];
  angular_rate[1] = control_input[DIM_S::CONTROL::PITCH_RATE_IND];
  angular_rate[2] = yaw_rate;
  angular_rate = angular_rotation_ned_to_body * angular_rate.transpose();

  Eigen::Matrix<SCALAR,3,3,OPTIONS> temp;
  temp.setZero();
  temp(2, 0) = (this->gravity_accel / truth_state[DIM_S::TRUTH::AIR_SPEED_IND]) * t_roll_pd;

  Eigen::Matrix<SCALAR,3,3,OPTIONS> ned_to_body_rotation_col_2_pd_euler; // Also angular_rotation_ned_to_body_col_2_pd_euler
  ned_to_body_rotation_col_2_pd_euler(0, 0) = 0;
  ned_to_body_rotation_col_2_pd_euler(1, 0) =  c_pitch * c_roll;
  ned_to_body_rotation_col_2_pd_euler(2, 0) = -c_pitch * s_roll;
  ned_to_body_rotation_col_2_pd_euler(0, 1) = -c_pitch;
  ned_to_body_rotation_col_2_pd_euler(1, 1) = -s_pitch * s_roll;
  ned_to_body_rotation_col_2_pd_euler(2, 1) = -s_pitch * c_roll;
  ned_to_body_rotation_col_2_pd_euler.template rightCols<1>().setZero();

  Eigen::Matrix<SCALAR,3,3,OPTIONS> angular_rotation_ned_to_body_col_1_pd_euler;
  angular_rotation_ned_to_body_col_1_pd_euler(0, 0) = 0;
  angular_rotation_ned_to_body_col_1_pd_euler(1, 0) = -s_roll;
  angular_rotation_ned_to_body_col_1_pd_euler(2, 0) = -c_roll;
  angular_rotation_ned_to_body_col_1_pd_euler(0, 1) = 0;
  angular_rotation_ned_to_body_col_1_pd_euler(1, 1) = 0;
  angular_rotation_ned_to_body_col_1_pd_euler(2, 1) = 0;
  angular_rotation_ned_to_body_col_1_pd_euler(0, 2) = 0;
  angular_rotation_ned_to_body_col_1_pd_euler(1, 2) = 0;
  angular_rotation_ned_to_body_col_1_pd_euler(2, 2) = 0;

  // Euler Angles
  output.template block<3,3>(DIM_S::INER_MEAS::GYRO_START_IND, DIM_S::TRUTH_DISP::EULER_START_IND).noalias() =
     (ned_to_body_rotation_col_2_pd_euler * yaw_rate) +
     (angular_rotation_ned_to_body_col_1_pd_euler * control_input[DIM_S::CONTROL::PITCH_RATE_IND]) +
     (angular_rotation_ned_to_body * temp);
  output.template block<3,1>(DIM_S::INER_MEAS::GYRO_START_IND, DIM_S::TRUTH_DISP::AIR_SPEED_IND).noalias() =
    -angular_rotation_ned_to_body * Eigen::Matrix<SCALAR,1,3,OPTIONS>({0, 0, (this->gravity_accel / std::pow(truth_state[DIM_S::TRUTH::AIR_SPEED_IND], SCALAR(2))) * t_roll}).transpose();
  // Accelerations
  output.template block<3,3>(DIM_S::INER_MEAS::ACCEL_START_IND, DIM_S::TRUTH_DISP::EULER_START_IND) =
    (
     -truth_vel_cross_mat *
     output.template block<3,3>(DIM_S::INER_MEAS::GYRO_START_IND, DIM_S::TRUTH_DISP::EULER_START_IND)
    ) -
    (ned_to_body_rotation_col_2_pd_euler * this->gravity_accel);

  output.template block<3,1>(DIM_S::INER_MEAS::ACCEL_START_IND, DIM_S::TRUTH_DISP::AIR_SPEED_IND) =
    (
     math::crossProductMatrix(angular_rate) * Eigen::Matrix<SCALAR,1,3,OPTIONS>({1, 0, 0}).transpose()
    ) -
    (
     truth_vel_cross_mat * output.template block<3,1>(DIM_S::INER_MEAS::GYRO_START_IND, DIM_S::TRUTH_DISP::AIR_SPEED_IND)
    );
  if constexpr(DIM_S::USE_IMU_BIASES)
  {
    // Biases
    output.template block<3,3>(DIM_S::INER_MEAS::GYRO_START_IND, DIM_S::TRUTH_DISP::GYRO_BIAS_START_IND). setIdentity();
    output.template block<3,3>(DIM_S::INER_MEAS::ACCEL_START_IND,DIM_S::TRUTH_DISP::ACCEL_BIAS_START_IND).setIdentity();
  }

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,DIM_S::CONTROL_DIM,OPTIONS> DubinsAirplaneIMU<DIM_S,SCALAR,OPTIONS>::
  getMeasurementPDWRControl(
    const SCALAR                                                                /* time */,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,  OPTIONS>>& truth_state,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS>>& /* control_input */)
{
  Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,DIM_S::CONTROL_DIM,OPTIONS> output;
  output.setZero();

  const SCALAR s_roll = std::sin(truth_state[DIM_S::TRUTH::ROLL_IND]);
  const SCALAR c_roll = std::cos(truth_state[DIM_S::TRUTH::ROLL_IND]);

  Eigen::Matrix<SCALAR,3,2,OPTIONS> angular_rotation_ned_to_body;
  angular_rotation_ned_to_body(0, 0) = 1;
  angular_rotation_ned_to_body(1, 0) = 0;
  angular_rotation_ned_to_body(2, 0) = 0;
  angular_rotation_ned_to_body(0, 1) = 0;
  angular_rotation_ned_to_body(1, 1) = c_roll;
  angular_rotation_ned_to_body(2, 1) = -s_roll;

  // Angular Rates
  output.template block<3,2>(DIM_S::INER_MEAS::GYRO_START_IND, DIM_S::CONTROL::ROLL_RATE_IND) = angular_rotation_ned_to_body;
  // Accelerations
  output.template middleRows<3>(DIM_S::INER_MEAS::ACCEL_START_IND) =
    -math::crossProductMatrix(Eigen::Matrix<SCALAR,1,3,OPTIONS>({truth_state[DIM_S::TRUTH::AIR_SPEED_IND], 0, 0})) *
    output.template middleRows<3>(DIM_S::INER_MEAS::GYRO_START_IND);
  output(DIM_S::INER_MEAS::X_ACCEL_IND, DIM_S::CONTROL::AIR_SPEED_RATE_IND) += 1;

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,DIM_S::INER_MEAS_NOISE_DIM,OPTIONS>
  DubinsAirplaneIMU<DIM_S,SCALAR,OPTIONS>::getMeasurementPDWRNoise(
    const SCALAR                                                                /* time */,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,  OPTIONS>>& /* truth_state */,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS>>& /* control_input */)
{
  return Eigen::Matrix<SCALAR,6,6,OPTIONS>::Identity();
}
} // namespace sensors
} // namespace kf

#endif
/* dubins_airplane_imu.hpp */
