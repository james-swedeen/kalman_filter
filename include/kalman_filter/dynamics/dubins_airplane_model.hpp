/**
 * @File: dubins_airplane_model.hpp
 * @Date: July 2023
 * @Author: James Swedeen
 *
 * @brief
 * Defines dynamics that roughly follow the Dubin's Airplane model.
 **/

#ifndef KALMAN_FILTER_DYNMAICS_DUBINS_AIRPLANE_MODEL_HPP
#define KALMAN_FILTER_DYNMAICS_DUBINS_AIRPLANE_MODEL_HPP

/* C++ Headers */
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<kalman_filter/helpers/dimension_struct.hpp>
#include<kalman_filter/math/quaternion.hpp>
#include<kalman_filter/dynamics/dynamics_base.hpp>

namespace kf
{
namespace dynamics
{
/**
 * @DubinsAirplaneDim
 **/
template<bool STEADY_STATE_ERROR_COV_T, bool USE_IMU_BIASES_T>
struct DubinsAirplaneDimBase;

using DubinsAirplaneDim   = DubinsAirplaneDimBase<false,false>;
using DubinsAirplaneDimSS = DubinsAirplaneDimBase<true, false>;

template<bool STEADY_STATE_ERROR_COV_T, bool USE_IMU_BIASES_T>
struct DubinsAirplaneDimBase
 : public Dimensions<9, // Ref
                     (USE_IMU_BIASES_T) ? 13 : 7, // Truth
                     (USE_IMU_BIASES_T) ? 16 : 10, // Nav
                     5, // Num measurement
                     (USE_IMU_BIASES_T) ? 13 : 7, // Truth Dips
                     (USE_IMU_BIASES_T) ? 15 : 9, // Error
                     6, // IMU Dim
                     3, // Control
                     (USE_IMU_BIASES_T) ? 9 : 3, // Process Noise
                     6, // IMU Noise
                     STEADY_STATE_ERROR_COV_T>
{
public:
  inline static constexpr const bool USE_IMU_BIASES = USE_IMU_BIASES_T;
  struct REF
  {
  public:
    inline static constexpr const Eigen::Index NORTH_IND     = 0;
    inline static constexpr const Eigen::Index EAST_IND      = 1;
    inline static constexpr const Eigen::Index DOWN_IND      = 2;
    inline static constexpr const Eigen::Index ROLL_IND      = 3;
    inline static constexpr const Eigen::Index PITCH_IND     = 4;
    inline static constexpr const Eigen::Index YAW_IND       = 5;
    inline static constexpr const Eigen::Index NORTH_VEL_IND = 6;
    inline static constexpr const Eigen::Index EAST_VEL_IND  = 7;
    inline static constexpr const Eigen::Index DOWN_VEL_IND  = 8;

    inline static constexpr const Eigen::Index POS_START_IND   = NORTH_IND;
    inline static constexpr const Eigen::Index POS_END_IND     = DOWN_IND;
    inline static constexpr const Eigen::Index EULER_START_IND = ROLL_IND;
    inline static constexpr const Eigen::Index EULER_END_IND   = YAW_IND;
    inline static constexpr const Eigen::Index VEL_START_IND   = NORTH_VEL_IND;
    inline static constexpr const Eigen::Index VEL_END_IND     = DOWN_VEL_IND;
  };
  struct TRUTH
  {
  public:
    inline static constexpr const Eigen::Index NORTH_IND        = 0;
    inline static constexpr const Eigen::Index EAST_IND         = 1;
    inline static constexpr const Eigen::Index DOWN_IND         = 2;
    inline static constexpr const Eigen::Index ROLL_IND         = 3;
    inline static constexpr const Eigen::Index PITCH_IND        = 4;
    inline static constexpr const Eigen::Index YAW_IND          = 5;
    inline static constexpr const Eigen::Index AIR_SPEED_IND    = 6;
    inline static constexpr const Eigen::Index GYRO_BIAS_X_IND  = 7;
    inline static constexpr const Eigen::Index GYRO_BIAS_Y_IND  = 8;
    inline static constexpr const Eigen::Index GYRO_BIAS_Z_IND  = 9;
    inline static constexpr const Eigen::Index ACCEL_BIAS_X_IND = 10;
    inline static constexpr const Eigen::Index ACCEL_BIAS_Y_IND = 11;
    inline static constexpr const Eigen::Index ACCEL_BIAS_Z_IND = 12;

    inline static constexpr const Eigen::Index POS_START_IND        = NORTH_IND;
    inline static constexpr const Eigen::Index POS_END_IND          = DOWN_IND;
    inline static constexpr const Eigen::Index EULER_START_IND      = ROLL_IND;
    inline static constexpr const Eigen::Index EULER_END_IND        = YAW_IND;
    inline static constexpr const Eigen::Index GYRO_BIAS_START_IND  = GYRO_BIAS_X_IND;
    inline static constexpr const Eigen::Index GYRO_BIAS_END_IND    = GYRO_BIAS_Z_IND;
    inline static constexpr const Eigen::Index ACCEL_BIAS_START_IND = ACCEL_BIAS_X_IND;
    inline static constexpr const Eigen::Index ACCEL_BIAS_END_IND   = ACCEL_BIAS_Z_IND;
  };
  struct NAV
  {
  public:
    inline static constexpr const Eigen::Index NORTH_IND        = 0;
    inline static constexpr const Eigen::Index EAST_IND         = 1;
    inline static constexpr const Eigen::Index DOWN_IND         = 2;
    inline static constexpr const Eigen::Index QUAT_W_IND       = 3;
    inline static constexpr const Eigen::Index QUAT_X_IND       = 4;
    inline static constexpr const Eigen::Index QUAT_Y_IND       = 5;
    inline static constexpr const Eigen::Index QUAT_Z_IND       = 6;
    inline static constexpr const Eigen::Index NORTH_VEL_IND    = 7;
    inline static constexpr const Eigen::Index EAST_VEL_IND     = 8;
    inline static constexpr const Eigen::Index DOWN_VEL_IND     = 9;
    inline static constexpr const Eigen::Index GYRO_BIAS_X_IND  = 10;
    inline static constexpr const Eigen::Index GYRO_BIAS_Y_IND  = 11;
    inline static constexpr const Eigen::Index GYRO_BIAS_Z_IND  = 12;
    inline static constexpr const Eigen::Index ACCEL_BIAS_X_IND = 13;
    inline static constexpr const Eigen::Index ACCEL_BIAS_Y_IND = 14;
    inline static constexpr const Eigen::Index ACCEL_BIAS_Z_IND = 15;

    inline static constexpr const Eigen::Index POS_START_IND        = NORTH_IND;
    inline static constexpr const Eigen::Index POS_END_IND          = DOWN_IND;
    inline static constexpr const Eigen::Index QUAT_START_IND       = QUAT_W_IND;
    inline static constexpr const Eigen::Index QUAT_END_IND         = QUAT_Z_IND;
    inline static constexpr const Eigen::Index VEL_START_IND        = NORTH_VEL_IND;
    inline static constexpr const Eigen::Index VEL_END_IND          = DOWN_VEL_IND;
    inline static constexpr const Eigen::Index GYRO_BIAS_START_IND  = GYRO_BIAS_X_IND;
    inline static constexpr const Eigen::Index GYRO_BIAS_END_IND    = GYRO_BIAS_Z_IND;
    inline static constexpr const Eigen::Index ACCEL_BIAS_START_IND = ACCEL_BIAS_X_IND;
    inline static constexpr const Eigen::Index ACCEL_BIAS_END_IND   = ACCEL_BIAS_Z_IND;
  };
  struct NUM_MEAS
  {
  public:
    inline static constexpr const Eigen::Index GPS_IND           = 0;
    inline static constexpr const Eigen::Index COMPASS_IND       = 1;
    inline static constexpr const Eigen::Index ABS_PRESSURE_IND  = 2;
    inline static constexpr const Eigen::Index DIFF_PRESSURE_IND = 3;
    inline static constexpr const Eigen::Index FEATURE_IND       = 4;
  };
  struct TRUTH_DISP
  {
  public:
    inline static constexpr const Eigen::Index NORTH_IND        = 0;
    inline static constexpr const Eigen::Index EAST_IND         = 1;
    inline static constexpr const Eigen::Index DOWN_IND         = 2;
    inline static constexpr const Eigen::Index ROLL_IND         = 3;
    inline static constexpr const Eigen::Index PITCH_IND        = 4;
    inline static constexpr const Eigen::Index YAW_IND          = 5;
    inline static constexpr const Eigen::Index AIR_SPEED_IND    = 6;
    inline static constexpr const Eigen::Index GYRO_BIAS_X_IND  = 7;
    inline static constexpr const Eigen::Index GYRO_BIAS_Y_IND  = 8;
    inline static constexpr const Eigen::Index GYRO_BIAS_Z_IND  = 9;
    inline static constexpr const Eigen::Index ACCEL_BIAS_X_IND = 10;
    inline static constexpr const Eigen::Index ACCEL_BIAS_Y_IND = 11;
    inline static constexpr const Eigen::Index ACCEL_BIAS_Z_IND = 12;

    inline static constexpr const Eigen::Index POS_START_IND        = NORTH_IND;
    inline static constexpr const Eigen::Index POS_END_IND          = DOWN_IND;
    inline static constexpr const Eigen::Index EULER_START_IND      = ROLL_IND;
    inline static constexpr const Eigen::Index EULER_END_IND        = YAW_IND;
    inline static constexpr const Eigen::Index GYRO_BIAS_START_IND  = GYRO_BIAS_X_IND;
    inline static constexpr const Eigen::Index GYRO_BIAS_END_IND    = GYRO_BIAS_Z_IND;
    inline static constexpr const Eigen::Index ACCEL_BIAS_START_IND = ACCEL_BIAS_X_IND;
    inline static constexpr const Eigen::Index ACCEL_BIAS_END_IND   = ACCEL_BIAS_Z_IND;
  };
  struct ERROR
  {
  public:
    inline static constexpr const Eigen::Index NORTH_IND        = 0;
    inline static constexpr const Eigen::Index EAST_IND         = 1;
    inline static constexpr const Eigen::Index DOWN_IND         = 2;
    inline static constexpr const Eigen::Index ROLL_IND         = 3;
    inline static constexpr const Eigen::Index PITCH_IND        = 4;
    inline static constexpr const Eigen::Index YAW_IND          = 5;
    inline static constexpr const Eigen::Index NORTH_VEL_IND    = 6;
    inline static constexpr const Eigen::Index EAST_VEL_IND     = 7;
    inline static constexpr const Eigen::Index DOWN_VEL_IND     = 8;
    inline static constexpr const Eigen::Index GYRO_BIAS_X_IND  = 9;
    inline static constexpr const Eigen::Index GYRO_BIAS_Y_IND  = 10;
    inline static constexpr const Eigen::Index GYRO_BIAS_Z_IND  = 11;
    inline static constexpr const Eigen::Index ACCEL_BIAS_X_IND = 12;
    inline static constexpr const Eigen::Index ACCEL_BIAS_Y_IND = 13;
    inline static constexpr const Eigen::Index ACCEL_BIAS_Z_IND = 14;

    inline static constexpr const Eigen::Index POS_START_IND        = NORTH_IND;
    inline static constexpr const Eigen::Index POS_END_IND          = DOWN_IND;
    inline static constexpr const Eigen::Index EULER_START_IND      = ROLL_IND;
    inline static constexpr const Eigen::Index EULER_END_IND        = YAW_IND;
    inline static constexpr const Eigen::Index VEL_START_IND        = NORTH_VEL_IND;
    inline static constexpr const Eigen::Index VEL_END_IND          = DOWN_VEL_IND;
    inline static constexpr const Eigen::Index GYRO_BIAS_START_IND  = GYRO_BIAS_X_IND;
    inline static constexpr const Eigen::Index GYRO_BIAS_END_IND    = GYRO_BIAS_Z_IND;
    inline static constexpr const Eigen::Index ACCEL_BIAS_START_IND = ACCEL_BIAS_X_IND;
    inline static constexpr const Eigen::Index ACCEL_BIAS_END_IND   = ACCEL_BIAS_Z_IND;
  };
  struct INER_MEAS
  {
  public:
    inline static constexpr const Eigen::Index ROLL_RATE_IND  = 0;
    inline static constexpr const Eigen::Index PITCH_RATE_IND = 1;
    inline static constexpr const Eigen::Index YAW_RATE_IND   = 2;
    inline static constexpr const Eigen::Index X_ACCEL_IND    = 3;
    inline static constexpr const Eigen::Index Y_ACCEL_IND    = 4;
    inline static constexpr const Eigen::Index Z_ACCEL_IND    = 5;

    inline static constexpr const Eigen::Index GYRO_START_IND  = ROLL_RATE_IND;
    inline static constexpr const Eigen::Index GYRO_END_IND    = YAW_RATE_IND;
    inline static constexpr const Eigen::Index ACCEL_START_IND = X_ACCEL_IND;
    inline static constexpr const Eigen::Index ACCEL_END_IND   = Z_ACCEL_IND;
  };
  struct CONTROL
  {
  public:
    inline static constexpr const Eigen::Index ROLL_RATE_IND      = 0;
    inline static constexpr const Eigen::Index PITCH_RATE_IND     = 1;
    inline static constexpr const Eigen::Index AIR_SPEED_RATE_IND = 2;
  };
  struct TRUTH_NOISE
  {
  public:
    inline static constexpr const Eigen::Index NORTH_IND           = 0;
    inline static constexpr const Eigen::Index EAST_IND            = 1;
    inline static constexpr const Eigen::Index DOWN_IND            = 2;
    inline static constexpr const Eigen::Index ROLL_RATE_BIAS_IND  = 3;
    inline static constexpr const Eigen::Index PITCH_RATE_BIAS_IND = 4;
    inline static constexpr const Eigen::Index YAW_RATE_BIAS_IND   = 5;
    inline static constexpr const Eigen::Index X_ACCEL_BIAS_IND    = 6;
    inline static constexpr const Eigen::Index Y_ACCEL_BIAS_IND    = 7;
    inline static constexpr const Eigen::Index Z_ACCEL_BIAS_IND    = 8;

    inline static constexpr const Eigen::Index POS_START_IND        = NORTH_IND;
    inline static constexpr const Eigen::Index POS_END_IND          = DOWN_IND;
    inline static constexpr const Eigen::Index GYRO_BIAS_START_IND  = ROLL_RATE_BIAS_IND;
    inline static constexpr const Eigen::Index GYRO_BIAS_END_IND    = YAW_RATE_BIAS_IND;
    inline static constexpr const Eigen::Index ACCEL_BIAS_START_IND = X_ACCEL_BIAS_IND;
    inline static constexpr const Eigen::Index ACCEL_BIAS_END_IND   = Z_ACCEL_BIAS_IND;
  };
  struct INER_MEAS_NOISE
  {
  public:
    inline static constexpr const Eigen::Index ROLL_RATE_IND  = 0;
    inline static constexpr const Eigen::Index PITCH_RATE_IND = 1;
    inline static constexpr const Eigen::Index YAW_RATE_IND   = 2;
    inline static constexpr const Eigen::Index X_ACCEL_IND    = 3;
    inline static constexpr const Eigen::Index Y_ACCEL_IND    = 4;
    inline static constexpr const Eigen::Index Z_ACCEL_IND    = 5;

    inline static constexpr const Eigen::Index GYRO_START_IND  = ROLL_RATE_IND;
    inline static constexpr const Eigen::Index GYRO_END_IND    = YAW_RATE_IND;
    inline static constexpr const Eigen::Index ACCEL_START_IND = X_ACCEL_IND;
    inline static constexpr const Eigen::Index ACCEL_END_IND   = Z_ACCEL_IND;
  };
};

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
class DubinsAirplane;

template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using DubinsAirplanePtr = std::shared_ptr<DubinsAirplane<DIM_S,SCALAR,OPTIONS>>;

/**
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options.
 **/
template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class DubinsAirplane
 : public DynamicsBase<DIM_S,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  DubinsAirplane() = delete;
  /**
   * @Copy Constructor
   **/
  DubinsAirplane(const DubinsAirplane&) noexcept = default;
  /**
   * @Move Constructor
   **/
  DubinsAirplane(DubinsAirplane&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * Initializes the class for use.
   *
   * @parameters
   * gravity_accel: The magnitude of the acceleration from gravity
   * accel_time_constant: The first order Gauss Markov time constant for the accelerometer bias
   * gyro_time_constant: The first order Gauss Markov time constant for the gyroscope bias
   **/
  DubinsAirplane(const SCALAR gravity_accel,
                 const SCALAR accel_time_constant,
                 const SCALAR gyro_time_constant) noexcept;
  /**
   * @Deconstructor
   **/
  ~DubinsAirplane() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  DubinsAirplane& operator=(const DubinsAirplane&)  noexcept = default;
  DubinsAirplane& operator=(      DubinsAirplane&&) noexcept = default;
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
  inline Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>
    getTruthStateDynamics(const SCALAR                                                                    time,
                          const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,      OPTIONS>>& truth_state,
                          const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,    OPTIONS>>& control_input,
                          const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_NOISE_DIM,OPTIONS>>& process_noise) override;
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
  inline Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>
    getNavStateDynamics(const SCALAR                                                                  time,
                        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,      OPTIONS>>& nav_state,
                        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,      OPTIONS>>& ref_state,
                        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::INER_MEAS_DIM,OPTIONS>>& inertial_reading) override;
  /**
   * @getTruthStateDynamicsPDWRDispersionState
   *
   * @brief
   * Finds the derivative of the truth state dynamics with respect to the truth state and then evaluated
   * along the nominal reference trajectory.
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
  inline Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS>
    getTruthStateDynamicsPDWRDispersionState(const SCALAR                                                                time,
                                             const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,  OPTIONS>>& truth_state,
                                             const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS>>& control_input) override;
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
  inline Eigen::Matrix<SCALAR,DIM_S::CONTROL_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS>
    getTruthStateDynamicsPDWRControl(
      const SCALAR                                                                time,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,  OPTIONS>>& truth_state,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS>>& control_input) override;
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
  inline Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_NOISE_DIM,OPTIONS>
    getTruthStateDynamicsPDWRNoise(
      const SCALAR                                                                time,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,  OPTIONS>>& truth_state,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS>>& control_input) override;
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
  inline Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS>
    getNavStateDynamicsPDWRErrorState(
      const SCALAR                                                                  time,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,      OPTIONS>>& nav_state,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,      OPTIONS>>& ref_state,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::INER_MEAS_DIM,OPTIONS>>& inertial_reading) override;
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
  inline Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,DIM_S::ERROR_DIM,OPTIONS>
    getNavStateDynamicsPDWRInertialMeasurement(
      const SCALAR                                                                  time,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,      OPTIONS>>& nav_state,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,      OPTIONS>>& ref_state,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::INER_MEAS_DIM,OPTIONS>>& inertial_reading) override;
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
  inline SCALAR findTimeStep(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,OPTIONS>>& prev_ref_state,
                             const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,OPTIONS>>& cur_ref_state) override;
  /**
   * @calculateTruthVelocity
   *
   * @brief
   * Finds the velocity of the UAV from the truth state.
   *
   * @templates
   * DERIVED: The matrix type of the truth state vector
   *
   * @parameters
   * truth_state: The current truth state vector
   *
   * @return
   * The North, East, Down velocity of the UAV (in that order) and assuming no process noise.
   **/
  template<typename DERIVED>
  inline static Eigen::Matrix<SCALAR,1,3,OPTIONS>
    calculateTruthVelocity(const Eigen::MatrixBase<DERIVED>& truth_state) noexcept;
private:
  SCALAR gravity_accel;
  SCALAR accel_time_constant;
  SCALAR gyro_time_constant;
};


template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
DubinsAirplane<DIM_S,SCALAR,OPTIONS>::DubinsAirplane(const SCALAR gravity_accel,
                                                     const SCALAR accel_time_constant,
                                                     const SCALAR gyro_time_constant) noexcept
 : DynamicsBase<DIM_S,SCALAR,OPTIONS>(),
   gravity_accel(gravity_accel),
   accel_time_constant((DIM_S::USE_IMU_BIASES) ? accel_time_constant : std::numeric_limits<SCALAR>::quiet_NaN()),
   gyro_time_constant( (DIM_S::USE_IMU_BIASES) ? gyro_time_constant  : std::numeric_limits<SCALAR>::quiet_NaN())
{}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS> DubinsAirplane<DIM_S,SCALAR,OPTIONS>::
  getTruthStateDynamics(const SCALAR                                                                    /* time */,
                        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,      OPTIONS>>& truth_state,
                        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,    OPTIONS>>& control_input,
                        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_NOISE_DIM,OPTIONS>>& process_noise)
{
  Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS> output;

  output.template leftCols<3>() = DubinsAirplane<DIM_S,SCALAR,OPTIONS>::calculateTruthVelocity(truth_state) + process_noise.template middleCols<3>(DIM_S::TRUTH_NOISE::POS_START_IND);
  output[DIM_S::TRUTH::ROLL_IND] = control_input[DIM_S::CONTROL::ROLL_RATE_IND];
  output[DIM_S::TRUTH::PITCH_IND] = control_input[DIM_S::CONTROL::PITCH_RATE_IND];
  output[DIM_S::TRUTH::YAW_IND] = (this->gravity_accel / truth_state[DIM_S::TRUTH::AIR_SPEED_IND]) * std::tan(truth_state[DIM_S::TRUTH::ROLL_IND]);
  output[DIM_S::TRUTH::AIR_SPEED_IND] = control_input[DIM_S::CONTROL::AIR_SPEED_RATE_IND];
  if constexpr(DIM_S::USE_IMU_BIASES)
  {
    output.template middleCols<3>(DIM_S::TRUTH::GYRO_BIAS_START_IND).noalias() =
      ((SCALAR(-1)/this->gyro_time_constant) * truth_state.template middleCols<3>(DIM_S::TRUTH::GYRO_BIAS_START_IND).array()).matrix() +
        process_noise.template middleCols<3>(DIM_S::TRUTH_NOISE::GYRO_BIAS_START_IND);
    output.template middleCols<3>(DIM_S::TRUTH::ACCEL_BIAS_START_IND).noalias() =
      ((SCALAR(-1)/this->accel_time_constant) * truth_state.template middleCols<3>(DIM_S::TRUTH::ACCEL_BIAS_START_IND).array()).matrix() +
        process_noise.template middleCols<3>(DIM_S::TRUTH_NOISE::ACCEL_BIAS_START_IND);
  }

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS> DubinsAirplane<DIM_S,SCALAR,OPTIONS>::
  getNavStateDynamics(const SCALAR                                                                  /* time */,
                      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,      OPTIONS>>& nav_state,
                      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,      OPTIONS>>& ref_state,
                      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::INER_MEAS_DIM,OPTIONS>>& inertial_reading)
{
  Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS> output;

  // Make helper information
  Eigen::Matrix<SCALAR,1,3,OPTIONS> gravity_vec;
  Eigen::Matrix<SCALAR,3,3,OPTIONS> body_to_ned_rotation;
  Eigen::Matrix<SCALAR,1,4,OPTIONS> angular_rotation_quat;

  gravity_vec[0] = 0;
  gravity_vec[1] = 0;
  gravity_vec[2] = this->gravity_accel;
  body_to_ned_rotation = math::quat::quaternionToDirectionCosineMatrix(nav_state.template middleCols<4>(DIM_S::NAV::QUAT_START_IND));
  angular_rotation_quat[0] = 0;
  angular_rotation_quat.template rightCols<3>() = inertial_reading.template middleCols<3>(DIM_S::INER_MEAS::GYRO_START_IND);
  if constexpr(DIM_S::USE_IMU_BIASES)
  {
    angular_rotation_quat.template rightCols<3>() -= nav_state.template middleCols<3>(DIM_S::NAV::GYRO_BIAS_START_IND);
  }

  output.template middleCols<3>(DIM_S::NAV::POS_START_IND) = nav_state.template middleCols<3>(DIM_S::NAV::VEL_START_IND);
  output.template middleCols<4>(DIM_S::NAV::QUAT_START_IND).noalias() =
    (SCALAR(1)/SCALAR(2)) * math::quat::product(nav_state.template middleCols<4>(DIM_S::NAV::QUAT_START_IND),
                                                angular_rotation_quat);
  if constexpr(DIM_S::USE_IMU_BIASES)
  {
    output.template middleCols<3>(DIM_S::NAV::VEL_START_IND).noalias() =
      ((inertial_reading.template middleCols<3>(DIM_S::INER_MEAS::ACCEL_START_IND) -
        nav_state.       template middleCols<3>(DIM_S::NAV::ACCEL_BIAS_START_IND)) * body_to_ned_rotation.transpose()) + gravity_vec;
    output.template middleCols<3>(DIM_S::NAV::GYRO_BIAS_START_IND) =
      (SCALAR(-1)/this->gyro_time_constant) * nav_state.template middleCols<3>(DIM_S::NAV::GYRO_BIAS_START_IND).array();
    output.template middleCols<3>(DIM_S::NAV::ACCEL_BIAS_START_IND) =
      (SCALAR(-1)/this->accel_time_constant) * nav_state.template middleCols<3>(DIM_S::NAV::ACCEL_BIAS_START_IND).array();
  }
  else // Don't use IMU biases
  {
    output.template middleCols<3>(DIM_S::NAV::VEL_START_IND).noalias() =
      (body_to_ned_rotation * inertial_reading.template middleCols<3>(DIM_S::INER_MEAS::ACCEL_START_IND).transpose()).transpose() + gravity_vec;
  }

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS> DubinsAirplane<DIM_S,SCALAR,OPTIONS>::
  getTruthStateDynamicsPDWRDispersionState(const SCALAR                                                                /* time */,
                                           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,  OPTIONS>>& truth_state,
                                           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS>>& /* control_input */)
{
  Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS> output;
  output.setZero();

  // Helper variables
  const SCALAR c_pitch = std::cos(truth_state[DIM_S::TRUTH::PITCH_IND]);
  const SCALAR s_pitch = std::sin(truth_state[DIM_S::TRUTH::PITCH_IND]);
  const SCALAR c_yaw   = std::cos(truth_state[DIM_S::TRUTH::YAW_IND]);
  const SCALAR s_yaw   = std::sin(truth_state[DIM_S::TRUTH::YAW_IND]);

  // North position terms
  output(DIM_S::TRUTH_DISP::NORTH_IND, DIM_S::TRUTH_DISP::PITCH_IND)     = -truth_state[DIM_S::TRUTH::AIR_SPEED_IND] * c_yaw * s_pitch;
  output(DIM_S::TRUTH_DISP::NORTH_IND, DIM_S::TRUTH_DISP::YAW_IND)       = -truth_state[DIM_S::TRUTH::AIR_SPEED_IND] * s_yaw * c_pitch;
  output(DIM_S::TRUTH_DISP::NORTH_IND, DIM_S::TRUTH_DISP::AIR_SPEED_IND) = c_yaw * c_pitch;
  // East position terms
  output(DIM_S::TRUTH_DISP::EAST_IND, DIM_S::TRUTH_DISP::PITCH_IND)     = -truth_state[DIM_S::TRUTH::AIR_SPEED_IND] * s_yaw * s_pitch;
  output(DIM_S::TRUTH_DISP::EAST_IND, DIM_S::TRUTH_DISP::YAW_IND)       =  truth_state[DIM_S::TRUTH::AIR_SPEED_IND] * c_yaw * c_pitch;
  output(DIM_S::TRUTH_DISP::EAST_IND, DIM_S::TRUTH_DISP::AIR_SPEED_IND) = s_yaw * c_pitch;
  // Down position terms
  output(DIM_S::TRUTH_DISP::DOWN_IND, DIM_S::TRUTH_DISP::PITCH_IND)     = -truth_state[DIM_S::TRUTH::AIR_SPEED_IND] * c_pitch;
  output(DIM_S::TRUTH_DISP::DOWN_IND, DIM_S::TRUTH_DISP::AIR_SPEED_IND) = -s_pitch;
  // Yaw terms
  output(DIM_S::TRUTH_DISP::YAW_IND, DIM_S::TRUTH_DISP::ROLL_IND)      = (this->gravity_accel / truth_state[DIM_S::TRUTH::AIR_SPEED_IND]) *
                                                                         (SCALAR(1) / std::pow(std::cos(truth_state[DIM_S::TRUTH::ROLL_IND]), SCALAR(2)));
  output(DIM_S::TRUTH_DISP::YAW_IND, DIM_S::TRUTH_DISP::AIR_SPEED_IND) = (-this->gravity_accel / std::pow(truth_state[DIM_S::TRUTH::AIR_SPEED_IND], SCALAR(2))) *
                                                                         std::tan(truth_state[DIM_S::TRUTH::ROLL_IND]);
  if constexpr(DIM_S::USE_IMU_BIASES)
  {
    // Gyroscope bias terms
    output.template block<3,3>(DIM_S::TRUTH_DISP::GYRO_BIAS_START_IND, DIM_S::TRUTH_DISP::GYRO_BIAS_START_IND) =
      (SCALAR(-1)/this->gyro_time_constant) * Eigen::Matrix<SCALAR,3,3,OPTIONS>::Identity().array();
    // Accelerometer bias terms
    output.template block<3,3>(DIM_S::TRUTH_DISP::ACCEL_BIAS_START_IND, DIM_S::TRUTH_DISP::ACCEL_BIAS_START_IND) =
      (SCALAR(-1)/this->accel_time_constant) * Eigen::Matrix<SCALAR,3,3,OPTIONS>::Identity().array();
  }

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,DIM_S::CONTROL_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS> DubinsAirplane<DIM_S,SCALAR,OPTIONS>::
  getTruthStateDynamicsPDWRControl(
    const SCALAR                                                                /* time */,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,  OPTIONS>>& truth_state,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS>>& /* control_input */)
{
  Eigen::Matrix<SCALAR,DIM_S::CONTROL_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS> output;
  output.setZero();

  output(DIM_S::CONTROL::ROLL_RATE_IND,      DIM_S::TRUTH_DISP::ROLL_IND)      = 1;
  output(DIM_S::CONTROL::PITCH_RATE_IND,     DIM_S::TRUTH_DISP::PITCH_IND)     = 1;
  output(DIM_S::CONTROL::AIR_SPEED_RATE_IND, DIM_S::TRUTH_DISP::AIR_SPEED_IND) = 1;

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_NOISE_DIM,OPTIONS> DubinsAirplane<DIM_S,SCALAR,OPTIONS>::
  getTruthStateDynamicsPDWRNoise(
    const SCALAR                                                                /* time */,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,  OPTIONS>>& /* truth_state */,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS>>& /* control_input */)
{
  Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_NOISE_DIM,OPTIONS> output;
  output.setZero();

  output.template block<3,3>(DIM_S::TRUTH_DISP::POS_START_IND, DIM_S::TRUTH_NOISE::POS_START_IND).setIdentity();
  if constexpr(DIM_S::USE_IMU_BIASES)
  {
    output.template block<3,3>(DIM_S::TRUTH_DISP::GYRO_BIAS_START_IND,  DIM_S::TRUTH_NOISE::GYRO_BIAS_START_IND). setIdentity();
    output.template block<3,3>(DIM_S::TRUTH_DISP::ACCEL_BIAS_START_IND, DIM_S::TRUTH_NOISE::ACCEL_BIAS_START_IND).setIdentity();
  }

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS> DubinsAirplane<DIM_S,SCALAR,OPTIONS>::
  getNavStateDynamicsPDWRErrorState(
    const SCALAR                                                                  /* time */,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,      OPTIONS>>& nav_state,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,      OPTIONS>>& ref_state,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::INER_MEAS_DIM,OPTIONS>>& inertial_reading)
{
  Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS> output;
  output.setZero();

  // Helper variables
  const Eigen::Matrix<SCALAR,3,3,OPTIONS> body_to_ned_rotation =
    math::quat::quaternionToDirectionCosineMatrix(nav_state.template middleCols<4>(DIM_S::NAV::QUAT_START_IND));

  // Position terms
  output.template block<3,3>(DIM_S::ERROR::POS_START_IND, DIM_S::ERROR::VEL_START_IND).setIdentity();
  // Euler angles terms
  if constexpr(DIM_S::USE_IMU_BIASES)
  {
    output.template block<3,3>(DIM_S::ERROR::EULER_START_IND, DIM_S::ERROR::EULER_START_IND) =
      -math::crossProductMatrix(inertial_reading.template middleCols<3>(DIM_S::INER_MEAS::GYRO_START_IND) -
                                nav_state.       template middleCols<3>(DIM_S::NAV::GYRO_BIAS_START_IND));
    output.template block<3,3>(DIM_S::ERROR::EULER_START_IND, DIM_S::ERROR::GYRO_BIAS_START_IND) = -Eigen::Matrix<SCALAR,3,3,OPTIONS>::Identity(); // = -body_to_ned_rotation;
  }
  else // Don't use IMU biases
  {
    output.template block<3,3>(DIM_S::ERROR::EULER_START_IND, DIM_S::ERROR::EULER_START_IND) =
      -math::crossProductMatrix(inertial_reading.template middleCols<3>(DIM_S::INER_MEAS::GYRO_START_IND));
  }
  // Velocity terms
  if constexpr(DIM_S::USE_IMU_BIASES)
  {
    output.template block<3,3>(DIM_S::ERROR::VEL_START_IND, DIM_S::ERROR::EULER_START_IND).noalias() =
      -body_to_ned_rotation * math::crossProductMatrix(inertial_reading.template middleCols<3>(DIM_S::INER_MEAS::ACCEL_START_IND) -
                                                       nav_state.       template middleCols<3>(DIM_S::NAV::ACCEL_BIAS_START_IND));
    output.template block<3,3>(DIM_S::ERROR::VEL_START_IND, DIM_S::ERROR::ACCEL_BIAS_START_IND) = -body_to_ned_rotation;
  }
  else // Don't use IMU biases
  {
    output.template block<3,3>(DIM_S::ERROR::VEL_START_IND, DIM_S::ERROR::EULER_START_IND).noalias() =
      -body_to_ned_rotation * math::crossProductMatrix(inertial_reading.template middleCols<3>(DIM_S::INER_MEAS::ACCEL_START_IND));
  }
  if constexpr(DIM_S::USE_IMU_BIASES)
  {
    // Gyroscope bias terms
    output.template block<3,3>(DIM_S::ERROR::GYRO_BIAS_START_IND, DIM_S::ERROR::GYRO_BIAS_START_IND) =
      (SCALAR(-1)/this->gyro_time_constant) * Eigen::Matrix<SCALAR,3,3,OPTIONS>::Identity().array();
    // Accelerometer bias terms
    output.template block<3,3>(DIM_S::ERROR::ACCEL_BIAS_START_IND, DIM_S::ERROR::ACCEL_BIAS_START_IND) =
      (SCALAR(-1)/this->accel_time_constant) * Eigen::Matrix<SCALAR,3,3,OPTIONS>::Identity().array();
  }

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,DIM_S::ERROR_DIM,OPTIONS> DubinsAirplane<DIM_S,SCALAR,OPTIONS>::
  getNavStateDynamicsPDWRInertialMeasurement(
    const SCALAR                                                                  /* time */,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,      OPTIONS>>& nav_state,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,      OPTIONS>>& /* ref_state */,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::INER_MEAS_DIM,OPTIONS>>& /* inertial_reading */)
{
  Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,DIM_S::ERROR_DIM,OPTIONS> output;
  output.setZero();

  // Helper variables
  const Eigen::Matrix<SCALAR,3,3,OPTIONS> body_to_ned_rotation =
    math::quat::quaternionToDirectionCosineMatrix(nav_state.template middleCols<4>(DIM_S::NAV::QUAT_START_IND));

  output.template block<3,3>(DIM_S::INER_MEAS::GYRO_START_IND,  DIM_S::ERROR::EULER_START_IND).setIdentity();
  output.template block<3,3>(DIM_S::INER_MEAS::ACCEL_START_IND, DIM_S::ERROR::VEL_START_IND) = body_to_ned_rotation.transpose();

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR DubinsAirplane<DIM_S,SCALAR,OPTIONS>::
  findTimeStep(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,OPTIONS>>& prev_ref_state,
               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,OPTIONS>>& cur_ref_state)
{
  const SCALAR dist_traveled = (prev_ref_state.template middleCols<3>(DIM_S::REF::POS_START_IND) -
                                cur_ref_state. template middleCols<3>(DIM_S::REF::POS_START_IND)).norm();
  const SCALAR nom_vel       = prev_ref_state.template middleCols<3>(DIM_S::REF::VEL_START_IND).norm();

  return dist_traveled / nom_vel;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename DERIVED>
inline Eigen::Matrix<SCALAR,1,3,OPTIONS> DubinsAirplane<DIM_S,SCALAR,OPTIONS>::
  calculateTruthVelocity(const Eigen::MatrixBase<DERIVED>& truth_state) noexcept
{
  Eigen::Matrix<SCALAR,1,3,OPTIONS> output;

  const SCALAR c_pitch = std::cos(truth_state[DIM_S::TRUTH::PITCH_IND]);
  const SCALAR s_pitch = std::sin(truth_state[DIM_S::TRUTH::PITCH_IND]);
  const SCALAR c_yaw   = std::cos(truth_state[DIM_S::TRUTH::YAW_IND]);
  const SCALAR s_yaw   = std::sin(truth_state[DIM_S::TRUTH::YAW_IND]);

  output[0] =  truth_state[DIM_S::TRUTH::AIR_SPEED_IND] * c_yaw * c_pitch;
  output[1] =  truth_state[DIM_S::TRUTH::AIR_SPEED_IND] * s_yaw * c_pitch;
  output[2] = -truth_state[DIM_S::TRUTH::AIR_SPEED_IND] * s_pitch;

  return output;
}
} // namespace dynamics
} // namespace kf

#endif
/* dubins_airplane_model.hpp */
