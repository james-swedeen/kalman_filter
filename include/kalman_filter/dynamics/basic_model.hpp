/**
 * @File: basic_model.hpp
 * @Date: May 2022
 * @Author: James Swedeen
 *
 * @brief
 * A simple model for 3 dimensional systems.
 **/

#ifndef KALMAN_FILTER_DYNAMICS_BASIC_MODEL_HPP
#define KALMAN_FILTER_DYNAMICS_BASIC_MODEL_HPP

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
 * @BasicModelDim
 **/
template<bool STEADY_STATE_ERROR_COV_T>
struct BasicModelDimBase;

using BasicModelDim = BasicModelDimBase<false>;
using BasicModelDimSS = BasicModelDimBase<true>;

template<bool STEADY_STATE_ERROR_COV_T>
struct BasicModelDimBase
 : public Dimensions<15,25,25,7,24,24,6,6,15,6,STEADY_STATE_ERROR_COV_T>
{
public:
  struct REF
  {
  public:
    inline static constexpr const Eigen::Index NORTH_IND      = 0;
    inline static constexpr const Eigen::Index EAST_IND       = 1;
    inline static constexpr const Eigen::Index DOWN_IND       = 2;
    inline static constexpr const Eigen::Index ROLL_IND       = 3;
    inline static constexpr const Eigen::Index PITCH_IND      = 4;
    inline static constexpr const Eigen::Index YAW_IND        = 5;
    inline static constexpr const Eigen::Index NORTH_VEL_IND  = 6;
    inline static constexpr const Eigen::Index EAST_VEL_IND   = 7;
    inline static constexpr const Eigen::Index DOWN_VEL_IND   = 8;
    inline static constexpr const Eigen::Index ROLL_RATE_IND  = 9;
    inline static constexpr const Eigen::Index PITCH_RATE_IND = 10;
    inline static constexpr const Eigen::Index YAW_RATE_IND   = 11;
    inline static constexpr const Eigen::Index X_ACCEL_IND    = 12;
    inline static constexpr const Eigen::Index Y_ACCEL_IND    = 13;
    inline static constexpr const Eigen::Index Z_ACCEL_IND    = 14;

    inline static constexpr const Eigen::Index POS_START_IND        = NORTH_IND;
    inline static constexpr const Eigen::Index POS_END_IND          = DOWN_IND;
    inline static constexpr const Eigen::Index EULER_START_IND      = ROLL_IND;
    inline static constexpr const Eigen::Index EULER_END_IND        = YAW_IND;
    inline static constexpr const Eigen::Index VEL_START_IND        = NORTH_VEL_IND;
    inline static constexpr const Eigen::Index VEL_END_IND          = DOWN_VEL_IND;
    inline static constexpr const Eigen::Index EULER_RATE_START_IND = ROLL_RATE_IND;
    inline static constexpr const Eigen::Index EULER_RATE_END_IND   = YAW_RATE_IND;
    inline static constexpr const Eigen::Index ACCEL_START_IND      = X_ACCEL_IND;
    inline static constexpr const Eigen::Index ACCEL_END_IND        = Z_ACCEL_IND;
  };
  struct TRUTH
  {
  public:
    inline static constexpr const Eigen::Index NORTH_IND                      = 0;
    inline static constexpr const Eigen::Index EAST_IND                       = 1;
    inline static constexpr const Eigen::Index DOWN_IND                       = 2;
    inline static constexpr const Eigen::Index QUAT_W_IND                     = 3;
    inline static constexpr const Eigen::Index QUAT_X_IND                     = 4;
    inline static constexpr const Eigen::Index QUAT_Y_IND                     = 5;
    inline static constexpr const Eigen::Index QUAT_Z_IND                     = 6;
    inline static constexpr const Eigen::Index NORTH_VEL_IND                  = 7;
    inline static constexpr const Eigen::Index EAST_VEL_IND                   = 8;
    inline static constexpr const Eigen::Index DOWN_VEL_IND                   = 9;
    inline static constexpr const Eigen::Index HEADING_BIAS_IND               = 10;
    inline static constexpr const Eigen::Index ABS_PRESSURE_BIAS_IND          = 11;
    inline static constexpr const Eigen::Index FEATURE_RANGE_BIAS_IND         = 12;
    inline static constexpr const Eigen::Index FEATURE_BEARING_ROLL_BIAS_IND  = 13;
    inline static constexpr const Eigen::Index FEATURE_BEARING_PITCH_BIAS_IND = 14;
    inline static constexpr const Eigen::Index FEATURE_BEARING_YAW_BIAS_IND   = 15;
    inline static constexpr const Eigen::Index GPS_NORTH_BIAS_IND             = 16;
    inline static constexpr const Eigen::Index GPS_EAST_BIAS_IND              = 17;
    inline static constexpr const Eigen::Index GPS_DOWN_BIAS_IND              = 18;
    inline static constexpr const Eigen::Index GYRO_BIAS_X_IND                = 19;
    inline static constexpr const Eigen::Index GYRO_BIAS_Y_IND                = 20;
    inline static constexpr const Eigen::Index GYRO_BIAS_Z_IND                = 21;
    inline static constexpr const Eigen::Index ACCEL_BIAS_X_IND               = 22;
    inline static constexpr const Eigen::Index ACCEL_BIAS_Y_IND               = 23;
    inline static constexpr const Eigen::Index ACCEL_BIAS_Z_IND               = 24;

    inline static constexpr const Eigen::Index POS_START_IND                  = NORTH_IND;
    inline static constexpr const Eigen::Index POS_END_IND                    = DOWN_IND;
    inline static constexpr const Eigen::Index QUAT_START_IND                 = QUAT_W_IND;
    inline static constexpr const Eigen::Index QUAT_END_IND                   = QUAT_Z_IND;
    inline static constexpr const Eigen::Index VEL_START_IND                  = NORTH_VEL_IND;
    inline static constexpr const Eigen::Index VEL_END_IND                    = DOWN_VEL_IND;
    inline static constexpr const Eigen::Index FEATURE_BEARING_BIAS_START_IND = FEATURE_BEARING_ROLL_BIAS_IND;
    inline static constexpr const Eigen::Index FEATURE_BEARING_BIAS_END_IND   = FEATURE_BEARING_YAW_BIAS_IND;
    inline static constexpr const Eigen::Index GPS_POS_BIAS_START_IND         = GPS_NORTH_BIAS_IND;
    inline static constexpr const Eigen::Index GPS_POS_BIAS_END_IND           = GPS_DOWN_BIAS_IND;
    inline static constexpr const Eigen::Index GYRO_BIAS_START_IND            = GYRO_BIAS_X_IND;
    inline static constexpr const Eigen::Index GYRO_BIAS_END_IND              = GYRO_BIAS_Z_IND;
    inline static constexpr const Eigen::Index ACCEL_BIAS_START_IND           = ACCEL_BIAS_X_IND;
    inline static constexpr const Eigen::Index ACCEL_BIAS_END_IND             = ACCEL_BIAS_Z_IND;
  };
  struct NAV
  {
  public:
    inline static constexpr const Eigen::Index NORTH_IND                      = 0;
    inline static constexpr const Eigen::Index EAST_IND                       = 1;
    inline static constexpr const Eigen::Index DOWN_IND                       = 2;
    inline static constexpr const Eigen::Index QUAT_W_IND                     = 3;
    inline static constexpr const Eigen::Index QUAT_X_IND                     = 4;
    inline static constexpr const Eigen::Index QUAT_Y_IND                     = 5;
    inline static constexpr const Eigen::Index QUAT_Z_IND                     = 6;
    inline static constexpr const Eigen::Index NORTH_VEL_IND                  = 7;
    inline static constexpr const Eigen::Index EAST_VEL_IND                   = 8;
    inline static constexpr const Eigen::Index DOWN_VEL_IND                   = 9;
    inline static constexpr const Eigen::Index HEADING_BIAS_IND               = 10;
    inline static constexpr const Eigen::Index ABS_PRESSURE_BIAS_IND          = 11;
    inline static constexpr const Eigen::Index FEATURE_RANGE_BIAS_IND         = 12;
    inline static constexpr const Eigen::Index FEATURE_BEARING_ROLL_BIAS_IND  = 13;
    inline static constexpr const Eigen::Index FEATURE_BEARING_PITCH_BIAS_IND = 14;
    inline static constexpr const Eigen::Index FEATURE_BEARING_YAW_BIAS_IND   = 15;
    inline static constexpr const Eigen::Index GPS_NORTH_BIAS_IND             = 16;
    inline static constexpr const Eigen::Index GPS_EAST_BIAS_IND              = 17;
    inline static constexpr const Eigen::Index GPS_DOWN_BIAS_IND              = 18;
    inline static constexpr const Eigen::Index GYRO_BIAS_X_IND                = 19;
    inline static constexpr const Eigen::Index GYRO_BIAS_Y_IND                = 20;
    inline static constexpr const Eigen::Index GYRO_BIAS_Z_IND                = 21;
    inline static constexpr const Eigen::Index ACCEL_BIAS_X_IND               = 22;
    inline static constexpr const Eigen::Index ACCEL_BIAS_Y_IND               = 23;
    inline static constexpr const Eigen::Index ACCEL_BIAS_Z_IND               = 24;

    inline static constexpr const Eigen::Index POS_START_IND                  = NORTH_IND;
    inline static constexpr const Eigen::Index POS_END_IND                    = DOWN_IND;
    inline static constexpr const Eigen::Index QUAT_START_IND                 = QUAT_W_IND;
    inline static constexpr const Eigen::Index QUAT_END_IND                   = QUAT_Z_IND;
    inline static constexpr const Eigen::Index VEL_START_IND                  = NORTH_VEL_IND;
    inline static constexpr const Eigen::Index VEL_END_IND                    = DOWN_VEL_IND;
    inline static constexpr const Eigen::Index FEATURE_BEARING_BIAS_START_IND = FEATURE_BEARING_ROLL_BIAS_IND;
    inline static constexpr const Eigen::Index FEATURE_BEARING_BIAS_END_IND   = FEATURE_BEARING_YAW_BIAS_IND;
    inline static constexpr const Eigen::Index GPS_POS_BIAS_START_IND         = GPS_NORTH_BIAS_IND;
    inline static constexpr const Eigen::Index GPS_POS_BIAS_END_IND           = GPS_DOWN_BIAS_IND;
    inline static constexpr const Eigen::Index GYRO_BIAS_START_IND            = GYRO_BIAS_X_IND;
    inline static constexpr const Eigen::Index GYRO_BIAS_END_IND              = GYRO_BIAS_Z_IND;
    inline static constexpr const Eigen::Index ACCEL_BIAS_START_IND           = ACCEL_BIAS_X_IND;
    inline static constexpr const Eigen::Index ACCEL_BIAS_END_IND             = ACCEL_BIAS_Z_IND;
  };
  struct NUM_MEAS
  {
  public:
    inline static constexpr const Eigen::Index GPS_IND             = 0;
    inline static constexpr const Eigen::Index HEADING_IND         = 1;
    inline static constexpr const Eigen::Index ALTITUDE_IND        = 2;
    inline static constexpr const Eigen::Index ABS_PRESSURE_IND    = 3;
    inline static constexpr const Eigen::Index GROUND_VELOCITY_IND = 4;
    inline static constexpr const Eigen::Index FEATURE_RANGE_IND   = 5;
    inline static constexpr const Eigen::Index FEATURE_BEARING_IND = 6;
  };
  struct ERROR
  {
  public:
    inline static constexpr const Eigen::Index NORTH_IND                      = 0;
    inline static constexpr const Eigen::Index EAST_IND                       = 1;
    inline static constexpr const Eigen::Index DOWN_IND                       = 2;
    inline static constexpr const Eigen::Index ROLL_IND                       = 3;
    inline static constexpr const Eigen::Index PITCH_IND                      = 4;
    inline static constexpr const Eigen::Index YAW_IND                        = 5;
    inline static constexpr const Eigen::Index NORTH_VEL_IND                  = 6;
    inline static constexpr const Eigen::Index EAST_VEL_IND                   = 7;
    inline static constexpr const Eigen::Index DOWN_VEL_IND                   = 8;
    inline static constexpr const Eigen::Index HEADING_BIAS_IND               = 9;
    inline static constexpr const Eigen::Index ABS_PRESSURE_BIAS_IND          = 10;
    inline static constexpr const Eigen::Index FEATURE_RANGE_BIAS_IND         = 11;
    inline static constexpr const Eigen::Index FEATURE_BEARING_ROLL_BIAS_IND  = 12;
    inline static constexpr const Eigen::Index FEATURE_BEARING_PITCH_BIAS_IND = 13;
    inline static constexpr const Eigen::Index FEATURE_BEARING_YAW_BIAS_IND   = 14;
    inline static constexpr const Eigen::Index GPS_NORTH_BIAS_IND             = 15;
    inline static constexpr const Eigen::Index GPS_EAST_BIAS_IND              = 16;
    inline static constexpr const Eigen::Index GPS_DOWN_BIAS_IND              = 17;
    inline static constexpr const Eigen::Index GYRO_BIAS_X_IND                = 18;
    inline static constexpr const Eigen::Index GYRO_BIAS_Y_IND                = 19;
    inline static constexpr const Eigen::Index GYRO_BIAS_Z_IND                = 20;
    inline static constexpr const Eigen::Index ACCEL_BIAS_X_IND               = 21;
    inline static constexpr const Eigen::Index ACCEL_BIAS_Y_IND               = 22;
    inline static constexpr const Eigen::Index ACCEL_BIAS_Z_IND               = 23;

    inline static constexpr const Eigen::Index POS_START_IND                  = NORTH_IND;
    inline static constexpr const Eigen::Index POS_END_IND                    = DOWN_IND;
    inline static constexpr const Eigen::Index EULER_START_IND                = ROLL_IND;
    inline static constexpr const Eigen::Index EULER_END_IND                  = YAW_IND;
    inline static constexpr const Eigen::Index VEL_START_IND                  = NORTH_VEL_IND;
    inline static constexpr const Eigen::Index VEL_END_IND                    = DOWN_VEL_IND;
    inline static constexpr const Eigen::Index FEATURE_BEARING_BIAS_START_IND = FEATURE_BEARING_ROLL_BIAS_IND;
    inline static constexpr const Eigen::Index FEATURE_BEARING_BIAS_END_IND   = FEATURE_BEARING_YAW_BIAS_IND;
    inline static constexpr const Eigen::Index GPS_POS_BIAS_START_IND         = GPS_NORTH_BIAS_IND;
    inline static constexpr const Eigen::Index GPS_POS_BIAS_END_IND           = GPS_DOWN_BIAS_IND;
    inline static constexpr const Eigen::Index GYRO_BIAS_START_IND            = GYRO_BIAS_X_IND;
    inline static constexpr const Eigen::Index GYRO_BIAS_END_IND              = GYRO_BIAS_Z_IND;
    inline static constexpr const Eigen::Index ACCEL_BIAS_START_IND           = ACCEL_BIAS_X_IND;
    inline static constexpr const Eigen::Index ACCEL_BIAS_END_IND             = ACCEL_BIAS_Z_IND;
  };
  struct TRUTH_DISP
  {
  public:
    inline static constexpr const Eigen::Index NORTH_IND                      = 0;
    inline static constexpr const Eigen::Index EAST_IND                       = 1;
    inline static constexpr const Eigen::Index DOWN_IND                       = 2;
    inline static constexpr const Eigen::Index ROLL_IND                       = 3;
    inline static constexpr const Eigen::Index PITCH_IND                      = 4;
    inline static constexpr const Eigen::Index YAW_IND                        = 5;
    inline static constexpr const Eigen::Index NORTH_VEL_IND                  = 6;
    inline static constexpr const Eigen::Index EAST_VEL_IND                   = 7;
    inline static constexpr const Eigen::Index DOWN_VEL_IND                   = 8;
    inline static constexpr const Eigen::Index HEADING_BIAS_IND               = 9;
    inline static constexpr const Eigen::Index ABS_PRESSURE_BIAS_IND          = 10;
    inline static constexpr const Eigen::Index FEATURE_RANGE_BIAS_IND         = 11;
    inline static constexpr const Eigen::Index FEATURE_BEARING_ROLL_BIAS_IND  = 12;
    inline static constexpr const Eigen::Index FEATURE_BEARING_PITCH_BIAS_IND = 13;
    inline static constexpr const Eigen::Index FEATURE_BEARING_YAW_BIAS_IND   = 14;
    inline static constexpr const Eigen::Index GPS_NORTH_BIAS_IND             = 15;
    inline static constexpr const Eigen::Index GPS_EAST_BIAS_IND              = 16;
    inline static constexpr const Eigen::Index GPS_DOWN_BIAS_IND              = 17;
    inline static constexpr const Eigen::Index GYRO_BIAS_X_IND                = 18;
    inline static constexpr const Eigen::Index GYRO_BIAS_Y_IND                = 19;
    inline static constexpr const Eigen::Index GYRO_BIAS_Z_IND                = 20;
    inline static constexpr const Eigen::Index ACCEL_BIAS_X_IND               = 21;
    inline static constexpr const Eigen::Index ACCEL_BIAS_Y_IND               = 22;
    inline static constexpr const Eigen::Index ACCEL_BIAS_Z_IND               = 23;

    inline static constexpr const Eigen::Index POS_START_IND                  = NORTH_IND;
    inline static constexpr const Eigen::Index POS_END_IND                    = DOWN_IND;
    inline static constexpr const Eigen::Index EULER_START_IND                = ROLL_IND;
    inline static constexpr const Eigen::Index EULER_END_IND                  = YAW_IND;
    inline static constexpr const Eigen::Index VEL_START_IND                  = NORTH_VEL_IND;
    inline static constexpr const Eigen::Index VEL_END_IND                    = DOWN_VEL_IND;
    inline static constexpr const Eigen::Index FEATURE_BEARING_BIAS_START_IND = FEATURE_BEARING_ROLL_BIAS_IND;
    inline static constexpr const Eigen::Index FEATURE_BEARING_BIAS_END_IND   = FEATURE_BEARING_YAW_BIAS_IND;
    inline static constexpr const Eigen::Index GPS_POS_BIAS_START_IND         = GPS_NORTH_BIAS_IND;
    inline static constexpr const Eigen::Index GPS_POS_BIAS_END_IND           = GPS_DOWN_BIAS_IND;
    inline static constexpr const Eigen::Index GYRO_BIAS_START_IND            = GYRO_BIAS_X_IND;
    inline static constexpr const Eigen::Index GYRO_BIAS_END_IND              = GYRO_BIAS_Z_IND;
    inline static constexpr const Eigen::Index ACCEL_BIAS_START_IND           = ACCEL_BIAS_X_IND;
    inline static constexpr const Eigen::Index ACCEL_BIAS_END_IND             = ACCEL_BIAS_Z_IND;
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
  struct TRUTH_NOISE
  {
  public:
    inline static constexpr const Eigen::Index HEADING_BIAS_IND               = 0;
    inline static constexpr const Eigen::Index ABS_PRESSURE_BIAS_IND          = 1;
    inline static constexpr const Eigen::Index FEATURE_RANGE_BIAS_IND         = 2;
    inline static constexpr const Eigen::Index FEATURE_BEARING_ROLL_BIAS_IND  = 3;
    inline static constexpr const Eigen::Index FEATURE_BEARING_PITCH_BIAS_IND = 4;
    inline static constexpr const Eigen::Index FEATURE_BEARING_YAW_BIAS_IND   = 5;
    inline static constexpr const Eigen::Index GPS_NORTH_BIAS_IND             = 6;
    inline static constexpr const Eigen::Index GPS_EAST_BIAS_IND              = 7;
    inline static constexpr const Eigen::Index GPS_DOWN_BIAS_IND              = 8;
    inline static constexpr const Eigen::Index ROLL_RATE_BIAS_IND             = 9;
    inline static constexpr const Eigen::Index PITCH_RATE_BIAS_IND            = 10;
    inline static constexpr const Eigen::Index YAW_RATE_BIAS_IND              = 11;
    inline static constexpr const Eigen::Index X_ACCEL_BIAS_IND               = 12;
    inline static constexpr const Eigen::Index Y_ACCEL_BIAS_IND               = 13;
    inline static constexpr const Eigen::Index Z_ACCEL_BIAS_IND               = 14;

    inline static constexpr const Eigen::Index FEATURE_BEARING_BIAS_START_IND = FEATURE_BEARING_ROLL_BIAS_IND;
    inline static constexpr const Eigen::Index FEATURE_BEARING_BIAS_END_IND   = FEATURE_BEARING_YAW_BIAS_IND;
    inline static constexpr const Eigen::Index GPS_POS_BIAS_START_IND         = GPS_NORTH_BIAS_IND;
    inline static constexpr const Eigen::Index GPS_POS_BIAS_END_IND           = GPS_DOWN_BIAS_IND;
    inline static constexpr const Eigen::Index GYRO_BIAS_START_IND            = ROLL_RATE_BIAS_IND;
    inline static constexpr const Eigen::Index GYRO_BIAS_END_IND              = YAW_RATE_BIAS_IND;
    inline static constexpr const Eigen::Index ACCEL_BIAS_START_IND           = X_ACCEL_BIAS_IND;
    inline static constexpr const Eigen::Index ACCEL_BIAS_END_IND             = Z_ACCEL_BIAS_IND;
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
class BasicModel;

template<typename DIM_S = BasicModelDim, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using BasicModelPtr = std::shared_ptr<BasicModel<DIM_S,SCALAR,OPTIONS>>;

/**
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options.
 **/
template<typename DIM_S = BasicModelDim, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class BasicModel
 : public DynamicsBase<DIM_S,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  BasicModel() = delete;
  /**
   * @Copy Constructor
   **/
  BasicModel(const BasicModel&) noexcept = default;
  /**
   * @Move Constructor
   **/
  BasicModel(BasicModel&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * Initializes the class for use.
   *
   * @templates
   * DERIVED: The matrix type that the imu noise covariance
   *
   * @parameters
   * nominal_velocity: The nominal velocity of the vehicle
   * gravity_accel: The magnitude of the acceleration from gravity
   * heading_time_constant: The first order Gauss Markov time constant for the heading bias
   * abs_pressure_time_constant: The first order Gauss Markov time constant for the abs pressure bias
   * feature_range_time_constant: The first order Gauss Markov time constant for the feature range bias
   * feature_bearing_time_constant: The first order Gauss Markov time constant for the feature bearing bias
   * gps_position_time_constant: The first order Gauss Markov time constant for the GPS position reading's bias
   * accel_time_constant: The first order Gauss Markov time constant for the accelerometer bias
   * gyro_time_constant: The first order Gauss Markov time constant for the gyroscope bias
   **/
  BasicModel(const SCALAR nominal_velocity,
             const SCALAR gravity_accel,
             const SCALAR heading_time_constant,
             const SCALAR abs_pressure_time_constant,
             const SCALAR feature_range_time_constant,
             const SCALAR feature_bearing_time_constant,
             const SCALAR gps_position_time_constant,
             const SCALAR accel_time_constant,
             const SCALAR gyro_time_constant);
  /**
   * @Deconstructor
   **/
  ~BasicModel() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  BasicModel& operator=(const BasicModel&)  noexcept = default;
  BasicModel& operator=(      BasicModel&&) noexcept = default;
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
private:
  SCALAR nominal_velocity;
  SCALAR gravity_accel;
  SCALAR heading_time_constant;
  SCALAR abs_pressure_time_constant;
  SCALAR feature_range_time_constant;
  SCALAR feature_bearing_time_constant;
  SCALAR gps_position_time_constant;
  SCALAR accel_time_constant;
  SCALAR gyro_time_constant;
};

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
BasicModel<DIM_S,SCALAR,OPTIONS>::BasicModel(const SCALAR nominal_velocity,
                                             const SCALAR gravity_accel,
                                             const SCALAR heading_time_constant,
                                             const SCALAR abs_pressure_time_constant,
                                             const SCALAR feature_range_time_constant,
                                             const SCALAR feature_bearing_time_constant,
                                             const SCALAR gps_position_time_constant,
                                             const SCALAR accel_time_constant,
                                             const SCALAR gyro_time_constant)
 : DynamicsBase<DIM_S,SCALAR,OPTIONS>(),
   nominal_velocity(nominal_velocity),
   gravity_accel(gravity_accel),
   heading_time_constant(heading_time_constant),
   abs_pressure_time_constant(abs_pressure_time_constant),
   feature_range_time_constant(feature_range_time_constant),
   feature_bearing_time_constant(feature_bearing_time_constant),
   gps_position_time_constant(gps_position_time_constant),
   accel_time_constant(accel_time_constant),
   gyro_time_constant(gyro_time_constant)
{}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS> BasicModel<DIM_S,SCALAR,OPTIONS>::
  getTruthStateDynamics(const SCALAR                                                                    /* time */,
                        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,      OPTIONS>>& truth_state,
                        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,    OPTIONS>>& control_input,
                        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_NOISE_DIM,OPTIONS>>& process_noise)
{
  Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS> output;

  // Make helper information
  Eigen::Matrix<SCALAR,1,3,OPTIONS> gravity_vec;
  Eigen::Matrix<SCALAR,3,3,OPTIONS> body_to_ned_rotation;
  Eigen::Matrix<SCALAR,1,4,OPTIONS> angular_rotation_quat;

  gravity_vec[0] = 0;
  gravity_vec[1] = 0;
  gravity_vec[2] = this->gravity_accel;
  body_to_ned_rotation = math::quat::quaternionToDirectionCosineMatrix(truth_state.template middleCols<4>(DIM_S::TRUTH::QUAT_START_IND));
  angular_rotation_quat[0] = 0;
  angular_rotation_quat.template rightCols<3>() = control_input.template middleCols<3>(DIM_S::CONTROL::GYRO_START_IND);

  output.template middleCols<3>(DIM_S::TRUTH::POS_START_IND) =
    truth_state.template middleCols<3>(DIM_S::TRUTH::VEL_START_IND);
  output.template middleCols<4>(DIM_S::TRUTH::QUAT_START_IND) =
    (SCALAR(1)/SCALAR(2)) * math::quat::product(truth_state.template middleCols<4>(DIM_S::TRUTH::QUAT_START_IND),
                                                angular_rotation_quat).array();
  output.template middleCols<3>(DIM_S::TRUTH::VEL_START_IND).noalias() =
    (control_input.template middleCols<3>(DIM_S::CONTROL::ACCEL_START_IND) * body_to_ned_rotation.transpose()) + gravity_vec;
  output[DIM_S::TRUTH::HEADING_BIAS_IND] =
    ((SCALAR(-1)/this->heading_time_constant) * truth_state[DIM_S::TRUTH::HEADING_BIAS_IND]) +
      process_noise[DIM_S::TRUTH_NOISE::HEADING_BIAS_IND];
  output[DIM_S::TRUTH::ABS_PRESSURE_BIAS_IND] =
    ((SCALAR(-1)/this->abs_pressure_time_constant) * truth_state[DIM_S::TRUTH::ABS_PRESSURE_BIAS_IND]) +
      process_noise[DIM_S::TRUTH_NOISE::ABS_PRESSURE_BIAS_IND];
  output[DIM_S::TRUTH::FEATURE_RANGE_BIAS_IND] =
    ((SCALAR(-1)/this->feature_range_time_constant) * truth_state[DIM_S::TRUTH::FEATURE_RANGE_BIAS_IND]) +
      process_noise[DIM_S::TRUTH_NOISE::FEATURE_RANGE_BIAS_IND];
  output.template middleCols<3>(DIM_S::TRUTH::FEATURE_BEARING_BIAS_START_IND) =
    ((SCALAR(-1)/this->feature_bearing_time_constant) * truth_state.template middleCols<3>(DIM_S::TRUTH::FEATURE_BEARING_BIAS_START_IND).array()).matrix() +
      process_noise.template middleCols<3>(DIM_S::TRUTH_NOISE::FEATURE_BEARING_BIAS_START_IND);
  output.template middleCols<3>(DIM_S::TRUTH::GPS_POS_BIAS_START_IND) =
    ((SCALAR(-1)/this->gps_position_time_constant) * truth_state.template middleCols<3>(DIM_S::TRUTH::GPS_POS_BIAS_START_IND).array()).matrix() +
      process_noise.template middleCols<3>(DIM_S::TRUTH_NOISE::GPS_POS_BIAS_START_IND);
  output.template middleCols<3>(DIM_S::TRUTH::GYRO_BIAS_START_IND) =
    ((SCALAR(-1)/this->gyro_time_constant) * truth_state.template middleCols<3>(DIM_S::TRUTH::GYRO_BIAS_START_IND).array()).matrix() +
      process_noise.template middleCols<3>(DIM_S::TRUTH_NOISE::GYRO_BIAS_START_IND);
  output.template middleCols<3>(DIM_S::TRUTH::ACCEL_BIAS_START_IND) =
    ((SCALAR(-1)/this->accel_time_constant) * truth_state.template middleCols<3>(DIM_S::TRUTH::ACCEL_BIAS_START_IND).array()).matrix() +
      process_noise.template middleCols<3>(DIM_S::TRUTH_NOISE::ACCEL_BIAS_START_IND);

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS> BasicModel<DIM_S,SCALAR,OPTIONS>::
  getNavStateDynamics(const SCALAR                                                                  /* time */,
                      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,      OPTIONS>>& nav_state,
                      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,      OPTIONS>>& /* ref_state */,
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
  angular_rotation_quat.template rightCols<3>() = inertial_reading.template middleCols<3>(DIM_S::INER_MEAS::GYRO_START_IND) -
                                                  nav_state.       template middleCols<3>(DIM_S::NAV::GYRO_BIAS_START_IND);

  output.template middleCols<3>(DIM_S::NAV::POS_START_IND) =
    nav_state.template middleCols<3>(DIM_S::NAV::VEL_START_IND);
  output.template middleCols<4>(DIM_S::NAV::QUAT_START_IND) =
    (SCALAR(1)/SCALAR(2)) * math::quat::product(nav_state.template middleCols<4>(DIM_S::NAV::QUAT_START_IND),
                                                angular_rotation_quat).array();
  output.template middleCols<3>(DIM_S::NAV::VEL_START_IND).noalias() =
    ((inertial_reading.template middleCols<3>(DIM_S::INER_MEAS::ACCEL_START_IND) -
      nav_state.       template middleCols<3>(DIM_S::NAV::ACCEL_BIAS_START_IND)) * body_to_ned_rotation.transpose()) + gravity_vec;
  output[DIM_S::NAV::HEADING_BIAS_IND] = (SCALAR(-1)/this->heading_time_constant) * nav_state[DIM_S::NAV::HEADING_BIAS_IND];
  output[DIM_S::NAV::ABS_PRESSURE_BIAS_IND] = (SCALAR(-1)/this->abs_pressure_time_constant) * nav_state[DIM_S::NAV::ABS_PRESSURE_BIAS_IND];
  output[DIM_S::NAV::FEATURE_RANGE_BIAS_IND] = (SCALAR(-1)/this->feature_range_time_constant) * nav_state[DIM_S::NAV::FEATURE_RANGE_BIAS_IND];
  output.template middleCols<3>(DIM_S::NAV::FEATURE_BEARING_BIAS_START_IND) =
    (SCALAR(-1)/this->feature_bearing_time_constant) * nav_state.template middleCols<3>(DIM_S::NAV::FEATURE_BEARING_BIAS_START_IND).array();
  output.template middleCols<3>(DIM_S::NAV::GPS_POS_BIAS_START_IND) =
    (SCALAR(-1)/this->gps_position_time_constant) * nav_state.template middleCols<3>(DIM_S::NAV::GPS_POS_BIAS_START_IND).array();
  output.template middleCols<3>(DIM_S::NAV::GYRO_BIAS_START_IND) =
    (SCALAR(-1)/this->gyro_time_constant) * nav_state.template middleCols<3>(DIM_S::NAV::GYRO_BIAS_START_IND).array();
  output.template middleCols<3>(DIM_S::NAV::ACCEL_BIAS_START_IND) =
    (SCALAR(-1)/this->accel_time_constant) * nav_state.template middleCols<3>(DIM_S::NAV::ACCEL_BIAS_START_IND).array();

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS> BasicModel<DIM_S,SCALAR,OPTIONS>::
  getTruthStateDynamicsPDWRDispersionState(
    const SCALAR                                                                /* time */,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,  OPTIONS>>& truth_state,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS>>& control_input)
{
  Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS> output;

  // Helper variables
  Eigen::Matrix<SCALAR,3,3,OPTIONS> body_to_ned_rotation =
    math::quat::quaternionToDirectionCosineMatrix(truth_state.template middleCols<4>(DIM_S::NAV::QUAT_START_IND));

  output.setZero();
  // Position terms
  output.template block<3,3>(DIM_S::ERROR::POS_START_IND, DIM_S::ERROR::VEL_START_IND).setIdentity();
  // Euler angles terms
  // Velocity terms
  output.template block<3,3>(DIM_S::ERROR::VEL_START_IND, DIM_S::ERROR::EULER_START_IND).noalias() =
    -math::crossProductMatrix(body_to_ned_rotation * control_input.template middleCols<3>(DIM_S::INER_MEAS::ACCEL_START_IND).transpose());
  // Other bias terms
  output(DIM_S::ERROR::HEADING_BIAS_IND,      DIM_S::ERROR::HEADING_BIAS_IND)       = SCALAR(-1)/this->heading_time_constant;
  output(DIM_S::ERROR::ABS_PRESSURE_BIAS_IND, DIM_S::ERROR::ABS_PRESSURE_BIAS_IND)  = SCALAR(-1)/this->abs_pressure_time_constant;
  output(DIM_S::ERROR::FEATURE_RANGE_BIAS_IND,DIM_S::ERROR::FEATURE_RANGE_BIAS_IND) = SCALAR(-1)/this->feature_range_time_constant;
  output.template block<3,3>(DIM_S::ERROR::FEATURE_BEARING_BIAS_START_IND, DIM_S::ERROR::FEATURE_BEARING_BIAS_START_IND) =
    (SCALAR(-1)/this->feature_bearing_time_constant) * Eigen::Matrix<SCALAR,3,3,OPTIONS>::Identity().array();
  output.template block<3,3>(DIM_S::ERROR::GPS_POS_BIAS_START_IND, DIM_S::ERROR::GPS_POS_BIAS_START_IND) =
    (SCALAR(-1)/this->gps_position_time_constant) * Eigen::Matrix<SCALAR,3,3,OPTIONS>::Identity().array();
  // Gyroscope bias terms
  output.template block<3,3>(DIM_S::ERROR::GYRO_BIAS_START_IND, DIM_S::ERROR::GYRO_BIAS_START_IND) =
    (SCALAR(-1)/this->gyro_time_constant) * Eigen::Matrix<SCALAR,3,3,OPTIONS>::Identity().array();
  // Accelerometer bias terms
  output.template block<3,3>(DIM_S::ERROR::ACCEL_BIAS_START_IND, DIM_S::ERROR::ACCEL_BIAS_START_IND) =
    (SCALAR(-1)/this->accel_time_constant) * Eigen::Matrix<SCALAR,3,3,OPTIONS>::Identity().array();

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,DIM_S::CONTROL_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS> BasicModel<DIM_S,SCALAR,OPTIONS>::
  getTruthStateDynamicsPDWRControl(
    const SCALAR                                                                /* time */,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,  OPTIONS>>& truth_state,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS>>& /* control_input */)
{
  Eigen::Matrix<SCALAR,DIM_S::CONTROL_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS> output;

  // Helper variables
  Eigen::Matrix<SCALAR,3,3,OPTIONS> body_to_ned_rotation =
    math::quat::quaternionToDirectionCosineMatrix(truth_state.template middleCols<4>(DIM_S::TRUTH::QUAT_START_IND));

  output.setZero();
  output.template block<3,3>(DIM_S::CONTROL::ACCEL_START_IND, DIM_S::TRUTH_DISP::VEL_START_IND)   = body_to_ned_rotation.transpose();
  output.template block<3,3>(DIM_S::CONTROL::GYRO_START_IND,  DIM_S::TRUTH_DISP::EULER_START_IND) = body_to_ned_rotation.transpose();

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_NOISE_DIM,OPTIONS> BasicModel<DIM_S,SCALAR,OPTIONS>::
  getTruthStateDynamicsPDWRNoise(
    const SCALAR                                                                /* time */,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,  OPTIONS>>& /* truth_state */,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS>>& /* control_input */)
{
  Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::TRUTH_NOISE_DIM,OPTIONS> output;

  output.setZero();
  output(DIM_S::ERROR::HEADING_BIAS_IND,      DIM_S::TRUTH_NOISE::HEADING_BIAS_IND)       = 1;
  output(DIM_S::ERROR::ABS_PRESSURE_BIAS_IND, DIM_S::TRUTH_NOISE::ABS_PRESSURE_BIAS_IND)  = 1;
  output(DIM_S::ERROR::FEATURE_RANGE_BIAS_IND,DIM_S::TRUTH_NOISE::FEATURE_RANGE_BIAS_IND) = 1;
  output.template block<3,3>(DIM_S::ERROR::FEATURE_BEARING_BIAS_START_IND,DIM_S::TRUTH_NOISE::FEATURE_BEARING_BIAS_START_IND).setIdentity();
  output.template block<3,3>(DIM_S::ERROR::GPS_POS_BIAS_START_IND,        DIM_S::TRUTH_NOISE::GPS_POS_BIAS_START_IND).        setIdentity();
  output.template block<3,3>(DIM_S::ERROR::GYRO_BIAS_START_IND,           DIM_S::TRUTH_NOISE::GYRO_BIAS_START_IND).           setIdentity();
  output.template block<3,3>(DIM_S::ERROR::ACCEL_BIAS_START_IND,          DIM_S::TRUTH_NOISE::ACCEL_BIAS_START_IND).          setIdentity();

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS> BasicModel<DIM_S,SCALAR,OPTIONS>::
  getNavStateDynamicsPDWRErrorState(
    const SCALAR                                                                  /* time */,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,      OPTIONS>>& nav_state,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,      OPTIONS>>& ref_state,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::INER_MEAS_DIM,OPTIONS>>& inertial_reading)
{
  Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS> output;

  // Helper variables
  Eigen::Matrix<SCALAR,3,3,OPTIONS> body_to_ned_rotation =
    math::quat::quaternionToDirectionCosineMatrix(nav_state.template middleCols<4>(DIM_S::NAV::QUAT_START_IND));

  output.setZero();
  // Position terms
  output.template block<3,3>(DIM_S::ERROR::POS_START_IND, DIM_S::ERROR::VEL_START_IND).setIdentity();
  // Euler angles terms
  output.template block<3,3>(DIM_S::ERROR::EULER_START_IND, DIM_S::ERROR::GYRO_BIAS_START_IND) = -body_to_ned_rotation;
  // Velocity terms
  output.template block<3,3>(DIM_S::ERROR::VEL_START_IND, DIM_S::ERROR::EULER_START_IND).noalias() =
    -math::crossProductMatrix(body_to_ned_rotation * (inertial_reading.template middleCols<3>(DIM_S::INER_MEAS::ACCEL_START_IND) -
                                                      nav_state.       template middleCols<3>(DIM_S::NAV::ACCEL_BIAS_START_IND)).transpose());
  output.template block<3,3>(DIM_S::ERROR::VEL_START_IND, DIM_S::ERROR::ACCEL_BIAS_START_IND) =
    -body_to_ned_rotation;
  // Other bias terms
  output(DIM_S::ERROR::HEADING_BIAS_IND,      DIM_S::ERROR::HEADING_BIAS_IND)       = SCALAR(-1)/this->heading_time_constant;
  output(DIM_S::ERROR::ABS_PRESSURE_BIAS_IND, DIM_S::ERROR::ABS_PRESSURE_BIAS_IND)  = SCALAR(-1)/this->abs_pressure_time_constant;
  output(DIM_S::ERROR::FEATURE_RANGE_BIAS_IND,DIM_S::ERROR::FEATURE_RANGE_BIAS_IND) = SCALAR(-1)/this->feature_range_time_constant;
  output.template block<3,3>(DIM_S::ERROR::FEATURE_BEARING_BIAS_START_IND, DIM_S::ERROR::FEATURE_BEARING_BIAS_START_IND) =
    (SCALAR(-1)/this->feature_bearing_time_constant) * Eigen::Matrix<SCALAR,3,3,OPTIONS>::Identity().array();
  output.template block<3,3>(DIM_S::ERROR::GPS_POS_BIAS_START_IND, DIM_S::ERROR::GPS_POS_BIAS_START_IND) =
    (SCALAR(-1)/this->gps_position_time_constant) * Eigen::Matrix<SCALAR,3,3,OPTIONS>::Identity().array();
  // Gyroscope bias terms
  output.template block<3,3>(DIM_S::ERROR::GYRO_BIAS_START_IND, DIM_S::ERROR::GYRO_BIAS_START_IND) =
    (SCALAR(-1)/this->gyro_time_constant) * Eigen::Matrix<SCALAR,3,3,OPTIONS>::Identity().array();
  // Accelerometer bias terms
  output.template block<3,3>(DIM_S::ERROR::ACCEL_BIAS_START_IND, DIM_S::ERROR::ACCEL_BIAS_START_IND) =
    (SCALAR(-1)/this->accel_time_constant) * Eigen::Matrix<SCALAR,3,3,OPTIONS>::Identity().array();

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,DIM_S::ERROR_DIM,OPTIONS> BasicModel<DIM_S,SCALAR,OPTIONS>::
  getNavStateDynamicsPDWRInertialMeasurement(
    const SCALAR                                                                  /* time */,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,      OPTIONS>>& nav_state,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,      OPTIONS>>& ref_state,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::INER_MEAS_DIM,OPTIONS>>& /* inertial_reading */)
{
  Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,DIM_S::ERROR_DIM,OPTIONS> output;

  // Helper variables
  Eigen::Matrix<SCALAR,3,3,OPTIONS> body_to_ned_rotation =
    math::quat::quaternionToDirectionCosineMatrix(nav_state.template middleCols<4>(DIM_S::NAV::QUAT_START_IND));

  output.setZero();
  output.template block<3,3>(DIM_S::INER_MEAS::ACCEL_START_IND,DIM_S::ERROR::VEL_START_IND)   = body_to_ned_rotation.transpose();
  output.template block<3,3>(DIM_S::INER_MEAS::GYRO_START_IND, DIM_S::ERROR::EULER_START_IND) = body_to_ned_rotation.transpose();

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR BasicModel<DIM_S,SCALAR,OPTIONS>::
  findTimeStep(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,OPTIONS>>& prev_ref_state,
               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,OPTIONS>>& cur_ref_state)
{
  const SCALAR dist_traveled = (prev_ref_state.template middleCols<3>(DIM_S::REF::POS_START_IND) -
                                cur_ref_state. template middleCols<3>(DIM_S::REF::POS_START_IND)).norm();
  return dist_traveled / this->nominal_velocity;
}
} // namespace dynamics
} // namespace kf

#endif
/* basic_model.hpp */
