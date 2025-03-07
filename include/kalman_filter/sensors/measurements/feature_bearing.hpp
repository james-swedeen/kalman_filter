/**
 * @File: feature_bearing.hpp
 * @Date: October 2022
 * @Author: James Swedeen
 *
 * @brief
 * A class for simulating measurements between the vehicle and feature locations.
 **/

#ifndef KALMAN_FILTER_SENSORS_MEASUREMENTS_FEATURE_BEARING_HPP
#define KALMAN_FILTER_SENSORS_MEASUREMENTS_FEATURE_BEARING_HPP

/* C++ Headers */
#include<cstdint>
#include<memory>
#include<type_traits>

/* Eigen Headers */
#include<Eigen/Dense>

/* ROS Headers */
#include<rclcpp/rclcpp.hpp>

/* Local Headers */
#include<kalman_filter/math/quaternion.hpp>
#include<kalman_filter/sensors/measurements/measurement_base.hpp>
#include<kalman_filter/noise/noise_base.hpp>
#include<kalman_filter/noise/normal_distribution.hpp>
#include<kalman_filter/dynamics/dubins_airplane_model.hpp>

namespace kf
{
namespace sensors
{
template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
class FeatureBearing;

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using FeatureBearingPtr = std::shared_ptr<FeatureBearing<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>>;

/**
 * @makeFeatureBearing
 *
 * @brief
 * Helper function that uses information from the ROS parameter server to construct the given object.
 *
 * @templates
 * DIM_S: The type of a Dimensions object or an inheriting object that has information about the size of the state vectors
 * USE_BIAS: True if you want to use additive biases from the truth and navigation states
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 *
 * @parameters
 * node: The node with the right namespacing to have access to the parameters needed
 * prefix: The prefix of the parameter names
 *
 * @return
 * A fully constructed FeatureBearing.
 **/
template<typename DIM_S, bool USE_BIAS = true, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
MeasurementBasePtr<2,DIM_S,SCALAR,OPTIONS> makeFeatureBearing(const rclcpp::Node::SharedPtr& node, const std::string& prefix);

/**
 * @DIM_S
 * The type of a Dimensions object or an inheriting object that has information about the size of the state vectors.
 *
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @ENABLED
 * False if and only if you want this measurement source to be turned off.
 *
 * @USE_BIAS
 * True if you want to use additive biases from the truth and navigation states.
 *
 * @OPTIONS
 * Eigen Matrix options.
 **/
template<typename DIM_S, bool ENABLED, bool USE_BIAS = true, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class FeatureBearing
 : public MeasurementBase<2,DIM_S,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  FeatureBearing() = delete;
  /**
   * @Copy Constructor
   **/
  FeatureBearing(const FeatureBearing&) = default;
  /**
   * @Move Constructor
   **/
  FeatureBearing(FeatureBearing&&) = default;
  /**
   * @Constructor
   *
   * @brief
   * Initializes the object for use.
   *
   * @parameters
   * measurement_period: The time between this measurement's updates
   * camera_offset: The offset vector from the body frame to the camera
   * camera_viewing_angles: The roll, pitch, and yaw of the body to camera rotation
   * feature_range: The x,y range over which this feature can be seen
   * feature_location: The location of the feature this object looks for
   **/
  template<typename DERIVED_CO, typename DERIVED_CVA, typename DERIVED_FL>
  FeatureBearing(const SCALAR                          measurement_period,
                 const Eigen::MatrixBase<DERIVED_CO>&  camera_offset,
                 const Eigen::MatrixBase<DERIVED_CVA>& camera_viewing_angles,
                 const SCALAR                          feature_range,
                 const Eigen::MatrixBase<DERIVED_FL>&  feature_location);
  /**
   * @Deconstructor
   **/
  ~FeatureBearing() override = default;
  /**
   * @Assignment Operators
   **/
  FeatureBearing& operator=(const FeatureBearing&)  = default;
  FeatureBearing& operator=(      FeatureBearing&&) = default;
  /**
   * @getMeasurement
   *
   * @brief
   * Used to synthesize a measurement.
   *
   * @parameters
   * time: The current simulation time
   * truth_state: The current truth state vector
   * measurement_noise: The vector of additive measurement noise to be added to the measurement
   *
   * @return
   * The vector of measurements.
   **/
  inline Eigen::Matrix<SCALAR,1,2,OPTIONS>
    getMeasurement(const SCALAR                                                              time,
                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& truth_state,
                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,2,               OPTIONS>>& measurement_noise) override;
  /**
   * @estimateMeasurement
   *
   * @brief
   * Used to estimate a measurement given the navigation state.
   *
   * @parameters
   * time: The current simulation time
   * nav_state: The current navigation state vector
   *
   * @return
   * The vector of measurements.
   **/
  inline Eigen::Matrix<SCALAR,1,2,OPTIONS>
    estimateMeasurement(const SCALAR                                                            time,
                        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& nav_state) override;
  /**
   * @getMeasurementEstimatePDWRErrorState
   *
   * @brief
   * Finds the derivative of "estimateMeasurement" with respect to the error state.
   *
   * @parameters
   * time: The current simulation time
   * nav_state: The current navigation state vector
   *
   * @return
   * The measurement matrix.
   **/
  inline Eigen::Matrix<SCALAR,2,DIM_S::ERROR_DIM,OPTIONS>
    getMeasurementEstimatePDWRErrorState(
      const SCALAR                                                            time,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& nav_state) override;
  /**
   * @getMeasurementPDWRDispersionState
   *
   * @brief
   * Finds the derivative of "getMeasurement" with respect to the truth dispersion state.
   *
   * @parameters
   * time: The current simulation time
   * truth_state: The current truth state vector
   *
   * @return
   * The measurement matrix.
   **/
  inline Eigen::Matrix<SCALAR,2,DIM_S::TRUTH_DISP_DIM,OPTIONS>
    getMeasurementPDWRDispersionState(
      const SCALAR                                                              time,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& truth_state) override;
  /**
   * @measurementReady
   *
   * @brief
   * Returns true if and only if it is time to use a measurement.
   *
   * @parameters
   * time: The current simulation time
   * next_measurement_time: The time that the next measurement will be ready to use
   * truth_state: The current truth state vector
   *
   * @return
   * True if and only if it is time to use a measurement.
   **/
  inline bool measurementReady(const SCALAR                                                              time,
                               const SCALAR                                                              next_measurement_time,
                               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& truth_state) override;
  /**
   * @updateNextMeasurementTime
   *
   * @brief
   * Finds the next time that a measurement should be applied.
   *
   * @parameters
   * time: The current simulation time
   * next_measurement_time: The old time that the next measurement will be ready to use
   *
   * @return
   * The new time that the next measurement will be ready to use
   **/
  inline SCALAR updateNextMeasurementTime(const SCALAR time, const SCALAR next_measurement_time) override;
  /**
   * @setFeature
   *
   * @brief
   * Sets the internal information about the feature.
   *
   * @parameters
   * feature_range: The x,y range over which this feature can be seen
   * feature_location: The location of the feature this object looks for
   **/
  template<typename DERIVED>
  inline void setFeature(const SCALAR feature_range, const Eigen::MatrixBase<DERIVED>& feature_location);
private:
  SCALAR                            measurement_period;
  Eigen::Matrix<SCALAR,1,3,OPTIONS> camera_offset;
  Eigen::Matrix<SCALAR,3,3,OPTIONS> camera_frame_rotation;
  SCALAR                            feature_range;
  Eigen::Matrix<SCALAR,1,3,OPTIONS> feature_location;
};

template<typename DIM_S, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
MeasurementBasePtr<2,DIM_S,SCALAR,OPTIONS> makeFeatureBearing(const rclcpp::Node::SharedPtr& node, const std::string& prefix)
{
  node->declare_parameter(prefix + ".enabled",               rclcpp::PARAMETER_BOOL);
  node->declare_parameter(prefix + ".measurement_period",    rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".camera_offset",         rclcpp::PARAMETER_DOUBLE_ARRAY);
  node->declare_parameter(prefix + ".camera_viewing_angles", rclcpp::PARAMETER_DOUBLE_ARRAY);
  node->declare_parameter(prefix + ".feature_range",         rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".feature_location",      rclcpp::PARAMETER_DOUBLE_ARRAY);

  const bool                enabled               = node->get_parameter(prefix + ".enabled").as_bool();
  const SCALAR              measurement_period    = node->get_parameter(prefix + ".measurement_period").as_double();
  const std::vector<double> camera_offset         = node->get_parameter(prefix + ".camera_offset").as_double_array();
  const std::vector<double> camera_viewing_angles = node->get_parameter(prefix + ".camera_viewing_angles").as_double_array();
  const SCALAR              feature_range         = node->get_parameter(prefix + ".feature_range").as_double();
  const std::vector<double> feature_location      = node->get_parameter(prefix + ".feature_location").as_double_array();
  assert(3 == camera_offset.size());
  assert(3 == camera_viewing_angles.size());
  assert(3 == feature_location.size());

  if(enabled)
  {
    return std::make_shared<FeatureBearing<DIM_S,true,USE_BIAS,SCALAR,OPTIONS>>(
             measurement_period,
             Eigen::Map<const Eigen::Matrix<double,1,3,OPTIONS>>(camera_offset.data()),
             Eigen::Map<const Eigen::Matrix<double,1,3,OPTIONS>>(camera_viewing_angles.data()),
             feature_range,
             Eigen::Map<const Eigen::Matrix<double,1,3,OPTIONS>>(feature_location.data()));
  }
  else // Not enabled
  {
    return std::make_shared<FeatureBearing<DIM_S,false,USE_BIAS,SCALAR,OPTIONS>>(
             measurement_period,
             Eigen::Map<const Eigen::Matrix<double,1,3,OPTIONS>>(camera_offset.data()),
             Eigen::Map<const Eigen::Matrix<double,1,3,OPTIONS>>(camera_viewing_angles.data()),
             feature_range,
             Eigen::Map<const Eigen::Matrix<double,1,3,OPTIONS>>(feature_location.data()));
  }
}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename DERIVED_CO, typename DERIVED_CVA, typename DERIVED_FL>
FeatureBearing<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
  FeatureBearing(const SCALAR                          measurement_period,
                 const Eigen::MatrixBase<DERIVED_CO>&  camera_offset,
                 const Eigen::MatrixBase<DERIVED_CVA>& camera_viewing_angles,
                 const SCALAR                          feature_range,
                 const Eigen::MatrixBase<DERIVED_FL>&  feature_location)
 : MeasurementBase<2,DIM_S,SCALAR,OPTIONS>(Eigen::Matrix<SCALAR,2,DIM_S::ERROR_DIM,OPTIONS>::Constant(std::numeric_limits<SCALAR>::quiet_NaN())),
   measurement_period(measurement_period),
   camera_offset(camera_offset),
   camera_frame_rotation(math::quat::rollPitchYawToDirectionCosineMatrix(camera_viewing_angles[0],
                                                                         camera_viewing_angles[1],
                                                                         camera_viewing_angles[2])),
   feature_range(feature_range),
   feature_location(feature_location)
{
  static_assert((int(DERIVED_CO:: RowsAtCompileTime) == 1) or (int(DERIVED_CO:: RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED_CO:: ColsAtCompileTime) == 3) or (int(DERIVED_CO:: ColsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED_CVA::RowsAtCompileTime) == 1) or (int(DERIVED_CVA::RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED_CVA::ColsAtCompileTime) == 3) or (int(DERIVED_CVA::ColsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED_FL:: RowsAtCompileTime) == 1) or (int(DERIVED_FL:: RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED_FL:: ColsAtCompileTime) == 3) or (int(DERIVED_FL:: ColsAtCompileTime) == Eigen::Dynamic));
  assert(camera_offset.        rows() == 1);
  assert(camera_offset.        cols() == 3);
  assert(camera_viewing_angles.rows() == 1);
  assert(camera_viewing_angles.cols() == 3);
  assert(feature_location.     rows() == 1);
  assert(feature_location.     cols() == 3);
}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,2,OPTIONS> FeatureBearing<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
  getMeasurement(const SCALAR                                                              /* time */,
                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& truth_state,
                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,2,               OPTIONS>>& measurement_noise)
{
  Eigen::Matrix<SCALAR,1,2,OPTIONS> output;

  Eigen::Matrix<SCALAR,3,3,OPTIONS> rot_cam_bias;
  if constexpr(USE_BIAS)
  {
    rot_cam_bias = math::quat::rollPitchYawToDirectionCosineMatrix(truth_state[DIM_S::TRUTH::FEATURE_BEARING_ROLL_BIAS_IND],
                                                                   truth_state[DIM_S::TRUTH::FEATURE_BEARING_PITCH_BIAS_IND],
                                                                   truth_state[DIM_S::TRUTH::FEATURE_BEARING_YAW_BIAS_IND]);
  }
  else
  {
    rot_cam_bias = Eigen::Matrix<SCALAR,3,3,OPTIONS>::Identity();
  }
  Eigen::Matrix<SCALAR,3,3,OPTIONS> rot_ned_body;
  if constexpr(std::is_same<DIM_S,dynamics::DubinsAirplaneDimBase<DIM_S::USE_STEADY_STATE_ERROR_COV,true>>:: value or
               std::is_same<DIM_S,dynamics::DubinsAirplaneDimBase<DIM_S::USE_STEADY_STATE_ERROR_COV,false>>::value)
  {
    rot_ned_body = math::quat::rollPitchYawToDirectionCosineMatrix(truth_state.template middleCols<3>(DIM_S::TRUTH::EULER_START_IND)).transpose();
  }
  else
  {
    rot_ned_body = math::quat::quaternionToDirectionCosineMatrix(truth_state.template middleCols<4>(DIM_S::TRUTH::QUAT_START_IND)).transpose();
  }
  const Eigen::Matrix<SCALAR,1,3,OPTIONS> diff_vec_ned  = this->feature_location - truth_state.template middleCols<3>(DIM_S::TRUTH::POS_START_IND);
  const Eigen::Matrix<SCALAR,1,3,OPTIONS> diff_vec_body = rot_ned_body * diff_vec_ned.transpose();
  const Eigen::Matrix<SCALAR,1,3,OPTIONS> rel_los_vec   = rot_cam_bias * this->camera_frame_rotation * (diff_vec_body - this->camera_offset).transpose();

  output[0] = rel_los_vec[0] / rel_los_vec[2];
  output[1] = rel_los_vec[1] / rel_los_vec[2];

  return output + measurement_noise;
}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,2,OPTIONS> FeatureBearing<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
  estimateMeasurement(const SCALAR                                                            /* time */,
                      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& nav_state)
{
  Eigen::Matrix<SCALAR,1,2,OPTIONS> output;

  Eigen::Matrix<SCALAR,3,3,OPTIONS> rot_cam_bias;
  if constexpr(USE_BIAS)
  {
    rot_cam_bias = math::quat::rollPitchYawToDirectionCosineMatrix(nav_state[DIM_S::NAV::FEATURE_BEARING_ROLL_BIAS_IND],
                                                                   nav_state[DIM_S::NAV::FEATURE_BEARING_PITCH_BIAS_IND],
                                                                   nav_state[DIM_S::NAV::FEATURE_BEARING_YAW_BIAS_IND]);
  }
  else
  {
    rot_cam_bias = Eigen::Matrix<SCALAR,3,3,OPTIONS>::Identity();
  }
  const Eigen::Matrix<SCALAR,3,3,OPTIONS> rot_ned_body  = math::quat::quaternionToDirectionCosineMatrix(nav_state.template middleCols<4>(DIM_S::NAV::QUAT_START_IND)).transpose();
  const Eigen::Matrix<SCALAR,1,3,OPTIONS> diff_vec_ned  = this->feature_location - nav_state.template middleCols<3>(DIM_S::NAV::POS_START_IND);
  const Eigen::Matrix<SCALAR,1,3,OPTIONS> diff_vec_body = rot_ned_body * diff_vec_ned.transpose();
  const Eigen::Matrix<SCALAR,1,3,OPTIONS> rel_los_vec   = rot_cam_bias * this->camera_frame_rotation * (diff_vec_body - this->camera_offset).transpose();

  output[0] = rel_los_vec[0] / rel_los_vec[2];
  output[1] = rel_los_vec[1] / rel_los_vec[2];

  return output;
}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,2,DIM_S::ERROR_DIM,OPTIONS> FeatureBearing<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
  getMeasurementEstimatePDWRErrorState(const SCALAR                                                            /* time */,
                                       const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& nav_state)
{
  Eigen::Matrix<SCALAR,2,DIM_S::ERROR_DIM,OPTIONS> output = Eigen::Matrix<SCALAR,2,DIM_S::ERROR_DIM,OPTIONS>::Zero();
  Eigen::Matrix<SCALAR,2,3,OPTIONS>                out_diff_rel_los;

  Eigen::Matrix<SCALAR,3,3,OPTIONS> rot_cam_bias;
  if constexpr(USE_BIAS)
  {
    rot_cam_bias = math::quat::rollPitchYawToDirectionCosineMatrix(nav_state[DIM_S::NAV::FEATURE_BEARING_ROLL_BIAS_IND],
                                                                   nav_state[DIM_S::NAV::FEATURE_BEARING_PITCH_BIAS_IND],
                                                                   nav_state[DIM_S::NAV::FEATURE_BEARING_YAW_BIAS_IND]);
  }
  else
  {
    rot_cam_bias = Eigen::Matrix<SCALAR,3,3,OPTIONS>::Identity();
  }
  const Eigen::Matrix<SCALAR,3,3,OPTIONS> rot_ned_body  = math::quat::quaternionToDirectionCosineMatrix(nav_state.template middleCols<4>(DIM_S::NAV::QUAT_START_IND)).transpose();
  const Eigen::Matrix<SCALAR,3,3,OPTIONS> rot_cam       = rot_cam_bias * this->camera_frame_rotation;
  const Eigen::Matrix<SCALAR,1,3,OPTIONS> diff_vec_ned  = this->feature_location - nav_state.template middleCols<3>(DIM_S::NAV::POS_START_IND);
  const Eigen::Matrix<SCALAR,1,3,OPTIONS> diff_vec_body = rot_ned_body * diff_vec_ned.transpose();
  const Eigen::Matrix<SCALAR,1,3,OPTIONS> rel_los_vec   = rot_cam * (diff_vec_body - this->camera_offset).transpose();

  out_diff_rel_los(0,0) = SCALAR(1) / rel_los_vec[2];
  out_diff_rel_los(0,1) = 0;
  out_diff_rel_los(0,2) = -rel_los_vec[0] / std::pow(rel_los_vec[2], 2);
  out_diff_rel_los(1,0) = 0;
  out_diff_rel_los(1,1) = SCALAR(1) / rel_los_vec[2];
  out_diff_rel_los(1,2) = -rel_los_vec[1] / std::pow(rel_los_vec[2], 2);

  output.template middleCols<3>(DIM_S::ERROR::POS_START_IND) = -out_diff_rel_los *
                                                               rot_cam *
                                                               rot_ned_body;
  output.template middleCols<3>(DIM_S::ERROR::EULER_START_IND) = out_diff_rel_los *
                                                                 rot_cam *
                                                                 math::crossProductMatrix(rot_ned_body * diff_vec_ned.transpose());
  if constexpr(USE_BIAS)
  {
    output.template middleCols<3>(DIM_S::ERROR::FEATURE_BEARING_BIAS_START_IND) = out_diff_rel_los *
                                                                                  math::crossProductMatrix(rot_cam * (diff_vec_body - this->camera_offset).transpose()).transpose();
  }

  return output;
}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,2,DIM_S::TRUTH_DISP_DIM,OPTIONS> FeatureBearing<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
  getMeasurementPDWRDispersionState(const SCALAR                                                              /* time */,
                                    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& truth_state)
{
  Eigen::Matrix<SCALAR,2,DIM_S::TRUTH_DISP_DIM,OPTIONS> output = Eigen::Matrix<SCALAR,2,DIM_S::TRUTH_DISP_DIM,OPTIONS>::Zero();
  Eigen::Matrix<SCALAR,2,3,OPTIONS>                     out_diff_rel_los;

  Eigen::Matrix<SCALAR,3,3,OPTIONS> rot_cam_bias;
  if constexpr(USE_BIAS)
  {
    rot_cam_bias = math::quat::rollPitchYawToDirectionCosineMatrix(truth_state[DIM_S::TRUTH::FEATURE_BEARING_ROLL_BIAS_IND],
                                                                   truth_state[DIM_S::TRUTH::FEATURE_BEARING_PITCH_BIAS_IND],
                                                                   truth_state[DIM_S::TRUTH::FEATURE_BEARING_YAW_BIAS_IND]);
  }
  else
  {
    rot_cam_bias = Eigen::Matrix<SCALAR,3,3,OPTIONS>::Identity();
  }
  Eigen::Matrix<SCALAR,3,3,OPTIONS> rot_ned_body;
  if constexpr(std::is_same<DIM_S,dynamics::DubinsAirplaneDimBase<DIM_S::USE_STEADY_STATE_ERROR_COV,true>>:: value or
               std::is_same<DIM_S,dynamics::DubinsAirplaneDimBase<DIM_S::USE_STEADY_STATE_ERROR_COV,false>>::value)
  {
    rot_ned_body = math::quat::rollPitchYawToDirectionCosineMatrix(truth_state.template middleCols<3>(DIM_S::TRUTH::EULER_START_IND)).transpose();
  }
  else
  {
    rot_ned_body = math::quat::quaternionToDirectionCosineMatrix(truth_state.template middleCols<4>(DIM_S::TRUTH::QUAT_START_IND)).transpose();
  }
  const Eigen::Matrix<SCALAR,3,3,OPTIONS> rot_cam       = rot_cam_bias * this->camera_frame_rotation;
  const Eigen::Matrix<SCALAR,1,3,OPTIONS> diff_vec_ned  = this->feature_location - truth_state.template middleCols<3>(DIM_S::TRUTH::POS_START_IND);
  const Eigen::Matrix<SCALAR,1,3,OPTIONS> diff_vec_body = rot_ned_body * diff_vec_ned.transpose();
  const Eigen::Matrix<SCALAR,1,3,OPTIONS> rel_los_vec   = rot_cam * (diff_vec_body - this->camera_offset).transpose();

  out_diff_rel_los(0,0) = SCALAR(1) / rel_los_vec[2];
  out_diff_rel_los(0,1) = 0;
  out_diff_rel_los(0,2) = -rel_los_vec[0] / std::pow(rel_los_vec[2], 2);
  out_diff_rel_los(1,0) = 0;
  out_diff_rel_los(1,1) = SCALAR(1) / rel_los_vec[2];
  out_diff_rel_los(1,2) = -rel_los_vec[1] / std::pow(rel_los_vec[2], 2);

  output.template middleCols<3>(DIM_S::TRUTH_DISP::POS_START_IND) = -out_diff_rel_los *
                                                                    rot_cam *
                                                                    rot_ned_body;
  output.template middleCols<3>(DIM_S::TRUTH_DISP::EULER_START_IND) = out_diff_rel_los *
                                                                      rot_cam *
                                                                      math::crossProductMatrix(rot_ned_body * diff_vec_ned.transpose());
  if constexpr(USE_BIAS)
  {
    output.template middleCols<3>(DIM_S::TRUTH_DISP::FEATURE_BEARING_BIAS_START_IND) = out_diff_rel_los *
                                                                                       math::crossProductMatrix(rot_cam * (diff_vec_body - this->camera_offset).transpose()).transpose();
  }

  return output;
}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool FeatureBearing<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
  measurementReady(const SCALAR                                                              time,
                   const SCALAR                                                              next_measurement_time,
                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& truth_state)
{
  if constexpr(ENABLED)
  {
    if(time >= next_measurement_time)
    {
      const SCALAR xy_range = (this->feature_location.template leftCols<2>() - truth_state.template middleCols<2>(DIM_S::TRUTH::POS_START_IND)).norm();
      if(xy_range <= this->feature_range)
      {
        return true;
      }
    }
  }
  return false;
}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR FeatureBearing<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
  updateNextMeasurementTime(const SCALAR time, const SCALAR next_measurement_time)
{
  if(time >= next_measurement_time)
  {
    return next_measurement_time + this->measurement_period;
  }
  return next_measurement_time;
}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename DERIVED>
inline void FeatureBearing<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
  setFeature(const SCALAR feature_range, const Eigen::MatrixBase<DERIVED>& feature_location)
{
  this->feature_range    = feature_range;
  this->feature_location = feature_location;
}
} // namespace sensors
} // namespace kf

#endif
/* feature_bearing.hpp */
