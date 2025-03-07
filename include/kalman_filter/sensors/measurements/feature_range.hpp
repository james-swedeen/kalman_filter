/**
 * @File: feature_range.hpp
 * @Date: October 2022
 * @Author: James Swedeen
 *
 * @brief
 * A class for simulating measurements between the vehicle and feature locations.
 **/

#ifndef KALMAN_FILTER_SENSORS_MEASUREMENTS_FEATURE_RANGE_HPP
#define KALMAN_FILTER_SENSORS_MEASUREMENTS_FEATURE_RANGE_HPP

/* C++ Headers */
#include<cstdint>
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* ROS Headers */
#include<rclcpp/rclcpp.hpp>

/* Local Headers */
#include<kalman_filter/sensors/measurements/measurement_base.hpp>
#include<kalman_filter/noise/noise_base.hpp>
#include<kalman_filter/noise/normal_distribution.hpp>

namespace kf
{
namespace sensors
{
template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
class FeatureRange;

template<typename DIM_S, bool ENABLED, bool USE_BIAS = true, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using FeatureRangePtr = std::shared_ptr<FeatureRange<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>>;

/**
 * @makeFeatureRange
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
 * A fully constructed FeatureRange.
 **/
template<typename DIM_S, bool USE_BIAS = true, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
MeasurementBasePtr<1,DIM_S,SCALAR,OPTIONS> makeFeatureRange(const rclcpp::Node::SharedPtr& node, const std::string& prefix);

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
class FeatureRange
 : public MeasurementBase<1,DIM_S,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  FeatureRange() = delete;
  /**
   * @Copy Constructor
   **/
  FeatureRange(const FeatureRange&) = default;
  /**
   * @Move Constructor
   **/
  FeatureRange(FeatureRange&&) = default;
  /**
   * @Constructor
   *
   * @brief
   * Initializes the object for use.
   *
   * @parameters
   * measurement_period: The time between this measurement's updates
   * feature_range: The x,y range over which this feature can be seen
   * feature_location: The location of the feature this object looks for
   **/
  template<typename DERIVED>
  FeatureRange(const SCALAR                      measurement_period,
               const SCALAR                      feature_range,
               const Eigen::MatrixBase<DERIVED>& feature_location);
  /**
   * @Deconstructor
   **/
  ~FeatureRange() override = default;
  /**
   * @Assignment Operators
   **/
  FeatureRange& operator=(const FeatureRange&)  = default;
  FeatureRange& operator=(      FeatureRange&&) = default;
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
  inline Eigen::Matrix<SCALAR,1,1,OPTIONS>
    getMeasurement(const SCALAR                                                              time,
                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& truth_state,
                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,1,               OPTIONS>>& measurement_noise) override;
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
  inline Eigen::Matrix<SCALAR,1,1,OPTIONS>
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
  inline Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS>
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
  inline Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DISP_DIM,OPTIONS>
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
  SCALAR                            feature_range;
  Eigen::Matrix<SCALAR,1,3,OPTIONS> feature_location;
};

template<typename DIM_S, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
MeasurementBasePtr<1,DIM_S,SCALAR,OPTIONS> makeFeatureRange(const rclcpp::Node::SharedPtr& node, const std::string& prefix)
{
  node->declare_parameter(prefix + ".enabled",            rclcpp::PARAMETER_BOOL);
  node->declare_parameter(prefix + ".measurement_period", rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".feature_range",      rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".feature_location",   rclcpp::PARAMETER_DOUBLE_ARRAY);

  const bool                enabled            = node->get_parameter(prefix + ".enabled").as_bool();
  const SCALAR              measurement_period = node->get_parameter(prefix + ".measurement_period").as_double();
  const SCALAR              feature_range      = node->get_parameter(prefix + ".feature_range").as_double();
  const std::vector<double> feature_location   = node->get_parameter(prefix + ".feature_location").as_double_array();
  assert(3 == feature_location.size());

  if(enabled)
  {
    return std::make_shared<FeatureRange<DIM_S,true,USE_BIAS,SCALAR,OPTIONS>>(
             measurement_period,
             feature_range,
             Eigen::Map<const Eigen::Matrix<double,1,3,OPTIONS>>(feature_location.data()));
  }
  else // Not enabled
  {
    return std::make_shared<FeatureRange<DIM_S,false,USE_BIAS,SCALAR,OPTIONS>>(
             measurement_period,
             feature_range,
             Eigen::Map<const Eigen::Matrix<double,1,3,OPTIONS>>(feature_location.data()));
  }
}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename DERIVED>
FeatureRange<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
  FeatureRange(const SCALAR                      measurement_period,
               const SCALAR                      feature_range,
               const Eigen::MatrixBase<DERIVED>& feature_location)
 : MeasurementBase<1,DIM_S,SCALAR,OPTIONS>(Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS>::Constant(std::numeric_limits<SCALAR>::quiet_NaN())),
   measurement_period(measurement_period),
   feature_range(feature_range),
   feature_location(feature_location)
{
  static_assert((int(DERIVED::RowsAtCompileTime) == 1) or (int(DERIVED::RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED::ColsAtCompileTime) == 3) or (int(DERIVED::ColsAtCompileTime) == Eigen::Dynamic));
  assert(feature_location.rows() == 1);
  assert(feature_location.cols() == 3);
}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,1,OPTIONS> FeatureRange<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
  getMeasurement(const SCALAR                                                              /* time */,
                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& truth_state,
                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,1,               OPTIONS>>& measurement_noise)
{
  Eigen::Matrix<SCALAR,1,1,OPTIONS> output;

  output[0] = (this->feature_location - truth_state.template middleCols<3>(DIM_S::TRUTH::POS_START_IND)).norm() +
              measurement_noise[0];
  if constexpr(USE_BIAS)
  {
    output += truth_state.template middleCols<1>(DIM_S::TRUTH::FEATURE_RANGE_BIAS_IND);
  }

  return output;
}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,1,OPTIONS> FeatureRange<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
  estimateMeasurement(const SCALAR                                                            /* time */,
                      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& nav_state)
{
  Eigen::Matrix<SCALAR,1,1,OPTIONS> output;

  output[0] = (this->feature_location - nav_state.template middleCols<3>(DIM_S::NAV::POS_START_IND)).norm();
  if constexpr(USE_BIAS)
  {
    output += nav_state.template middleCols<1>(DIM_S::NAV::FEATURE_RANGE_BIAS_IND);
  }

  return output;
}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS> FeatureRange<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
  getMeasurementEstimatePDWRErrorState(const SCALAR                                                            /* time */,
                                       const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& nav_state)
{
  Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS> output = Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS>::Zero();
  const Eigen::Matrix<SCALAR,1,3,OPTIONS>          diff   = this->feature_location - nav_state.template middleCols<3>(DIM_S::NAV::POS_START_IND);

  output.template middleCols<3>(DIM_S::ERROR::POS_START_IND) = -diff.array() / diff.norm();
  if constexpr(USE_BIAS)
  {
    output[DIM_S::ERROR::FEATURE_RANGE_BIAS_IND] = 1;
  }

  return output;
}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DISP_DIM,OPTIONS> FeatureRange<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
  getMeasurementPDWRDispersionState(const SCALAR                                                              /* time */,
                                    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& truth_state)
{
  Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DISP_DIM,OPTIONS> output = Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DISP_DIM,OPTIONS>::Zero();
  const Eigen::Matrix<SCALAR,1,3,OPTIONS>               diff   = this->feature_location - truth_state.template middleCols<3>(DIM_S::TRUTH::POS_START_IND);

  output.template middleCols<3>(DIM_S::TRUTH_DISP::POS_START_IND) = -diff.array() / diff.norm();
  if constexpr(USE_BIAS)
  {
    output[DIM_S::TRUTH_DISP::FEATURE_RANGE_BIAS_IND] = 1;
  }

  return output;
}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool FeatureRange<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
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
inline SCALAR FeatureRange<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
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
inline void FeatureRange<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
  setFeature(const SCALAR feature_range, const Eigen::MatrixBase<DERIVED>& feature_location)
{
  this->feature_range    = feature_range;
  this->feature_location = feature_location;
}
} // namespace sensors
} // namespace kf

#endif
/* feature_range.hpp */
