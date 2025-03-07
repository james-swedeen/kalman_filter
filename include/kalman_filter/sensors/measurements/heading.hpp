/**
 * @File: heading.hpp
 * @Date: May 2022
 * @Author: James Swedeen
 *
 * @brief
 * A class for simulating heading measurements.
 **/

#ifndef KALMAN_FILTER_SENSORS_MEASUREMENTS_HEADING_HPP
#define KALMAN_FILTER_SENSORS_MEASUREMENTS_HEADING_HPP

/* C++ Headers */
#include<cstdint>
#include<memory>
#include<type_traits>

/* Eigen Headers */
#include<Eigen/Dense>

/* ROS Headers */
#include<rclcpp/rclcpp.hpp>

/* Local Headers */
#include<kalman_filter/helpers/dimension_struct.hpp>
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
class Heading;

template<typename DIM_S, bool ENABLED, bool USE_BIAS = true, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using HeadingPtr = std::shared_ptr<Heading<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>>;

/**
 * @makeHeading
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
 * A fully constructed Heading.
 **/
template<typename DIM_S, bool USE_BIAS = true, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
MeasurementBasePtr<1,DIM_S,SCALAR,OPTIONS> makeHeading(const rclcpp::Node::SharedPtr& node, const std::string& prefix);

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
class Heading
 : public MeasurementBase<1,DIM_S,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  Heading() = delete;
  /**
   * @Copy Constructor
   **/
  Heading(const Heading&) = default;
  /**
   * @Move Constructor
   **/
  Heading(Heading&&) = default;
  /**
   * @Constructor
   *
   * @brief
   * Initializes the object for use.
   *
   * @parameters
   * measurement_period: The time between this measurement's updates
   * ss_kalman_gain: The steady state kalman gain, note this is only used if the USE_SS_KALMAN_GAIN flag is
   *                 enabled in the measurement controller
   **/
  Heading(const SCALAR                                                              measurement_period,
          const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS>>& ss_kalman_gain = Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS>::Constant(std::numeric_limits<SCALAR>::quiet_NaN()));
  /**
   * @Deconstructor
   **/
  ~Heading() override = default;
  /**
   * @Assignment Operators
   **/
  Heading& operator=(const Heading&)  = default;
  Heading& operator=(      Heading&&) = default;
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
   * Finds the linearized form of "estimateMeasurement" around the error state.
   *
   * @parameters
   * time: The current simulation time
   * nav_state: The current navigation state vector
   *
   * @return
   * The measurement matrix.
   **/
  inline Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS>
    getMeasurementEstimatePDWRErrorState(const SCALAR                                                            time,
                                         const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& nav_state) override;
  /**
   * @getMeasurementPDWRDispersionState
   *
   * @brief
   * Finds the linearized form of "getMeasurement" around the error state.
   *
   * @parameters
   * time: The current simulation time
   * truth_state: The current truth state vector
   *
   * @return
   * The measurement matrix.
   **/
  inline Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DISP_DIM,OPTIONS>
    getMeasurementPDWRDispersionState(const SCALAR                                                              time,
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
private:
  SCALAR measurement_period;
};

template<typename DIM_S, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
MeasurementBasePtr<1,DIM_S,SCALAR,OPTIONS> makeHeading(const rclcpp::Node::SharedPtr& node, const std::string& prefix)
{
  node->declare_parameter(prefix + ".enabled",            rclcpp::PARAMETER_BOOL);
  node->declare_parameter(prefix + ".measurement_period", rclcpp::PARAMETER_DOUBLE);

  const bool   enabled            = node->get_parameter(prefix + ".enabled").as_bool();
  const SCALAR measurement_period = node->get_parameter(prefix + ".measurement_period").as_double();

  if(enabled)
  {
    return std::make_shared<Heading<DIM_S,true,USE_BIAS,SCALAR,OPTIONS>>(measurement_period);
  }
  else // Not enabled
  {
    return std::make_shared<Heading<DIM_S,false,USE_BIAS,SCALAR,OPTIONS>>(measurement_period);
  }
}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
Heading<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
  Heading(const SCALAR                                                              measurement_period,
          const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS>>& ss_kalman_gain)
 : MeasurementBase<1,DIM_S,SCALAR,OPTIONS>(ss_kalman_gain),
   measurement_period(measurement_period)
{}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,1,OPTIONS> Heading<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
  getMeasurement(const SCALAR                                                              /* time */,
                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& truth_state,
                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,1,               OPTIONS>>& measurement_noise)
{
  if constexpr(std::is_same<DIM_S,dynamics::DubinsAirplaneDimBase<DIM_S::USE_STEADY_STATE_ERROR_COV,true>>:: value or
               std::is_same<DIM_S,dynamics::DubinsAirplaneDimBase<DIM_S::USE_STEADY_STATE_ERROR_COV,false>>::value)
  {
    if constexpr(USE_BIAS)
    {
      return truth_state[DIM_S::TRUTH::YAW_IND] +
             truth_state.template middleCols<1>(DIM_S::TRUTH::HEADING_BIAS_IND) +
             measurement_noise;
    }
    else
    {
      return truth_state[DIM_S::TRUTH::YAW_IND] + measurement_noise;
    }
  }
  else
  {
    const Eigen::Matrix<SCALAR,1,3,OPTIONS> roll_pitch_yaw =
      math::quat::quaternionToRollPitchYaw(truth_state.template middleCols<4>(DIM_S::TRUTH::QUAT_START_IND));

    if constexpr(USE_BIAS)
    {
      return roll_pitch_yaw.col(2) +
             truth_state.template middleCols<1>(DIM_S::TRUTH::HEADING_BIAS_IND) +
             measurement_noise;
    }
    else
    {
      return roll_pitch_yaw.col(2) + measurement_noise;
    }
  }
}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,1,OPTIONS> Heading<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
  estimateMeasurement(const SCALAR                                                            /* time */,
                      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& nav_state)
{
  const Eigen::Matrix<SCALAR,1,3,OPTIONS> roll_pitch_yaw =
    math::quat::quaternionToRollPitchYaw(nav_state.template middleCols<4>(DIM_S::NAV::QUAT_START_IND));

  if constexpr(USE_BIAS)
  {
    return roll_pitch_yaw.col(2) + nav_state.template middleCols<1>(DIM_S::NAV::HEADING_BIAS_IND);
  }
  else
  {
    return roll_pitch_yaw.col(2);
  }
}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS> Heading<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
  getMeasurementEstimatePDWRErrorState(const SCALAR                                                            /* time */,
                                       const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& /* nav_state */)
{
  Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS> output = Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS>::Zero();

  output[DIM_S::ERROR::YAW_IND] = 1;
  if constexpr(USE_BIAS)
  {
    output[DIM_S::ERROR::HEADING_BIAS_IND] = 1;
  }

  return output;
}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DISP_DIM,OPTIONS> Heading<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
  getMeasurementPDWRDispersionState(const SCALAR                                                              /* time */,
                                    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& /* truth_state */)
{
  Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DISP_DIM,OPTIONS> output = Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DISP_DIM,OPTIONS>::Zero();

  output[DIM_S::TRUTH_DISP::YAW_IND] = 1;
  if constexpr(USE_BIAS)
  {
    output[DIM_S::TRUTH_DISP::HEADING_BIAS_IND] = 1;
  }

  return output;
}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool Heading<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
  measurementReady(const SCALAR                                                              time,
                   const SCALAR                                                              next_measurement_time,
                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& /* truth_state */)
{
  if constexpr(ENABLED)
  {
    if(time >= next_measurement_time)
    {
      return true;
    }
  }
  return false;
}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR Heading<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
  updateNextMeasurementTime(const SCALAR time, const SCALAR next_measurement_time)
{
  if(time >= next_measurement_time)
  {
    return next_measurement_time + this->measurement_period;
  }
  return next_measurement_time;
}
} // namespace sensors
} // namespace kf

#endif
/* heading.hpp */
