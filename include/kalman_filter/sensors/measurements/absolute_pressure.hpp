/**
 * @File: absolute_pressure.hpp
 * @Date: September 2022
 * @Author: James Swedeen
 *
 * @brief
 * A class for simulating absolute pressure measurement which give you information about altitude.
 **/

#ifndef KALMAN_FILTER_SENSORS_MEASUREMENTS_ABSOLUTE_PRESSURE_HPP
#define KALMAN_FILTER_SENSORS_MEASUREMENTS_ABSOLUTE_PRESSURE_HPP

/* C++ Headers */
#include<cstdint>
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* ROS Headers */
#include<rclcpp/rclcpp.hpp>

/* Local Headers */
#include<kalman_filter/helpers/dimension_struct.hpp>
#include<kalman_filter/sensors/measurements/measurement_base.hpp>
#include<kalman_filter/noise/noise_base.hpp>
#include<kalman_filter/noise/normal_distribution.hpp>

namespace kf
{
namespace sensors
{
template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
class AbsolutePressure;

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using AbsolutePressurePtr = std::shared_ptr<AbsolutePressure<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>>;

/**
 * @makeAbsolutePressure
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
 * A fully constructed AbsolutePressure.
 **/
template<typename DIM_S, bool USE_BIAS = true, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
MeasurementBasePtr<1,DIM_S,SCALAR,OPTIONS> makeAbsolutePressure(const rclcpp::Node::SharedPtr& node, const std::string& prefix);

/**
 * @DIM_S
 * The type of a Dimensions object or an inheriting object that has information about the size of the state vectors.
 *
 * @ENABLED
 * False if and only if you want this measurement source to be turned off.
 *
 * @USE_BIAS
 * True if you want to use additive biases from the truth and navigation states.

 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options.
 **/
template<typename DIM_S, bool ENABLED, bool USE_BIAS = true, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class AbsolutePressure
 : public MeasurementBase<1,DIM_S,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  AbsolutePressure() = delete;
  /**
   * @Copy Constructor
   **/
  AbsolutePressure(const AbsolutePressure&) = default;
  /**
   * @Move Constructor
   **/
  AbsolutePressure(AbsolutePressure&&) = default;
  /**
   * @Constructor
   *
   * @brief
   * Initializes the object for use.
   *
   * @parameters
   * measurement_period: The time between this measurement's updates
   * gravity_magnitude: The magnitude of the acceleration from gravity
   * air_density: The density of the air at the flight location
   * ss_kalman_gain: The steady state kalman gain, note this is only used if the USE_SS_KALMAN_GAIN flag is
   *                 enabled in the measurement controller
   **/
  AbsolutePressure(const SCALAR                                                              measurement_period,
                   const SCALAR                                                              gravity_magnitude,
                   const SCALAR                                                              air_density,
                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS>>& ss_kalman_gain = Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS>::Constant(std::numeric_limits<SCALAR>::quiet_NaN()));
  /**
   * @Deconstructor
   **/
  ~AbsolutePressure() override = default;
  /**
   * @Assignment Operators
   **/
  AbsolutePressure& operator=(const AbsolutePressure&)  = default;
  AbsolutePressure& operator=(      AbsolutePressure&&) = default;
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
private:
  SCALAR measurement_period;
  SCALAR gravity_magnitude;
  SCALAR air_density;
};

template<typename DIM_S, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
MeasurementBasePtr<1,DIM_S,SCALAR,OPTIONS>
  makeAbsolutePressure(const rclcpp::Node::SharedPtr& node, const std::string& prefix)
{
  node->declare_parameter(prefix + ".enabled",            rclcpp::PARAMETER_BOOL);
  node->declare_parameter(prefix + ".measurement_period", rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".gravity_magnitude",  rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".air_density",        rclcpp::PARAMETER_DOUBLE);

  const bool   enabled            = node->get_parameter(prefix + ".enabled").as_bool();
  const SCALAR measurement_period = node->get_parameter(prefix + ".measurement_period").as_double();
  const SCALAR gravity_magnitude  = node->get_parameter(prefix + ".gravity_magnitude").as_double();
  const SCALAR air_density        = node->get_parameter(prefix + ".air_density").as_double();

  if(enabled)
  {
    return std::make_shared<AbsolutePressure<DIM_S,true,USE_BIAS,SCALAR,OPTIONS>>(
             measurement_period,
             gravity_magnitude,
             air_density);
  }
  else // Not enabled
  {
    return std::make_shared<AbsolutePressure<DIM_S,false,USE_BIAS,SCALAR,OPTIONS>>(
             measurement_period,
             gravity_magnitude,
             air_density);
  }
}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
AbsolutePressure<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
  AbsolutePressure(const SCALAR                                                              measurement_period,
                   const SCALAR                                                              gravity_magnitude,
                   const SCALAR                                                              air_density,
                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS>>& ss_kalman_gain)
 : MeasurementBase<1,DIM_S,SCALAR,OPTIONS>(ss_kalman_gain),
   measurement_period(measurement_period),
   gravity_magnitude(gravity_magnitude),
   air_density(air_density)
{}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,1,OPTIONS> AbsolutePressure<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
  getMeasurement(const SCALAR                                                              /* time */,
                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& truth_state,
                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,1,               OPTIONS>>& measurement_noise)
{
  Eigen::Matrix<SCALAR,1,1,OPTIONS> output;

  output = (-truth_state.template middleCols<1>(DIM_S::TRUTH::DOWN_IND) * this->gravity_magnitude * this->air_density) +
           measurement_noise;
  if constexpr(USE_BIAS)
  {
    output += truth_state.template middleCols<1>(DIM_S::TRUTH::ABS_PRESSURE_BIAS_IND);
  }

  return output;
}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,1,OPTIONS> AbsolutePressure<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
  estimateMeasurement(const SCALAR                                                            /* time */,
                      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& nav_state)
{
  Eigen::Matrix<SCALAR,1,1,OPTIONS> output;

  output = -nav_state.template middleCols<1>(DIM_S::NAV::DOWN_IND) * this->gravity_magnitude * this->air_density;
  if constexpr(USE_BIAS)
  {
    output += nav_state.template middleCols<1>(DIM_S::NAV::ABS_PRESSURE_BIAS_IND);
  }

  return output;
}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS> AbsolutePressure<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
  getMeasurementEstimatePDWRErrorState(const SCALAR                                                            /* time */,
                                       const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& /* nav_state */)
{
  Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS> output = Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS>::Zero();

  output[DIM_S::ERROR::DOWN_IND] = -this->gravity_magnitude * this->air_density;
  if constexpr(USE_BIAS)
  {
    output[DIM_S::ERROR::ABS_PRESSURE_BIAS_IND] = 1;
  }

  return output;
}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DISP_DIM,OPTIONS> AbsolutePressure<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
  getMeasurementPDWRDispersionState(const SCALAR                                                              /* time */,
                                    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& /* truth_state */)
{
  Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DISP_DIM,OPTIONS> output =
    Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DISP_DIM,OPTIONS>::Zero();

  output[DIM_S::TRUTH_DISP::DOWN_IND] = -this->gravity_magnitude * this->air_density;
  if constexpr(USE_BIAS)
  {
    output[DIM_S::TRUTH_DISP::ABS_PRESSURE_BIAS_IND] = 1;
  }

  return output;
}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool AbsolutePressure<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
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
inline SCALAR AbsolutePressure<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
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
/* absolute_pressure.hpp */
