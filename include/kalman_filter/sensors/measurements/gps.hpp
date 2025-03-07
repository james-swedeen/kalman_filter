/**
 * @File: gps.hpp
 * @Date: May 2022
 * @Author: James Swedeen
 *
 * @brief
 * A class for simulating gps measurements.
 **/

#ifndef KALMAN_FILTER_SENSORS_MEASUREMENTS_GPS_HPP
#define KALMAN_FILTER_SENSORS_MEASUREMENTS_GPS_HPP

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
class GPS;

template<typename DIM_S, bool ENABLED, bool USE_BIAS = true, typename SCALAR = double, Eigen::StorageOptions OPTIONS= Eigen::RowMajor>
using GPSPtr = std::shared_ptr<GPS<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>>;

/**
 * @makeGPS
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
 * A fully constructed GPS.
 **/
template<typename DIM_S, bool USE_BIAS = true, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
MeasurementBasePtr<3,DIM_S,SCALAR,OPTIONS> makeGPS(const rclcpp::Node::SharedPtr& node, const std::string& prefix);

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
class GPS
 : public MeasurementBase<3,DIM_S,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  GPS() = delete;
  /**
   * @Copy Constructor
   **/
  GPS(const GPS&) = default;
  /**
   * @Move Constructor
   **/
  GPS(GPS&&) = default;
  /**
   * @Constructor
   *
   * @brief
   * Initializes the object for use.
   *
   * @parameters
   * measurement_period: The time between this measurement's updates
   * gps_denied_func: Function that takes in the current time and truth state and returns true if GPS is denied
   * ss_kalman_gain: The steady state kalman gain, note this is only used if the USE_SS_KALMAN_GAIN flag is
   *                 enabled in the measurement controller
   **/
  GPS(const SCALAR                                                                                                      measurement_period,
      const std::function<bool(const SCALAR,const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>&)> gps_denied_func = GPS<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::defaultDeniedFunc,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,3,DIM_S::ERROR_DIM,OPTIONS>>&                                         ss_kalman_gain = Eigen::Matrix<SCALAR,3,DIM_S::ERROR_DIM,OPTIONS>::Constant(std::numeric_limits<SCALAR>::quiet_NaN()));
  /**
   * @Deconstructor
   **/
  ~GPS() override = default;
  /**
   * @Assignment Operators
   **/
  GPS& operator=(const GPS&)  = default;
  GPS& operator=(      GPS&&) = default;
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
  inline Eigen::Matrix<SCALAR,1,3,OPTIONS>
    getMeasurement(const SCALAR                                                              time,
                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& truth_state,
                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,               OPTIONS>>& measurement_noise) override;
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
  inline Eigen::Matrix<SCALAR,1,3,OPTIONS>
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
  inline Eigen::Matrix<SCALAR,3,DIM_S::ERROR_DIM,OPTIONS>
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
  inline Eigen::Matrix<SCALAR,3,DIM_S::TRUTH_DISP_DIM,OPTIONS>
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
   * @setGPSDeniedFunc
   *
   * @brief
   * Sets the GPS denied function.
   **/
  inline void setGPSDeniedFunc(
    const std::function<bool(const SCALAR,const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>&)>& gps_denied_func);
  /**
   * @IND
   *
   * @brief
   * Helper enumeration that defines what each element of the measurement vector is.
   **/
  enum class IND : Eigen::Index
  {
    NORTH = 0, // The north part of NED
    EAST  = 1, // The east part of NED
    DOWN  = 2  // The down part of NED
  };
  /**
   * @defaultDeniedFunc
   *
   * @brief
   * Default GPS denied function used if one isn't given. Always returns false, denoting no GPS denied areas.
   *
   * @parameters
   * time: The current simulation time
   * truth_state: The current truth state vector
   *
   * @return
   * True if GPS is denied, false otherwise.
   **/
  inline static bool defaultDeniedFunc(const SCALAR time, const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& truth_state) noexcept;
private:
  SCALAR                                                                                                      measurement_period;
  std::function<bool(const SCALAR,const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>&)> gps_denied_func;
};

template<typename DIM_S, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
MeasurementBasePtr<3,DIM_S,SCALAR,OPTIONS> makeGPS(const rclcpp::Node::SharedPtr& node, const std::string& prefix)
{
  node->declare_parameter(prefix + ".enabled",            rclcpp::PARAMETER_BOOL);
  node->declare_parameter(prefix + ".measurement_period", rclcpp::PARAMETER_DOUBLE);

  const bool   enabled            = node->get_parameter(prefix + ".enabled").as_bool();
  const SCALAR measurement_period = node->get_parameter(prefix + ".measurement_period").as_double();

  if(enabled)
  {
    return std::make_shared<GPS<DIM_S,true,USE_BIAS,SCALAR,OPTIONS>>(measurement_period);
  }
  else // Not enabled
  {
    return std::make_shared<GPS<DIM_S,false,USE_BIAS,SCALAR,OPTIONS>>(measurement_period);
  }
}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
GPS<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
  GPS(const SCALAR                                                                                                      measurement_period,
      const std::function<bool(const SCALAR,const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>&)> gps_denied_func,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,3,DIM_S::ERROR_DIM,OPTIONS>>&                                         ss_kalman_gain)
 : MeasurementBase<3,DIM_S,SCALAR,OPTIONS>(ss_kalman_gain),
   measurement_period(measurement_period),
   gps_denied_func(gps_denied_func)
{}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,3,OPTIONS> GPS<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
  getMeasurement(const SCALAR                                                              /* time */,
                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& truth_state,
                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,               OPTIONS>>& measurement_noise)
{
  Eigen::Matrix<SCALAR,1,3,OPTIONS> output;

  output = truth_state.template middleCols<3>(DIM_S::TRUTH::POS_START_IND) +
           measurement_noise;
  if constexpr(USE_BIAS)
  {
    output += truth_state.template middleCols<3>(DIM_S::TRUTH::GPS_POS_BIAS_START_IND);
  }

  return output;
}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,3,OPTIONS> GPS<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
  estimateMeasurement(const SCALAR                                                            /* time */,
                      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& nav_state)
{
  Eigen::Matrix<SCALAR,1,3,OPTIONS> output;

  output = nav_state.template middleCols<3>(DIM_S::NAV::POS_START_IND);
  if constexpr(USE_BIAS)
  {
    output += nav_state.template middleCols<3>(DIM_S::NAV::GPS_POS_BIAS_START_IND);
  }

  return output;
}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,3,DIM_S::ERROR_DIM,OPTIONS> GPS<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
  getMeasurementEstimatePDWRErrorState(const SCALAR                                                            /* time */,
                                       const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& /* nav_state */)
{
  Eigen::Matrix<SCALAR,3,DIM_S::ERROR_DIM,OPTIONS> output = Eigen::Matrix<SCALAR,3,DIM_S::ERROR_DIM,OPTIONS>::Zero();

  output.template middleCols<3>(DIM_S::ERROR::POS_START_IND).setIdentity();
  if constexpr(USE_BIAS)
  {
    output.template middleCols<3>(DIM_S::ERROR::GPS_POS_BIAS_START_IND).setIdentity();
  }

  return output;
}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,3,DIM_S::TRUTH_DISP_DIM,OPTIONS> GPS<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
  getMeasurementPDWRDispersionState(const SCALAR                                                              /* time */,
                                    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& /* truth_state */)
{
  Eigen::Matrix<SCALAR,3,DIM_S::TRUTH_DISP_DIM,OPTIONS> output = Eigen::Matrix<SCALAR,3,DIM_S::TRUTH_DISP_DIM,OPTIONS>::Zero();

  output.template middleCols<3>(DIM_S::TRUTH_DISP::POS_START_IND).setIdentity();
  if constexpr(USE_BIAS)
  {
    output.template middleCols<3>(DIM_S::TRUTH_DISP::GPS_POS_BIAS_START_IND).setIdentity();
  }

  return output;
}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool GPS<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
  measurementReady(const SCALAR                                                              time,
                   const SCALAR                                                              next_measurement_time,
                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& truth_state)
{
  if constexpr(ENABLED)
  {
    return (time >= next_measurement_time) and (not this->gps_denied_func(time, truth_state));
  }
  return false;
}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR GPS<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
  updateNextMeasurementTime(const SCALAR time, const SCALAR next_measurement_time)
{
  if(time >= next_measurement_time)
  {
    return next_measurement_time + this->measurement_period;
  }
  return next_measurement_time;
}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void GPS<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::setGPSDeniedFunc(
  const std::function<bool(const SCALAR,const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>&)>& gps_denied_func)
{
  this->gps_denied_func = gps_denied_func;
}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool GPS<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
  defaultDeniedFunc(const SCALAR /* time */, const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& /* truth_state */) noexcept
{
  return false;
}
} // namespace sensors
} // namespace kf

#endif
/* gps.hpp */
