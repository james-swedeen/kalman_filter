/**
 * @File: ground_velocity.hpp
 * @Date: September 2022
 * @Author: James Swedeen
 *
 * @brief
 * A class for simulating ground velocity measurements.
 **/

#ifndef KALMAN_FILTER_SENSORS_MEASUREMENTS_GROUND_VELOCITY_HPP
#define KALMAN_FILTER_SENSORS_MEASUREMENTS_GROUND_VELOCITY_HPP

/* C++ Headers */
#include<cstdint>
#include<memory>
#include<type_traits>

/* Eigen Headers */
#include<Eigen/Dense>

/* ROS Headers */
#include<rclcpp/rclcpp.hpp>

/* Local Headers */
#include<kalman_filter/sensors/measurements/measurement_base.hpp>
#include<kalman_filter/noise/noise_base.hpp>
#include<kalman_filter/noise/normal_distribution.hpp>
#include<kalman_filter/dynamics/dubins_airplane_model.hpp>

namespace kf
{
namespace sensors
{
template<typename DIM_S, bool ENABLED, typename SCALAR, Eigen::StorageOptions OPTIONS>
class GroundVelocity;

template<typename DIM_S, bool ENABLED, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using GroundVelocityPtr = std::shared_ptr<GroundVelocity<DIM_S,ENABLED,SCALAR,OPTIONS>>;

/**
 * @makeGroundVelocity
 *
 * @brief
 * Helper function that uses information from the ROS parameter server to construct the given object.
 *
 * @templates
 * DIM_S: The type of a Dimensions object or an inheriting object that has information about the size of the state vectors
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 *
 * @parameters
 * node: The node with the right namespacing to have access to the parameters needed
 * prefix: The prefix of the parameter names
 *
 * @return
 * A fully constructed GroundVelocity.
 **/
template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
MeasurementBasePtr<1,DIM_S,SCALAR,OPTIONS> makeGroundVelocity(const rclcpp::Node::SharedPtr& node, const std::string& prefix);

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
 * @OPTIONS
 * Eigen Matrix options.
 **/
template<typename DIM_S, bool ENABLED, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class GroundVelocity
 : public MeasurementBase<1,DIM_S,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  GroundVelocity() = delete;
  /**
   * @Copy Constructor
   **/
  GroundVelocity(const GroundVelocity&) = default;
  /**
   * @Move Constructor
   **/
  GroundVelocity(GroundVelocity&&) = default;
  /**
   * @Constructor
   *
   * @brief
   * Initializes the object for use.
   *
   * @parameters
   * measurement_period: The time between this measurement's updates
   **/
  GroundVelocity(const SCALAR measurement_period);
  /**
   * @Deconstructor
   **/
  ~GroundVelocity() override = default;
  /**
   * @Assignment Operators
   **/
  GroundVelocity& operator=(const GroundVelocity&)  = default;
  GroundVelocity& operator=(      GroundVelocity&&) = default;
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
};

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
MeasurementBasePtr<1,DIM_S,SCALAR,OPTIONS> makeGroundVelocity(const rclcpp::Node::SharedPtr& node, const std::string& prefix)
{
  node->declare_parameter(prefix + ".enabled",            rclcpp::PARAMETER_BOOL);
  node->declare_parameter(prefix + ".measurement_period", rclcpp::PARAMETER_DOUBLE);

  const bool   enabled            = node->get_parameter(prefix + ".enabled").as_bool();
  const SCALAR measurement_period = node->get_parameter(prefix + ".measurement_period").as_double();

  if(enabled)
  {
    return std::make_shared<GroundVelocity<DIM_S,true,SCALAR,OPTIONS>>(measurement_period);
  }
  else // Not enabled
  {
    return std::make_shared<GroundVelocity<DIM_S,false,SCALAR,OPTIONS>>(measurement_period);
  }
}

template<typename DIM_S, bool ENABLED, typename SCALAR, Eigen::StorageOptions OPTIONS>
GroundVelocity<DIM_S,ENABLED,SCALAR,OPTIONS>::
  GroundVelocity(const SCALAR measurement_period)
 : MeasurementBase<1,DIM_S,SCALAR,OPTIONS>(Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS>::Constant(std::numeric_limits<SCALAR>::quiet_NaN())),
   measurement_period(measurement_period)
{}

template<typename DIM_S, bool ENABLED, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,1,OPTIONS> GroundVelocity<DIM_S,ENABLED,SCALAR,OPTIONS>::
  getMeasurement(const SCALAR                                                              /* time */,
                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& truth_state,
                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,1,               OPTIONS>>& measurement_noise)
{
  if constexpr(std::is_same<DIM_S,dynamics::DubinsAirplaneDimBase<DIM_S::USE_STEADY_STATE_ERROR_COV,true>>:: value or
               std::is_same<DIM_S,dynamics::DubinsAirplaneDimBase<DIM_S::USE_STEADY_STATE_ERROR_COV,false>>::value)
  {
    return (truth_state[DIM_S::TRUTH::AIR_SPEED_IND] * std::cos(truth_state[DIM_S::TRUTH::PITCH_IND])) + measurement_noise;
  }
  else
  {
    return Eigen::Matrix<SCALAR,1,1,OPTIONS>(truth_state.template middleCols<2>(DIM_S::TRUTH::NORTH_VEL_IND).norm()) +
           measurement_noise;
  }
}

template<typename DIM_S, bool ENABLED, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,1,OPTIONS> GroundVelocity<DIM_S,ENABLED,SCALAR,OPTIONS>::
  estimateMeasurement(const SCALAR                                                            /* time */,
                      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& nav_state)
{
  return Eigen::Matrix<SCALAR,1,1,OPTIONS>(nav_state.template middleCols<2>(DIM_S::NAV::NORTH_VEL_IND).norm());
}

template<typename DIM_S, bool ENABLED, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS> GroundVelocity<DIM_S,ENABLED,SCALAR,OPTIONS>::
  getMeasurementEstimatePDWRErrorState(const SCALAR                                                            /* time */,
                                       const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& nav_state)
{
  Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS> output = Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS>::Zero();

  output.template middleCols<2>(DIM_S::ERROR::NORTH_VEL_IND) =
    nav_state.template middleCols<2>(DIM_S::NAV::NORTH_VEL_IND).array() / nav_state.template middleCols<2>(DIM_S::NAV::NORTH_VEL_IND).norm();

  return output;
}

template<typename DIM_S, bool ENABLED, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DISP_DIM,OPTIONS> GroundVelocity<DIM_S,ENABLED,SCALAR,OPTIONS>::
  getMeasurementPDWRDispersionState(const SCALAR                                                              /* time */,
                                    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& truth_state)
{
  Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DISP_DIM,OPTIONS> output = Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DISP_DIM,OPTIONS>::Zero();

  if constexpr(std::is_same<DIM_S,dynamics::DubinsAirplaneDimBase<DIM_S::USE_STEADY_STATE_ERROR_COV,true>>:: value or
               std::is_same<DIM_S,dynamics::DubinsAirplaneDimBase<DIM_S::USE_STEADY_STATE_ERROR_COV,false>>::value)
  {
    output[DIM_S::TRUTH_DISP::AIR_SPEED_IND] = std::cos(truth_state[DIM_S::TRUTH::PITCH_IND]);
    output[DIM_S::TRUTH_DISP::PITCH_IND]     = -truth_state[DIM_S::TRUTH::AIR_SPEED_IND] * std::sin(truth_state[DIM_S::TRUTH::PITCH_IND]);
  }
  else
  {
    output.template middleCols<2>(DIM_S::TRUTH_DISP::NORTH_VEL_IND) =
      truth_state.template middleCols<2>(DIM_S::TRUTH::NORTH_VEL_IND).array() / truth_state.template middleCols<2>(DIM_S::TRUTH::NORTH_VEL_IND).norm();
  }

  return output;
}

template<typename DIM_S, bool ENABLED, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool GroundVelocity<DIM_S,ENABLED,SCALAR,OPTIONS>::
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

template<typename DIM_S, bool ENABLED, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR GroundVelocity<DIM_S,ENABLED,SCALAR,OPTIONS>::
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
/* ground_velocity.hpp */
