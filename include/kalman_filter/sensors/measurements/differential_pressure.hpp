/**
 * @File: differential_pressure.hpp
 * @Date: July 2023
 * @Author: James Swedeen
 *
 * @brief
 * A class for simulating differential pressure measurement which give you information about air speed.
 **/

#ifndef KALMAN_FILTER_SENSORS_MEASUREMENTS_DIFFERENTIAL_PRESSURE_HPP
#define KALMAN_FILTER_SENSORS_MEASUREMENTS_DIFFERENTIAL_PRESSURE_HPP

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
#include<kalman_filter/math/quaternion.hpp>

namespace kf
{
namespace sensors
{
template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
class DifferentialPressure;

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using DifferentialPressurePtr = std::shared_ptr<DifferentialPressure<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>>;

/**
 * @makeDifferentialPressure
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
 * A fully constructed DifferentialPressure.
 **/
template<typename DIM_S, bool USE_BIAS = true, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
MeasurementBasePtr<1,DIM_S,SCALAR,OPTIONS> makeDifferentialPressure(const rclcpp::Node::SharedPtr& node, const std::string& prefix);

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
class DifferentialPressure
 : public MeasurementBase<1,DIM_S,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  DifferentialPressure() = delete;
  /**
   * @Copy Constructor
   **/
  DifferentialPressure(const DifferentialPressure&) noexcept = default;
  /**
   * @Move Constructor
   **/
  DifferentialPressure(DifferentialPressure&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * Initializes the object for use.
   *
   * @parameters
   * measurement_period: The time between this measurement's updates
   * air_density: The density of the air at the flight location
   * ss_kalman_gain: The steady state kalman gain, note this is only used if the USE_SS_KALMAN_GAIN flag is
   *                 enabled in the measurement controller
   **/
  DifferentialPressure(const SCALAR                                                              measurement_period,
                       const SCALAR                                                              air_density,
                       const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS>>& ss_kalman_gain = Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS>::Constant(std::numeric_limits<SCALAR>::quiet_NaN()));
  /**
   * @Deconstructor
   **/
  ~DifferentialPressure() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  DifferentialPressure& operator=(const DifferentialPressure&)  noexcept = default;
  DifferentialPressure& operator=(      DifferentialPressure&&) noexcept = default;
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
  SCALAR air_density;
  SCALAR one_half_air_density;
};

template<typename DIM_S, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
MeasurementBasePtr<1,DIM_S,SCALAR,OPTIONS>
  makeDifferentialPressure(const rclcpp::Node::SharedPtr& node, const std::string& prefix)
{
  node->declare_parameter(prefix + ".enabled",            rclcpp::PARAMETER_BOOL);
  node->declare_parameter(prefix + ".measurement_period", rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".air_density",        rclcpp::PARAMETER_DOUBLE);

  const bool   enabled            = node->get_parameter(prefix + ".enabled").as_bool();
  const SCALAR measurement_period = node->get_parameter(prefix + ".measurement_period").as_double();
  const SCALAR air_density        = node->get_parameter(prefix + ".air_density").as_double();

  if(enabled)
  {
    return std::make_shared<DifferentialPressure<DIM_S,true,USE_BIAS,SCALAR,OPTIONS>>(
             measurement_period,
             air_density);
  }
  else // Not enabled
  {
    return std::make_shared<DifferentialPressure<DIM_S,false,USE_BIAS,SCALAR,OPTIONS>>(
             measurement_period,
             air_density);
  }
}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
DifferentialPressure<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
  DifferentialPressure(const SCALAR                                                              measurement_period,
                       const SCALAR                                                              air_density,
                       const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS>>& ss_kalman_gain)
 : MeasurementBase<1,DIM_S,SCALAR,OPTIONS>(ss_kalman_gain),
   measurement_period(measurement_period),
   air_density(air_density),
   one_half_air_density(air_density/SCALAR(2))
{}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,1,OPTIONS> DifferentialPressure<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
  getMeasurement(const SCALAR                                                              /* time */,
                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& truth_state,
                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,1,               OPTIONS>>& measurement_noise)
{
  Eigen::Matrix<SCALAR,1,1,OPTIONS> output;

  //output[0] = (std::pow(truth_state[DIM_S::TRUTH::AIR_SPEED_IND], SCALAR(2)) * this->one_half_air_density) +
  output[0] = (truth_state[DIM_S::TRUTH::AIR_SPEED_IND] * this->one_half_air_density) +
              measurement_noise[0];
  if constexpr(USE_BIAS)
  {
    output += truth_state.template middleCols<1>(DIM_S::TRUTH::DIFF_PRESSURE_BIAS_IND);
  }

  return output;
}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,1,OPTIONS> DifferentialPressure<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
  estimateMeasurement(const SCALAR                                                            /* time */,
                      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& nav_state)
{
  Eigen::Matrix<SCALAR,1,1,OPTIONS> output;

  const Eigen::Matrix<SCALAR,3,3,OPTIONS> ned_to_body_rotation = math::quat::quaternionToDirectionCosineMatrix(nav_state.template middleCols<4>(DIM_S::NAV::QUAT_START_IND)).transpose();
  const Eigen::Matrix<SCALAR,1,3,OPTIONS> vel_body             = ned_to_body_rotation * nav_state.template middleCols<3>(DIM_S::NAV::VEL_START_IND).transpose();

  //output[0] = nav_state.template middleCols<3>(DIM_S::NAV::VEL_START_IND).squaredNorm() * this->one_half_air_density;
  output[0] = vel_body[0] * this->one_half_air_density;
  if constexpr(USE_BIAS)
  {
    output += nav_state.template middleCols<1>(DIM_S::NAV::DIFF_PRESSURE_BIAS_IND);
  }

  return output;
}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS> DifferentialPressure<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
  getMeasurementEstimatePDWRErrorState(const SCALAR                                                            /* time */,
                                       const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& nav_state)
{
  Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS> output = Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS>::Zero();

  const Eigen::Matrix<SCALAR,3,3,OPTIONS> ned_to_body_rotation = math::quat::quaternionToDirectionCosineMatrix(nav_state.template middleCols<4>(DIM_S::NAV::QUAT_START_IND)).transpose();
  const Eigen::Matrix<SCALAR,1,3,OPTIONS> rpy                  = math::quat::quaternionToRollPitchYaw(nav_state.template middleCols<4>(DIM_S::NAV::QUAT_START_IND));

  const SCALAR c_pitch = std::cos(rpy[1]);
  const SCALAR s_pitch = std::sin(rpy[1]);
  const SCALAR c_yaw   = std::cos(rpy[2]);
  const SCALAR s_yaw   = std::sin(rpy[2]);

  //output.template middleCols<3>(DIM_S::ERROR::VEL_START_IND) = this->air_density * nav_state.template middleCols<3>(DIM_S::NAV::VEL_START_IND);
  output.template middleCols<3>(DIM_S::ERROR::VEL_START_IND) = this->one_half_air_density * ned_to_body_rotation.template topRows<1>();
  output[DIM_S::ERROR::PITCH_IND] = ((-nav_state[DIM_S::NAV::NORTH_VEL_IND] * c_yaw * s_pitch) - (nav_state[DIM_S::NAV::EAST_VEL_IND] * s_yaw * s_pitch) - (nav_state[DIM_S::NAV::DOWN_VEL_IND] * c_pitch))* this->one_half_air_density;
  output[DIM_S::ERROR::YAW_IND]   = ((-nav_state[DIM_S::NAV::NORTH_VEL_IND] * s_yaw * c_pitch) + (nav_state[DIM_S::NAV::EAST_VEL_IND] * c_yaw * c_pitch))* this->one_half_air_density;

  if constexpr(USE_BIAS)
  {
    output[DIM_S::ERROR::DIFF_PRESSURE_BIAS_IND] = 1;
  }

  return output;
}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DISP_DIM,OPTIONS> DifferentialPressure<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
  getMeasurementPDWRDispersionState(const SCALAR                                                              /* time */,
                                    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& truth_state)
{
  Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DISP_DIM,OPTIONS> output =
    Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DISP_DIM,OPTIONS>::Zero();

  //output[DIM_S::TRUTH_DISP::AIR_SPEED_IND] = this->air_density * truth_state[DIM_S::TRUTH::AIR_SPEED_IND];
  output[DIM_S::TRUTH_DISP::AIR_SPEED_IND] = this->one_half_air_density;
  if constexpr(USE_BIAS)
  {
    output[DIM_S::TRUTH_DISP::DIFF_PRESSURE_BIAS_IND] = 1;
  }

  return output;
}

template<typename DIM_S, bool ENABLED, bool USE_BIAS, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool DifferentialPressure<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
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
inline SCALAR DifferentialPressure<DIM_S,ENABLED,USE_BIAS,SCALAR,OPTIONS>::
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
/* differential_pressure.hpp */
