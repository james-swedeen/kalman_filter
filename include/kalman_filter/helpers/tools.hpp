/**
 * @File: tools.hpp
 * @Date: May 2022
 * @Author: James Swedeen
 *
 * @brief
 * A helper class that hold all of the tools needed to run a Monte Carlo simulation.
 **/

#ifndef KALMAN_FILTER_HELPERS_TOOLS_HPP
#define KALMAN_FILTER_HELPERS_TOOLS_HPP

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<kalman_filter/dynamics/dynamics_base.hpp>
#include<kalman_filter/sensors/measurements/controllers/measurement_controller_base.hpp>
#include<kalman_filter/sensors/inertial_measurements/inertial_measurement_base.hpp>
#include<kalman_filter/controllers/controller_base.hpp>
#include<kalman_filter/noise/noise_base.hpp>

namespace kf
{
/**
 * @DIM_S
 * The type of a Dimensions object or an inheriting object that has information about the size of the state vectors.
 *
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS:
 * Eigen Matrix options.
 **/
template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
struct Tools
{
public:
  /**
   * @Default Constructor
   **/
  Tools() noexcept = default;
  /**
   * @Copy Constructor
   **/
  Tools(const Tools&) noexcept = default;
  /**
   * @Move Constructor
   **/
  Tools(Tools&&) noexcept = default;
  /**
   * @Deconstructor
   **/
  ~Tools() noexcept = default;
  /**
   * @Copy Assignment Operator
   **/
  Tools& operator=(const Tools&) noexcept = default;
  /**
   * @Move Assignment Operator
   **/
  Tools& operator=(Tools&&) noexcept = default;

  // Defines the truth and navigation state dynamics
  dynamics::DynamicsBasePtr<DIM_S,SCALAR,OPTIONS> dynamics;
  // Used to make process noise for the truth state dynamics
  noise::NoiseBasePtr<DIM_S::TRUTH_NOISE_DIM,SCALAR,OPTIONS> truth_process_noise;
  // A helper class that produces inertial measurement readings like the output of an IMU
  sensors::InertialMeasurementBasePtr<DIM_S,SCALAR,OPTIONS> inertial_measurements;
  // Used to make noise for the inertial measurements
  noise::NoiseBasePtr<DIM_S::INER_MEAS_NOISE_DIM,SCALAR,OPTIONS> inertial_measurements_noise;
  // A helper that is used to apply discreet measurements
  sensors::MeasurementControllerBasePtr<DIM_S,SCALAR,OPTIONS> measurement_controller;
  // A helper object that maps one state vector to another
  map::MappingsBasePtr<DIM_S,SCALAR,OPTIONS> mappings;
  // A helper that is used to calculate control inputs
  control::ControllerBasePtr<DIM_S,SCALAR,OPTIONS> controller;

  /**
   * @State Bounding Variables
   *
   * Used to bound the navigation state variables as the kalman filter runs.
   * Note that these variables are only used if the APPLY_STATE_BOUNDS flag is set.
   **/
  // The index of each state that should be bounded
  Eigen::Matrix<Eigen::Index,1,Eigen::Dynamic,OPTIONS,1,DIM_S::NAV_DIM> bounded_indexes;
  // The upper and lower bounds of the states in bounded_indexes, in the same order as they are in bounded_indexes
  Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS,1,DIM_S::NAV_DIM> upper_bound;
  Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS,1,DIM_S::NAV_DIM> lower_bound;

  /**
   * @Steady State Error Covariance
   *
   * The steady state error covariance used when the STEADY_STATE_ERROR_COV_T template parameter of the Dimensions
   * struct is set to true, Note that this parameter is only used in that case.
   * Should set it to be an underestimate of what is possible.
   **/
  Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS> ss_error_cov;
};
} // namespace kf

#endif
/* tools.hpp */
