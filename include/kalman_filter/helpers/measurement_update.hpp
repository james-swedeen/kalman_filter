/**
 * @File: measurement_update.hpp
 * @Date: April 2022
 * @Author: James Swedeen
 *
 * @brief
 * Used to apply updates to the navigation state and covariance.
 **/

#ifndef KALMAN_FILTER_HELPERS_MEASUREMENT_UPDATE_HPP
#define KALMAN_FILTER_HELPERS_MEASUREMENT_UPDATE_HPP

/* C++ Headers */
#include<cstdint>

#include<iostream>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<kalman_filter/sensors/measurements/measurement_base.hpp>
#include<kalman_filter/noise/noise_base.hpp>
#include<kalman_filter/mappings/mappings_base.hpp>

#include<kalman_filter/math/helpers.hpp>

namespace kf
{
/**
 * @applyMeasurement
 *
 * @brief
 * Used to update the navigation state and the error state covariance with a new measurement reading.
 *
 * @templates
 * MEAS_DIM: The size of the actual measurement vector
 * IS_EULER: Set to true if the measurement is of angles so the residual can be calculated correctly
 * DIM_S: The type of a Dimensions object or an inheriting object that has information
 *        about the size of the state vectors
 * USE_SS_KALMAN_GAIN: If true the steady state kalman gains held in the measurement objects will be used
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 * NAV_DERIVED: The matrix type of the navigation state
 * ERROR_COV_DERIVED: The matrix type of the error state covariance
 *
 * @parameters
 * measurement: The measurement object that will be used to update the estimates
 * mappings: A helper object that maps one state vector to another
 * time: The current simulation time
 * true_measurement: The real measurement from the sensors
 * measurement_noise_covariance: The covariance of the measurement noise
 * nav_state: The navigation state vector before the measurement is applied
 * error_covariance: The current a priori covariance matrix of the error state vector
 **/
template<Eigen::Index          MEAS_DIM,
         bool                  IS_EULER,
         typename              DIM_S,
         bool                  USE_SS_KALMAN_GAIN,
         typename              SCALAR,
         Eigen::StorageOptions OPTIONS,
         typename              MEAS_DERIVED,
         typename              MEAS_COV_DERIVED,
         typename              NAV_DERIVED,
         typename              ERROR_COV_DERIVED>
inline void applyMeasurement(const sensors::MeasurementBasePtr<MEAS_DIM,DIM_S,SCALAR,OPTIONS>& measurement,
                             const map::MappingsBasePtr<                DIM_S,SCALAR,OPTIONS>& mappings,
                             const SCALAR                                                      time,
                             const Eigen::MatrixBase<MEAS_DERIVED>&                            true_measurement,
                             const Eigen::MatrixBase<MEAS_COV_DERIVED>&                        measurement_noise_covariance,
                                   Eigen::MatrixBase<NAV_DERIVED>&                             nav_state,
                                   Eigen::MatrixBase<ERROR_COV_DERIVED>&                       error_covariance);
/**
 * @applyMeasurementMonteCarlo
 *
 * @brief
 * Used to update the navigation state using the steady state error state covariance and a new measurement reading.
 *
 * @templates
 * MEAS_DIM: The size of the actual measurement vector
 * IS_EULER: Set to true if the measurement is of angles so the residual can be calculated correctly
 * DIM_S: The type of a Dimensions object or an inheriting object that has information
 *        about the size of the state vectors
 * USE_SS_KALMAN_GAIN: If true the steady state kalman gains held in the measurement objects will be used
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 * TRUTH_DERIVED: The matrix type of the truth state
 * NAV_DERIVED: The matrix type of the navigation state
 * ERROR_COV_DERIVED: The matrix type of the error state covariance
 *
 * @parameters
 * measurement: The measurement object that will be used to update the estimates
 * measurement_noise: The noise object that make additive noise for the measurement
 * mappings: A helper object that maps one state vector to another
 * time: The current simulation time
 * truth_state: The current truth state vector
 * nav_state: The navigation state vector before the measurement is applied
 * error_covariance: The current a priori covariance matrix of the error state vector
 **/
template<Eigen::Index          MEAS_DIM,
         bool                  IS_EULER,
         typename              DIM_S,
         bool                  USE_SS_KALMAN_GAIN,
         typename              SCALAR,
         Eigen::StorageOptions OPTIONS,
         typename              TRUTH_DERIVED,
         typename              NAV_DERIVED,
         typename              ERROR_COV_DERIVED>
inline void applyMeasurementMonteCarlo(const sensors::MeasurementBasePtr<MEAS_DIM,DIM_S,SCALAR,OPTIONS>& measurement,
                                       const noise::NoiseBasePtr<        MEAS_DIM,      SCALAR,OPTIONS>& measurement_noise,
                                       const map::MappingsBasePtr<                DIM_S,SCALAR,OPTIONS>& mappings,
                                       const SCALAR                                                      time,
                                       const Eigen::MatrixBase<TRUTH_DERIVED>&                           truth_state,
                                             Eigen::MatrixBase<NAV_DERIVED>&                             nav_state,
                                             Eigen::MatrixBase<ERROR_COV_DERIVED>&                       error_covariance);
/**
 * @applyMeasurementMonteCarloSS
 *
 * @brief
 * Used to update the navigation state and the error state covariance with a new measurement reading.
 *
 * @templates
 * MEAS_DIM: The size of the actual measurement vector
 * IS_EULER: Set to true if the measurement is of angles so the residual can be calculated correctly
 * DIM_S: The type of a Dimensions object or an inheriting object that has information
 *        about the size of the state vectors
 * USE_SS_KALMAN_GAIN: If true the steady state kalman gains held in the measurement objects will be used
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 * TRUTH_DERIVED: The matrix type of the truth state
 * NAV_DERIVED: The matrix type of the navigation state
 * ERROR_COV_DERIVED: The matrix type of the error state covariance
 *
 * @parameters
 * measurement: The measurement object that will be used to update the estimates
 * measurement_noise: The noise object that make additive noise for the measurement
 * mappings: A helper object that maps one state vector to another
 * time: The current simulation time
 * truth_state: The current truth state vector
 * ss_error_covariance: The steady state covariance matrix of the error state vector
 * nav_state: The navigation state vector before the measurement is applied
 **/
template<Eigen::Index          MEAS_DIM,
         bool                  IS_EULER,
         typename              DIM_S,
         bool                  USE_SS_KALMAN_GAIN,
         typename              SCALAR,
         Eigen::StorageOptions OPTIONS,
         typename              TRUTH_DERIVED,
         typename              NAV_DERIVED,
         typename              ERROR_COV_DERIVED>
inline void applyMeasurementMonteCarloSS(const sensors::MeasurementBasePtr<MEAS_DIM,DIM_S,SCALAR,OPTIONS>& measurement,
                                         const noise::NoiseBasePtr<        MEAS_DIM,      SCALAR,OPTIONS>& measurement_noise,
                                         const map::MappingsBasePtr<                DIM_S,SCALAR,OPTIONS>& mappings,
                                         const SCALAR                                                      time,
                                         const Eigen::MatrixBase<TRUTH_DERIVED>&                           truth_state,
                                         const Eigen::MatrixBase<ERROR_COV_DERIVED>&                       ss_error_covariance,
                                               Eigen::MatrixBase<NAV_DERIVED>&                             nav_state);
/**
 * @applyMeasurementLinCov
 *
 * @brief
 * Used to update the error state covariance and augmented state covariance with a new measurement reading.
 *
 * @templates
 * MEAS_DIM: The size of the actual measurement vector
 * DIM_S: The type of a Dimensions object or an inheriting object that has information
 *        about the size of the state vectors
 * USE_SS_KALMAN_GAIN: If true the steady state kalman gains held in the measurement objects will be used
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 * TRUTH_DERIVED: The matrix type of the truth state
 * NAV_DERIVED: The matrix type of the navigation state
 * ERROR_COV_DERIVED: The matrix type of the error state covariance
 * AUG_COV_DERIVED: The matrix type of the augmented state covariance
 *
 * @parameters
 * measurement: The measurement object that will be used to update the estimates
 * measurement_noise: The noise object that make additive noise for the measurement
 * time: The current simulation time
 * ref_truth_state: The current state from the reference trajectory mapped into a truth state vector
 * ref_nav_state: The current state from the reference trajectory mapped into a navigation state vector
 * error_covariance: The current a priori covariance matrix of the error state vector
 * aug_covariance: The current a priori covariance matrix of the augmented state vector
 **/
template<Eigen::Index          MEAS_DIM,
         typename              DIM_S,
         bool                  USE_SS_KALMAN_GAIN,
         typename              SCALAR,
         Eigen::StorageOptions OPTIONS,
         typename              TRUTH_DERIVED,
         typename              NAV_DERIVED,
         typename              ERROR_COV_DERIVED,
         typename              AUG_COV_DERIVED>
inline void applyMeasurementLinCov(const sensors::MeasurementBasePtr<MEAS_DIM,DIM_S,SCALAR,OPTIONS>& measurement,
                                   const noise::NoiseBasePtr<        MEAS_DIM,      SCALAR,OPTIONS>& measurement_noise,
                                   const SCALAR                                                      time,
                                   const Eigen::MatrixBase<TRUTH_DERIVED>&                           ref_truth_state,
                                   const Eigen::MatrixBase<NAV_DERIVED>&                             ref_nav_state,
                                         Eigen::MatrixBase<ERROR_COV_DERIVED>&                       error_covariance,
                                         Eigen::MatrixBase<AUG_COV_DERIVED>&                         aug_covariance);
/**
 * @applyMeasurementErrorBudget
 *
 * @brief
 * Used to update the augmented state covariance with a new measurement reading while performing error budget analysis.
 *
 * @templates
 * MEAS_DIM: The size of the actual measurement vector
 * DIM_S: The type of a Dimensions object or an inheriting object that has information
 *        about the size of the state vectors
 * USE_SS_KALMAN_GAIN: If true the steady state kalman gains held in the measurement objects will be used
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 * TRUTH_DERIVED: The matrix type of the truth state
 * NAV_DERIVED: The matrix type of the navigation state
 * ERROR_COV_DERIVED: The matrix type of the error state covariance
 * AUG_COV_DERIVED: The matrix type of the augmented state covariance
 *
 * @parameters
 * measurement: The measurement object that will be used to update the estimates
 * measurement_noise: The noise object that make additive noise for the measurement
 * time: The current simulation time
 * ref_truth_state: The current state from the reference trajectory mapped into a truth state vector
 * ref_nav_state: The current state from the reference trajectory mapped into a navigation state vector
 * error_covariance: The current a priori covariance matrix of the error state vector
 * aug_covariance: The current a priori covariance matrix of the augmented state vector
 **/
template<Eigen::Index          MEAS_DIM,
         typename              DIM_S,
         bool                  USE_SS_KALMAN_GAIN,
         typename              SCALAR,
         Eigen::StorageOptions OPTIONS,
         typename              TRUTH_DERIVED,
         typename              NAV_DERIVED,
         typename              ERROR_COV_DERIVED,
         typename              AUG_COV_DERIVED>
inline void applyMeasurementErrorBudget(const sensors::MeasurementBasePtr<MEAS_DIM,DIM_S,SCALAR,OPTIONS>& measurement,
                                        const noise::NoiseBasePtr<        MEAS_DIM,      SCALAR,OPTIONS>& measurement_noise,
                                        const SCALAR                                                      time,
                                        const Eigen::MatrixBase<TRUTH_DERIVED>&                           ref_truth_state,
                                        const Eigen::MatrixBase<NAV_DERIVED>&                             ref_nav_state,
                                        const Eigen::MatrixBase<ERROR_COV_DERIVED>&                       error_covariance,
                                              Eigen::MatrixBase<AUG_COV_DERIVED>&                         aug_covariance);
/**
 * @findKalmanGain
 *
 * @brief
 * Helper function used to compute the Kalman gain of a particular measurement.
 *
 * @templates
 * MEAS_DIM: The size of the actual measurement vector
 * DIM_S: The type of a Dimensions object or an inheriting object that has information
 *        about the size of the state vectors
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 * ERROR_COV_DERIVED: The matrix type of the error state covariance
 * NOISE_COV_DERIVED: The matrix type of the noise covariance
 * MEAS_MAT_DERIVED: The matrix type of the measurement matrix
 *
 * @parameters
 * pre_error_covariance: The current a priori covariance matrix of the error state vector
 * noise_covariance: The covariance matrix of the measurement noise
 * measurement_matrix_nav: The linearized version of the measurement estimate with respect to the error state
 *
 * @return
 * The Kalman gain for this measurement.
 **/
template<Eigen::Index          MEAS_DIM,
         typename              DIM_S,
         typename              SCALAR,
         Eigen::StorageOptions OPTIONS,
         typename              ERROR_COV_DERIVED,
         typename              NOISE_DERIVED,
         typename              MEAS_MAT_DERIVED>
inline Eigen::Matrix<SCALAR,MEAS_DIM,DIM_S::ERROR_DIM,OPTIONS>
  findKalmanGain(const Eigen::MatrixBase<ERROR_COV_DERIVED>& pre_error_covariance,
                 const Eigen::MatrixBase<NOISE_DERIVED>&     noise_covariance,
                 const Eigen::MatrixBase<MEAS_MAT_DERIVED>&  measurement_matrix_nav);
/**
 * @updateNavState
 *
 * @brief
 * Used to update the navigation state using new a new measurement.
 *
 * @templates
 * MEAS_DIM: The size of the actual measurement vector
 * IS_EULER: Set to true if the measurement is of angles so the residual can be calculated correctly
 * DIM_S: The type of a Dimensions object or an inheriting object that has information
 *        about the size of the state vectors
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 * NAV_DERIVED: The matrix type of the navigation state
 * KALMAN_DERIVED: The matrix type of the kalman gain
 * TRUE_MEAS_DERIVED: The matrix type of the true measurement
 * PRED_MEAS_DERIVED: The matrix type of the predicted measurement
 *
 * @parameters
 * mappings: A helper object that maps one state vector to another
 * pre_nav_state: The navigation state vector before the measurement is applied
 * kalman_gain: The Kalman gain for this measurement
 * true_measurement: The actual value of the measurement
 * predicted_measurement: The value of the measurement that was predicted using the navigation state
 *
 * @return
 * The updated navigation state vector.
 **/
template<Eigen::Index          MEAS_DIM,
         bool                  IS_EULER,
         typename              DIM_S,
         typename              SCALAR,
         Eigen::StorageOptions OPTIONS,
         typename              NAV_DERIVED,
         typename              KALMAN_DERIVED,
         typename              TRUE_MEAS_DERIVED,
         typename              PRED_MEAS_DERIVED>
inline Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>
  updateNavState(const map::MappingsBasePtr<DIM_S,SCALAR,OPTIONS>& mappings,
                 const Eigen::MatrixBase<NAV_DERIVED>&             pre_nav_state,
                 const Eigen::MatrixBase<KALMAN_DERIVED>&          kalman_gain,
                 const Eigen::MatrixBase<TRUE_MEAS_DERIVED>&       true_measurement,
                 const Eigen::MatrixBase<PRED_MEAS_DERIVED>&       predicted_measurement);
/**
 * @updateErrorCovariance
 *
 * @brief
 * Used to apply a measurement to the error state covariance.
 *
 * @templates
 * MEAS_DIM: The size of the actual measurement vector
 * DIM_S: The type of a Dimensions object or an inheriting object that has information
 *        about the size of the state vectors
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 * ERROR_COV_DERIVED: The matrix type of the error state covariance
 * NOISE_COV_DERIVED: The matrix type of the noise covariance
 * MEAS_MAT_DERIVED: The matrix type of the measurement matrix
 * KALMAN_DERIVED: The matrix type of the kalman gain
 *
 * @parameters
 * pre_error_covariance: The current a priori covariance matrix of the error state vector
 * noise_covariance: The covariance matrix of the measurement noise
 * measurement_matrix_nav: The linearized version of the measurement estimate with respect to the error state
 * kalman_gain: The Kalman gain for this measurement
 *
 * @return
 * The updated covariance of the error state.
 **/
template<Eigen::Index          MEAS_DIM,
         typename              DIM_S,
         typename              SCALAR,
         Eigen::StorageOptions OPTIONS,
         typename              ERROR_COV_DERIVED,
         typename              NOISE_COV_DERIVED,
         typename              MEAS_MAT_DERIVED,
         typename              KALMAN_DERIVED>
inline Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS>
  updateErrorCovariance(const Eigen::MatrixBase<ERROR_COV_DERIVED>& pre_error_covariance,
                        const Eigen::MatrixBase<NOISE_COV_DERIVED>& noise_covariance,
                        const Eigen::MatrixBase<MEAS_MAT_DERIVED>&  measurement_matrix_nav,
                        const Eigen::MatrixBase<KALMAN_DERIVED>&    kalman_gain);
/**
 * @updateAugCovariance
 *
 * @brief
 * Used to apply a measurement to the augmented state covariance.
 *
 * @templates
 * MEAS_DIM: The size of the actual measurement vector
 * DIM_S: The type of a Dimensions object or an inheriting object that has information
 *        about the size of the state vectors
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 * AUG_COV_DERIVED: The matrix type of the augmented state covariance
 * NOISE_COV_DERIVED: The matrix type of the noise covariance
 * MEAS_MAT_NAV_DERIVED: The matrix type of the measurement matrix
 * MEAS_MAT_TRUE_DERIVED: The matrix type of the measurement matrix
 * KALMAN_DERIVED: The matrix type of the kalman gain
 *
 * @parameters
 * pre_aug_covariance: The current a priori covariance matrix of the augmented state vector
 * noise_covariance: The covariance matrix of the measurement noise
 * measurement_matrix_nav: The linearized version of the measurement estimate with respect to the error state
 * measurement_matrix_truth: The linearized version of the measurement function with respect to the error state
 * kalman_gain: The Kalman gain for this measurement
 *
 * @return
 * The updated covariance of the augmented state.
 **/
template<Eigen::Index          MEAS_DIM,
         typename              DIM_S,
         typename              SCALAR,
         Eigen::StorageOptions OPTIONS,
         typename              AUG_COV_DERIVED,
         typename              NOISE_COV_DERIVED,
         typename              MEAS_MAT_NAV_DERIVED,
         typename              MEAS_MAT_TRUE_DERIVED,
         typename              KALMAN_DERIVED>
inline Eigen::Matrix<SCALAR,DIM_S::LINCOV::AUG_DIM,DIM_S::LINCOV::AUG_DIM,OPTIONS>
  updateAugCovariance(const Eigen::MatrixBase<AUG_COV_DERIVED>&       pre_aug_covariance,
                      const Eigen::MatrixBase<NOISE_COV_DERIVED>&     noise_covariance,
                      const Eigen::MatrixBase<MEAS_MAT_NAV_DERIVED>&  measurement_matrix_nav,
                      const Eigen::MatrixBase<MEAS_MAT_TRUE_DERIVED>& measurement_matrix_truth,
                      const Eigen::MatrixBase<KALMAN_DERIVED>&        kalman_gain);
} // namespace kf

template<Eigen::Index          MEAS_DIM,
         bool                  IS_EULER,
         typename              DIM_S,
         bool                  USE_SS_KALMAN_GAIN,
         typename              SCALAR,
         Eigen::StorageOptions OPTIONS,
         typename              MEAS_DERIVED,
         typename              MEAS_COV_DERIVED,
         typename              NAV_DERIVED,
         typename              ERROR_COV_DERIVED>
inline void kf::applyMeasurement(const sensors::MeasurementBasePtr<MEAS_DIM,DIM_S,SCALAR,OPTIONS>& measurement,
                                 const map::MappingsBasePtr<                DIM_S,SCALAR,OPTIONS>& mappings,
                                 const SCALAR                                                      time,
                                 const Eigen::MatrixBase<MEAS_DERIVED>&                            true_measurement,
                                 const Eigen::MatrixBase<MEAS_COV_DERIVED>&                        measurement_noise_covariance,
                                       Eigen::MatrixBase<NAV_DERIVED>&                             nav_state,
                                       Eigen::MatrixBase<ERROR_COV_DERIVED>&                       error_covariance)
{
  static_assert((int(MEAS_DERIVED::     RowsAtCompileTime) == 1)                or (int(MEAS_DERIVED::     RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(MEAS_DERIVED::     ColsAtCompileTime) == MEAS_DIM)         or (int(MEAS_DERIVED::     ColsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(MEAS_COV_DERIVED:: RowsAtCompileTime) == MEAS_DIM)         or (int(MEAS_COV_DERIVED:: RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(MEAS_COV_DERIVED:: ColsAtCompileTime) == MEAS_DIM)         or (int(MEAS_COV_DERIVED:: ColsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(NAV_DERIVED::      RowsAtCompileTime) == 1)                or (int(NAV_DERIVED::      RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(NAV_DERIVED::      ColsAtCompileTime) == DIM_S::NAV_DIM)   or (int(NAV_DERIVED::      ColsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(ERROR_COV_DERIVED::RowsAtCompileTime) == DIM_S::ERROR_DIM) or (int(ERROR_COV_DERIVED::RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(ERROR_COV_DERIVED::ColsAtCompileTime) == DIM_S::ERROR_DIM) or (int(ERROR_COV_DERIVED::ColsAtCompileTime) == Eigen::Dynamic));
  assert(true_measurement.rows()             == 1);
  assert(true_measurement.cols()             == MEAS_DIM);
  assert(measurement_noise_covariance.rows() == MEAS_DIM);
  assert(measurement_noise_covariance.cols() == MEAS_DIM);
  assert(nav_state.       rows()             == 1);
  assert(nav_state.       cols()             == DIM_S::NAV_DIM);
  assert(error_covariance.rows()             == DIM_S::ERROR_DIM);
  assert(error_covariance.cols()             == DIM_S::ERROR_DIM);

  const Eigen::Matrix<SCALAR,MEAS_DIM,DIM_S::ERROR_DIM,OPTIONS> measurement_matrix_nav = measurement->getMeasurementEstimatePDWRErrorState(time, nav_state);
  const Eigen::Matrix<SCALAR,1,       MEAS_DIM,        OPTIONS> predicted_measurement  = measurement->estimateMeasurement(time, nav_state);

  Eigen::Matrix<SCALAR,MEAS_DIM,DIM_S::ERROR_DIM,OPTIONS> kalman_gain;
  if constexpr(USE_SS_KALMAN_GAIN)
  {
    kalman_gain = measurement->getSteadyStateKalmanGain();
  }
  else // Don't use steady state kalman gain
  {
    kalman_gain = findKalmanGain<MEAS_DIM,DIM_S,SCALAR,OPTIONS>(error_covariance, measurement_noise_covariance, measurement_matrix_nav);
  }

  nav_state = updateNavState<MEAS_DIM,IS_EULER,DIM_S,SCALAR,OPTIONS>(mappings,
                                                                     nav_state,
                                                                     kalman_gain,
                                                                     true_measurement,
                                                                     predicted_measurement);
  error_covariance = updateErrorCovariance<MEAS_DIM,DIM_S,SCALAR,OPTIONS>(error_covariance,
                                                                          measurement_noise_covariance,
                                                                          measurement_matrix_nav,
                                                                          kalman_gain);
}

template<Eigen::Index          MEAS_DIM,
         bool                  IS_EULER,
         typename              DIM_S,
         bool                  USE_SS_KALMAN_GAIN,
         typename              SCALAR,
         Eigen::StorageOptions OPTIONS,
         typename              TRUTH_DERIVED,
         typename              NAV_DERIVED,
         typename              ERROR_COV_DERIVED>
inline void kf::applyMeasurementMonteCarlo(const sensors::MeasurementBasePtr<MEAS_DIM,DIM_S,SCALAR,OPTIONS>& measurement,
                                           const noise::NoiseBasePtr<        MEAS_DIM,      SCALAR,OPTIONS>& measurement_noise,
                                           const map::MappingsBasePtr<                DIM_S,SCALAR,OPTIONS>& mappings,
                                           const SCALAR                                                      time,
                                           const Eigen::MatrixBase<TRUTH_DERIVED>&                           truth_state,
                                                 Eigen::MatrixBase<NAV_DERIVED>&                             nav_state,
                                                 Eigen::MatrixBase<ERROR_COV_DERIVED>&                       error_covariance)
{
  static_assert((int(TRUTH_DERIVED::    RowsAtCompileTime) == 1)                or (int(TRUTH_DERIVED::     RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(TRUTH_DERIVED::    ColsAtCompileTime) == DIM_S::TRUTH_DIM) or (int(TRUTH_DERIVED::     ColsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(NAV_DERIVED::      RowsAtCompileTime) == 1)                or (int(NAV_DERIVED::      RowsAtCompileTime)  == Eigen::Dynamic));
  static_assert((int(NAV_DERIVED::      ColsAtCompileTime) == DIM_S::NAV_DIM)   or (int(NAV_DERIVED::      ColsAtCompileTime)  == Eigen::Dynamic));
  static_assert((int(ERROR_COV_DERIVED::RowsAtCompileTime) == DIM_S::ERROR_DIM) or (int(ERROR_COV_DERIVED::RowsAtCompileTime)  == Eigen::Dynamic));
  static_assert((int(ERROR_COV_DERIVED::ColsAtCompileTime) == DIM_S::ERROR_DIM) or (int(ERROR_COV_DERIVED::ColsAtCompileTime)  == Eigen::Dynamic));
  assert(truth_state.     rows() == 1);
  assert(truth_state.     cols() == DIM_S::TRUTH_DIM);
  assert(nav_state.       rows() == 1);
  assert(nav_state.       cols() == DIM_S::NAV_DIM);
  assert(error_covariance.rows() == DIM_S::ERROR_DIM);
  assert(error_covariance.cols() == DIM_S::ERROR_DIM);

  const Eigen::Matrix<SCALAR,MEAS_DIM,MEAS_DIM,        OPTIONS> noise_covariance       = measurement_noise->getCovariance();
  const Eigen::Matrix<SCALAR,MEAS_DIM,DIM_S::ERROR_DIM,OPTIONS> measurement_matrix_nav = measurement->getMeasurementEstimatePDWRErrorState(time, nav_state);
  const Eigen::Matrix<SCALAR,1,       MEAS_DIM,        OPTIONS> measurement_noise_vec  = measurement_noise->getNoise();
  const Eigen::Matrix<SCALAR,1,       MEAS_DIM,        OPTIONS> true_measurement       = measurement->getMeasurement(time, truth_state, measurement_noise_vec);
  const Eigen::Matrix<SCALAR,1,       MEAS_DIM,        OPTIONS> predicted_measurement  = measurement->estimateMeasurement(time, nav_state);

  Eigen::Matrix<SCALAR,MEAS_DIM,DIM_S::ERROR_DIM,OPTIONS> kalman_gain;
  if constexpr(USE_SS_KALMAN_GAIN)
  {
    kalman_gain = measurement->getSteadyStateKalmanGain();
  }
  else // Don't use steady state kalman gain
  {
    kalman_gain = findKalmanGain<MEAS_DIM,DIM_S,SCALAR,OPTIONS>(error_covariance, noise_covariance, measurement_matrix_nav);
  }

  nav_state = updateNavState<MEAS_DIM,IS_EULER,DIM_S,SCALAR,OPTIONS>(mappings,
                                                                     nav_state,
                                                                     kalman_gain,
                                                                     true_measurement,
                                                                     predicted_measurement);
  error_covariance = updateErrorCovariance<MEAS_DIM,DIM_S,SCALAR,OPTIONS>(error_covariance,
                                                                          noise_covariance,
                                                                          measurement_matrix_nav,
                                                                          kalman_gain);
}

template<Eigen::Index          MEAS_DIM,
         bool                  IS_EULER,
         typename              DIM_S,
         bool                  USE_SS_KALMAN_GAIN,
         typename              SCALAR,
         Eigen::StorageOptions OPTIONS,
         typename              TRUTH_DERIVED,
         typename              NAV_DERIVED,
         typename              ERROR_COV_DERIVED>
inline void kf::applyMeasurementMonteCarloSS(const sensors::MeasurementBasePtr<MEAS_DIM,DIM_S,SCALAR,OPTIONS>& measurement,
                                             const noise::NoiseBasePtr<        MEAS_DIM,      SCALAR,OPTIONS>& measurement_noise,
                                             const map::MappingsBasePtr<                DIM_S,SCALAR,OPTIONS>& mappings,
                                             const SCALAR                                                      time,
                                             const Eigen::MatrixBase<TRUTH_DERIVED>&                           truth_state,
                                             const Eigen::MatrixBase<ERROR_COV_DERIVED>&                       ss_error_covariance,
                                                   Eigen::MatrixBase<NAV_DERIVED>&                             nav_state)
{
  static_assert((int(TRUTH_DERIVED::    RowsAtCompileTime) == 1)                or (int(TRUTH_DERIVED::     RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(TRUTH_DERIVED::    ColsAtCompileTime) == DIM_S::TRUTH_DIM) or (int(TRUTH_DERIVED::     ColsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(ERROR_COV_DERIVED::RowsAtCompileTime) == DIM_S::ERROR_DIM) or (int(ERROR_COV_DERIVED::RowsAtCompileTime)  == Eigen::Dynamic));
  static_assert((int(ERROR_COV_DERIVED::ColsAtCompileTime) == DIM_S::ERROR_DIM) or (int(ERROR_COV_DERIVED::ColsAtCompileTime)  == Eigen::Dynamic));
  static_assert((int(NAV_DERIVED::      RowsAtCompileTime) == 1)                or (int(NAV_DERIVED::      RowsAtCompileTime)  == Eigen::Dynamic));
  static_assert((int(NAV_DERIVED::      ColsAtCompileTime) == DIM_S::NAV_DIM)   or (int(NAV_DERIVED::      ColsAtCompileTime)  == Eigen::Dynamic));
  assert(truth_state.     rows()    == 1);
  assert(truth_state.     cols()    == DIM_S::TRUTH_DIM);
  assert(ss_error_covariance.rows() == DIM_S::ERROR_DIM);
  assert(ss_error_covariance.cols() == DIM_S::ERROR_DIM);
  assert(nav_state.       rows()    == 1);
  assert(nav_state.       cols()    == DIM_S::NAV_DIM);

  const Eigen::Matrix<SCALAR,1,MEAS_DIM,OPTIONS> measurement_noise_vec  = measurement_noise->getNoise();
  const Eigen::Matrix<SCALAR,1,MEAS_DIM,OPTIONS> true_measurement       = measurement->getMeasurement(time, truth_state, measurement_noise_vec);
  const Eigen::Matrix<SCALAR,1,MEAS_DIM,OPTIONS> predicted_measurement  = measurement->estimateMeasurement(time, nav_state);

  Eigen::Matrix<SCALAR,MEAS_DIM,DIM_S::ERROR_DIM,OPTIONS> kalman_gain;
  if constexpr(USE_SS_KALMAN_GAIN)
  {
    kalman_gain = measurement->getSteadyStateKalmanGain();
  }
  else // Don't use steady state kalman gain
  {
    kalman_gain = findKalmanGain<MEAS_DIM,DIM_S,SCALAR,OPTIONS>(ss_error_covariance,
                                                                measurement_noise->getCovariance(),
                                                                measurement->getMeasurementEstimatePDWRErrorState(time, nav_state));
  }

  nav_state = updateNavState<MEAS_DIM,IS_EULER,DIM_S,SCALAR,OPTIONS>(mappings,
                                                                     nav_state,
                                                                     kalman_gain,
                                                                     true_measurement,
                                                                     predicted_measurement);
}

template<Eigen::Index          MEAS_DIM,
         typename              DIM_S,
         bool                  USE_SS_KALMAN_GAIN,
         typename              SCALAR,
         Eigen::StorageOptions OPTIONS,
         typename              TRUTH_DERIVED,
         typename              NAV_DERIVED,
         typename              ERROR_COV_DERIVED,
         typename              AUG_COV_DERIVED>
inline void kf::applyMeasurementLinCov(const sensors::MeasurementBasePtr<MEAS_DIM,DIM_S,SCALAR,OPTIONS>& measurement,
                                       const noise::NoiseBasePtr<        MEAS_DIM,      SCALAR,OPTIONS>& measurement_noise,
                                       const SCALAR                                                      time,
                                       const Eigen::MatrixBase<TRUTH_DERIVED>&                           ref_truth_state,
                                       const Eigen::MatrixBase<NAV_DERIVED>&                             ref_nav_state,
                                             Eigen::MatrixBase<ERROR_COV_DERIVED>&                       error_covariance,
                                             Eigen::MatrixBase<AUG_COV_DERIVED>&                         aug_covariance)
{
  static_assert((int(TRUTH_DERIVED::    RowsAtCompileTime) == 1)                      or (int(TRUTH_DERIVED::     RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(TRUTH_DERIVED::    ColsAtCompileTime) == DIM_S::TRUTH_DIM)       or (int(TRUTH_DERIVED::     ColsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(NAV_DERIVED::      RowsAtCompileTime) == 1)                      or (int(NAV_DERIVED::      RowsAtCompileTime)  == Eigen::Dynamic));
  static_assert((int(NAV_DERIVED::      ColsAtCompileTime) == DIM_S::NAV_DIM)         or (int(NAV_DERIVED::      ColsAtCompileTime)  == Eigen::Dynamic));
  static_assert((int(ERROR_COV_DERIVED::RowsAtCompileTime) == DIM_S::ERROR_DIM)       or (int(ERROR_COV_DERIVED::RowsAtCompileTime)  == Eigen::Dynamic));
  static_assert((int(ERROR_COV_DERIVED::ColsAtCompileTime) == DIM_S::ERROR_DIM)       or (int(ERROR_COV_DERIVED::ColsAtCompileTime)  == Eigen::Dynamic));
  static_assert((int(AUG_COV_DERIVED::RowsAtCompileTime)   == DIM_S::LINCOV::AUG_DIM) or (int(AUG_COV_DERIVED::RowsAtCompileTime)    == Eigen::Dynamic));
  static_assert((int(AUG_COV_DERIVED::ColsAtCompileTime)   == DIM_S::LINCOV::AUG_DIM) or (int(AUG_COV_DERIVED::ColsAtCompileTime)    == Eigen::Dynamic));
  assert(ref_truth_state. rows() == 1);
  assert(ref_truth_state. cols() == DIM_S::TRUTH_DIM);
  assert(ref_nav_state.   rows() == 1);
  assert(ref_nav_state.   cols() == DIM_S::NAV_DIM);
  assert(error_covariance.rows() == DIM_S::ERROR_DIM);
  assert(error_covariance.cols() == DIM_S::ERROR_DIM);
  assert(aug_covariance.  rows() == DIM_S::LINCOV::AUG_DIM);
  assert(aug_covariance.  cols() == DIM_S::LINCOV::AUG_DIM);

  const Eigen::Matrix<SCALAR,MEAS_DIM,MEAS_DIM,             OPTIONS> noise_covariance         = measurement_noise->getCovariance();
  const Eigen::Matrix<SCALAR,MEAS_DIM,DIM_S::ERROR_DIM,     OPTIONS> measurement_matrix_nav   = measurement->getMeasurementEstimatePDWRErrorState(time, ref_nav_state);
  const Eigen::Matrix<SCALAR,MEAS_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS> measurement_matrix_truth = measurement->getMeasurementPDWRDispersionState(time, ref_truth_state);

  Eigen::Matrix<SCALAR,MEAS_DIM,DIM_S::ERROR_DIM,OPTIONS> kalman_gain;
  if constexpr(USE_SS_KALMAN_GAIN)
  {
    kalman_gain = measurement->getSteadyStateKalmanGain();
  }
  else // Don't use steady state kalman gain
  {
    kalman_gain = findKalmanGain<MEAS_DIM,DIM_S,SCALAR,OPTIONS>(error_covariance, noise_covariance, measurement_matrix_nav);
  }

  error_covariance = updateErrorCovariance<MEAS_DIM,DIM_S,SCALAR,OPTIONS>(error_covariance,
                                                                          noise_covariance,
                                                                          measurement_matrix_nav,
                                                                          kalman_gain);
  aug_covariance = updateAugCovariance<MEAS_DIM,DIM_S,SCALAR,OPTIONS>(aug_covariance,
                                                                      noise_covariance,
                                                                      measurement_matrix_nav,
                                                                      measurement_matrix_truth,
                                                                      kalman_gain);
}

template<Eigen::Index          MEAS_DIM,
         typename              DIM_S,
         bool                  USE_SS_KALMAN_GAIN,
         typename              SCALAR,
         Eigen::StorageOptions OPTIONS,
         typename              TRUTH_DERIVED,
         typename              NAV_DERIVED,
         typename              ERROR_COV_DERIVED,
         typename              AUG_COV_DERIVED>
inline void kf::applyMeasurementErrorBudget(const sensors::MeasurementBasePtr<MEAS_DIM,DIM_S,SCALAR,OPTIONS>& measurement,
                                            const noise::NoiseBasePtr<        MEAS_DIM,      SCALAR,OPTIONS>& measurement_noise,
                                            const SCALAR                                                      time,
                                            const Eigen::MatrixBase<TRUTH_DERIVED>&                           ref_truth_state,
                                            const Eigen::MatrixBase<NAV_DERIVED>&                             ref_nav_state,
                                            const Eigen::MatrixBase<ERROR_COV_DERIVED>&                       error_covariance,
                                                  Eigen::MatrixBase<AUG_COV_DERIVED>&                         aug_covariance)
{
  static_assert((int(TRUTH_DERIVED::    RowsAtCompileTime) == 1)                      or (int(TRUTH_DERIVED::     RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(TRUTH_DERIVED::    ColsAtCompileTime) == DIM_S::TRUTH_DIM)       or (int(TRUTH_DERIVED::     ColsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(NAV_DERIVED::      RowsAtCompileTime) == 1)                      or (int(NAV_DERIVED::      RowsAtCompileTime)  == Eigen::Dynamic));
  static_assert((int(NAV_DERIVED::      ColsAtCompileTime) == DIM_S::NAV_DIM)         or (int(NAV_DERIVED::      ColsAtCompileTime)  == Eigen::Dynamic));
  static_assert((int(ERROR_COV_DERIVED::RowsAtCompileTime) == DIM_S::ERROR_DIM)       or (int(ERROR_COV_DERIVED::RowsAtCompileTime)  == Eigen::Dynamic));
  static_assert((int(ERROR_COV_DERIVED::ColsAtCompileTime) == DIM_S::ERROR_DIM)       or (int(ERROR_COV_DERIVED::ColsAtCompileTime)  == Eigen::Dynamic));
  static_assert((int(AUG_COV_DERIVED::RowsAtCompileTime)   == DIM_S::LINCOV::AUG_DIM) or (int(AUG_COV_DERIVED::RowsAtCompileTime)    == Eigen::Dynamic));
  static_assert((int(AUG_COV_DERIVED::ColsAtCompileTime)   == DIM_S::LINCOV::AUG_DIM) or (int(AUG_COV_DERIVED::ColsAtCompileTime)    == Eigen::Dynamic));
  assert(ref_truth_state. rows() == 1);
  assert(ref_truth_state. cols() == DIM_S::TRUTH_DIM);
  assert(ref_nav_state.   rows() == 1);
  assert(ref_nav_state.   cols() == DIM_S::NAV_DIM);
  assert(error_covariance.rows() == DIM_S::ERROR_DIM);
  assert(error_covariance.cols() == DIM_S::ERROR_DIM);
  assert(aug_covariance.  rows() == DIM_S::LINCOV::AUG_DIM);
  assert(aug_covariance.  cols() == DIM_S::LINCOV::AUG_DIM);

  const Eigen::Matrix<SCALAR,MEAS_DIM,MEAS_DIM,             OPTIONS> noise_covariance         = measurement_noise->getCovariance();
  const Eigen::Matrix<SCALAR,MEAS_DIM,DIM_S::ERROR_DIM,     OPTIONS> measurement_matrix_nav   = measurement->getMeasurementEstimatePDWRErrorState(time, ref_nav_state);
  const Eigen::Matrix<SCALAR,MEAS_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS> measurement_matrix_truth = measurement->getMeasurementPDWRDispersionState(time, ref_truth_state);

  Eigen::Matrix<SCALAR,MEAS_DIM,DIM_S::ERROR_DIM,OPTIONS> kalman_gain;
  if constexpr(USE_SS_KALMAN_GAIN)
  {
    kalman_gain = measurement->getSteadyStateKalmanGain();
  }
  else // Don't use steady state kalman gain
  {
    kalman_gain = findKalmanGain<MEAS_DIM,DIM_S,SCALAR,OPTIONS>(error_covariance, noise_covariance, measurement_matrix_nav);
  }

  aug_covariance = updateAugCovariance<MEAS_DIM,DIM_S,SCALAR,OPTIONS>(aug_covariance,
                                                                      noise_covariance,
                                                                      measurement_matrix_nav,
                                                                      measurement_matrix_truth,
                                                                      kalman_gain);
}

template<Eigen::Index          MEAS_DIM,
         typename              DIM_S,
         typename              SCALAR,
         Eigen::StorageOptions OPTIONS,
         typename              ERROR_COV_DERIVED,
         typename              NOISE_DERIVED,
         typename              MEAS_MAT_DERIVED>
inline Eigen::Matrix<SCALAR,MEAS_DIM,DIM_S::ERROR_DIM,OPTIONS>
  kf::findKalmanGain(const Eigen::MatrixBase<ERROR_COV_DERIVED>& pre_error_covariance,
                     const Eigen::MatrixBase<NOISE_DERIVED>&     noise_covariance,
                     const Eigen::MatrixBase<MEAS_MAT_DERIVED>&  measurement_matrix_nav)
{
  const Eigen::Matrix<SCALAR,MEAS_DIM,MEAS_DIM,OPTIONS> to_invert =
    (measurement_matrix_nav * pre_error_covariance * measurement_matrix_nav.transpose()) + noise_covariance;
  const Eigen::Matrix<SCALAR,MEAS_DIM,DIM_S::ERROR_DIM,OPTIONS> rhs =
    (pre_error_covariance * measurement_matrix_nav.transpose()).transpose();

  if constexpr(1 == MEAS_DIM)
  {
    return rhs.array() / to_invert[0];
  }
  else
  {
    return to_invert.colPivHouseholderQr().solve(rhs); // slow
    //return to_invert.ldlt().solve(rhs);
    //return to_invert.llt().solve(rhs); // Fast
  }
}

template<Eigen::Index          MEAS_DIM,
         bool                  IS_EULER,
         typename              DIM_S,
         typename              SCALAR,
         Eigen::StorageOptions OPTIONS,
         typename              NAV_DERIVED,
         typename              KALMAN_DERIVED,
         typename              TRUE_MEAS_DERIVED,
         typename              PRED_MEAS_DERIVED>
inline Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>
  kf::updateNavState(const map::MappingsBasePtr<DIM_S,SCALAR,OPTIONS>& mappings,
                     const Eigen::MatrixBase<NAV_DERIVED>&             pre_nav_state,
                     const Eigen::MatrixBase<KALMAN_DERIVED>&          kalman_gain,
                     const Eigen::MatrixBase<TRUE_MEAS_DERIVED>&       true_measurement,
                     const Eigen::MatrixBase<PRED_MEAS_DERIVED>&       predicted_measurement)
{
  Eigen::Matrix<SCALAR,1,MEAS_DIM,OPTIONS> residual = true_measurement - predicted_measurement;
  if constexpr(IS_EULER)
  {
    for(Eigen::Index res_ind = 0; res_ind < MEAS_DIM; ++res_ind)
    {
      residual[res_ind] = std::atan2(std::sin(residual[res_ind]), std::cos(residual[res_ind]));
    }
  }
  const Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS> error_state = residual * kalman_gain;

  return mappings->correctErrors(pre_nav_state, error_state);
}

template<Eigen::Index          MEAS_DIM,
         typename              DIM_S,
         typename              SCALAR,
         Eigen::StorageOptions OPTIONS,
         typename              ERROR_COV_DERIVED,
         typename              NOISE_COV_DERIVED,
         typename              MEAS_MAT_DERIVED,
         typename              KALMAN_DERIVED>
inline Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS>
  kf::updateErrorCovariance(const Eigen::MatrixBase<ERROR_COV_DERIVED>& pre_error_covariance,
                            const Eigen::MatrixBase<NOISE_COV_DERIVED>& noise_covariance,
                            const Eigen::MatrixBase<MEAS_MAT_DERIVED>&  measurement_matrix_nav,
                            const Eigen::MatrixBase<KALMAN_DERIVED>&    kalman_gain)
{
  const Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS> eye_m_hk =
    Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS>::Identity() - (measurement_matrix_nav.transpose() * kalman_gain);

  return (eye_m_hk.   transpose() * pre_error_covariance * eye_m_hk) +
         (kalman_gain.transpose() * noise_covariance     * kalman_gain);
}

template<Eigen::Index          MEAS_DIM,
         typename              DIM_S,
         typename              SCALAR,
         Eigen::StorageOptions OPTIONS,
         typename              AUG_COV_DERIVED,
         typename              NOISE_COV_DERIVED,
         typename              MEAS_MAT_NAV_DERIVED,
         typename              MEAS_MAT_TRUE_DERIVED,
         typename              KALMAN_DERIVED>
inline Eigen::Matrix<SCALAR,DIM_S::LINCOV::AUG_DIM,DIM_S::LINCOV::AUG_DIM,OPTIONS>
  kf::updateAugCovariance(const Eigen::MatrixBase<AUG_COV_DERIVED>&       pre_aug_covariance,
                          const Eigen::MatrixBase<NOISE_COV_DERIVED>&     noise_covariance,
                          const Eigen::MatrixBase<MEAS_MAT_NAV_DERIVED>&  measurement_matrix_nav,
                          const Eigen::MatrixBase<MEAS_MAT_TRUE_DERIVED>& measurement_matrix_truth,
                          const Eigen::MatrixBase<KALMAN_DERIVED>&        kalman_gain)
{
  Eigen::Matrix<SCALAR,DIM_S::LINCOV::AUG_DIM,DIM_S::LINCOV::AUG_DIM,OPTIONS> A;
  Eigen::Matrix<SCALAR,MEAS_DIM,              DIM_S::LINCOV::AUG_DIM,OPTIONS> B;

  A.template block<DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_DISP_DIM>(DIM_S::LINCOV::TRUTH_DISP_START_IND,
                                                                DIM_S::LINCOV::TRUTH_DISP_START_IND).setIdentity();
  A.template block<DIM_S::TRUTH_DISP_DIM,DIM_S::ERROR_DIM>(DIM_S::LINCOV::TRUTH_DISP_START_IND,
                                                           DIM_S::LINCOV::NAV_DISP_START_IND).setZero();
  A.template block<DIM_S::ERROR_DIM,DIM_S::TRUTH_DISP_DIM>(DIM_S::LINCOV::NAV_DISP_START_IND,
                                                           DIM_S::LINCOV::TRUTH_DISP_START_IND).noalias() =
    kalman_gain.transpose() * measurement_matrix_truth;
  A.template block<DIM_S::ERROR_DIM,DIM_S::ERROR_DIM>(DIM_S::LINCOV::NAV_DISP_START_IND,
                                                      DIM_S::LINCOV::NAV_DISP_START_IND) =
    Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS>::Identity() - (kalman_gain.transpose() * measurement_matrix_nav);

  B.template middleCols<DIM_S::TRUTH_DISP_DIM>(DIM_S::LINCOV::TRUTH_DISP_START_IND).setZero();
  B.template middleCols<DIM_S::ERROR_DIM>(     DIM_S::LINCOV::NAV_DISP_START_IND) = kalman_gain;

  return (A * pre_aug_covariance * A.transpose()) + (B.transpose() * noise_covariance * B);
}

#endif
/* measurement_update.hpp */
