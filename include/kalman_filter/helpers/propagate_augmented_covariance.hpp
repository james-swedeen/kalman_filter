/**
 * @File: propagate_augmented_covariance.hpp
 * @Date: May 2022
 * @Author: James Swedeen
 *
 * @brief
 * Function used to propagate the augmented state covariance matrix.
 **/

#ifndef KALMAN_FILTER_HELPERS_PROPAGATE_AUGMENTED_COVARIANCE_HPP
#define KALMAN_FILTER_HELPERS_PROPAGATE_AUGMENTED_COVARIANCE_HPP

/* C++ Headers */
#include<functional>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<kalman_filter/dynamics/dynamics_base.hpp>
#include<kalman_filter/sensors/inertial_measurements/inertial_measurement_base.hpp>
#include<kalman_filter/controllers/controller_base.hpp>
#include<kalman_filter/helpers/versions.hpp>
#include<kalman_filter/helpers/integrator_step.hpp>

namespace kf
{
/**
 * @propagateAugCovariance
 *
 * @brief
 * The function that propagates the augmented state covariance matrix.
 *
 * @templates
 * DIM_S: The type of a Dimensions object or an inheriting object that has information
 *        about the size of the state vectors
 * VERSION: Controls what type of simulation will be ran
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 * AUG_COV_DERIVED: The matrix type of the augmented state covariance
 * NAV_DERIVED: The matrix type of the navigation state
 * TRUTH_DERIVED: The matrix type of the truth state
 * INER_MEAS_DERIVED: The matrix type of the inertial measurement state
 *
 * @parameters
 * prev_aug_cov: The old augmented state covariance matrix
 * dynamics: Defines the truth and navigation state dynamics
 * controller: A helper that is used to calculate control inputs
 * inertial_measurements: Produces inertial measurement readings
 * simulation_dt: The time step between each state vector in the simulation
 * time: The current simulation time
 * ref_state: The current state from the reference trajectory
 * ref_truth_state: The current state from the reference trajectory mapped into a truth state vector
 * ref_nav_state: The current state from the reference trajectory mapped into a navigation state vector
 * ref_control_input: The current control vector
 * ref_inertial_reading: The inertial measurements without biases and noise
 * truth_process_noise_cov: The cov of the truth state process noise
 * inertial_noise_cov: The cov of the noise from the inertial measurements
 *
 * @return
 * The new augmented state covariance matrix.
 **/
template<typename              DIM_S,
         Versions              VERSION,
         typename              SCALAR,
         Eigen::StorageOptions OPTIONS,
         typename              AUG_COV_DERIVED,
         typename              REF_DERIVED,
         typename              TRUTH_DERIVED,
         typename              NAV_DERIVED,
         typename              CONTROL_DERIVED,
         typename              INER_MEAS_DERIVED,
         typename              TRUTH_PROCESS_NOISE_COV_DERIVED,
         typename              INER_MEAS_COV_DERIVED>
inline typename AUG_COV_DERIVED::PlainMatrix
  propagateAugCovariance(const Eigen::MatrixBase<AUG_COV_DERIVED>&                        prev_aug_cov,
                         const dynamics::DynamicsBasePtr<          DIM_S,SCALAR,OPTIONS>& dynamics,
                         const control::ControllerBasePtr<         DIM_S,SCALAR,OPTIONS>& controller,
                         const sensors::InertialMeasurementBasePtr<DIM_S,SCALAR,OPTIONS>& inertial_measurements,
                         const SCALAR                                                     simulation_dt,
                         const SCALAR                                                     time,
                         const Eigen::MatrixBase<REF_DERIVED>&                            ref_state,
                         const Eigen::MatrixBase<TRUTH_DERIVED>&                          ref_truth_state,
                         const Eigen::MatrixBase<NAV_DERIVED>&                            ref_nav_state,
                         const Eigen::MatrixBase<CONTROL_DERIVED>&                        ref_control_input,
                         const Eigen::MatrixBase<INER_MEAS_DERIVED>&                      ref_inertial_reading,
                         const Eigen::MatrixBase<TRUTH_PROCESS_NOISE_COV_DERIVED>&        truth_process_noise_cov,
                         const Eigen::MatrixBase<INER_MEAS_COV_DERIVED>&                  inertial_noise_cov);
} // namespace kf

template<typename              DIM_S,
         kf::Versions          VERSION,
         typename              SCALAR,
         Eigen::StorageOptions OPTIONS,
         typename              AUG_COV_DERIVED,
         typename              REF_DERIVED,
         typename              TRUTH_DERIVED,
         typename              NAV_DERIVED,
         typename              CONTROL_DERIVED,
         typename              INER_MEAS_DERIVED,
         typename              TRUTH_PROCESS_NOISE_COV_DERIVED,
         typename              INER_MEAS_COV_DERIVED>
inline typename AUG_COV_DERIVED::PlainMatrix
  kf::propagateAugCovariance(const Eigen::MatrixBase<AUG_COV_DERIVED>&                        prev_aug_cov,
                             const dynamics::DynamicsBasePtr<          DIM_S,SCALAR,OPTIONS>& dynamics,
                             const control::ControllerBasePtr<         DIM_S,SCALAR,OPTIONS>& controller,
                             const sensors::InertialMeasurementBasePtr<DIM_S,SCALAR,OPTIONS>& inertial_measurements,
                             const SCALAR                                                     simulation_dt,
                             const SCALAR                                                     time,
                             const Eigen::MatrixBase<REF_DERIVED>&                            ref_state,
                             const Eigen::MatrixBase<TRUTH_DERIVED>&                          ref_truth_state,
                             const Eigen::MatrixBase<NAV_DERIVED>&                            ref_nav_state,
                             const Eigen::MatrixBase<CONTROL_DERIVED>&                        ref_control_input,
                             const Eigen::MatrixBase<INER_MEAS_DERIVED>&                      ref_inertial_reading,
                             const Eigen::MatrixBase<TRUTH_PROCESS_NOISE_COV_DERIVED>&        truth_process_noise_cov,
                             const Eigen::MatrixBase<INER_MEAS_COV_DERIVED>&                  inertial_noise_cov)
{
  Eigen::Matrix<SCALAR,DIM_S::LINCOV::AUG_DIM,DIM_S::LINCOV::AUG_DIM,OPTIONS> aug_dynamics_matrix;
  Eigen::Matrix<SCALAR,DIM_S::LINCOV::AUG_DIM,DIM_S::LINCOV::AUG_DIM,OPTIONS> additive_noise;

  const Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,DIM_S::ERROR_DIM,OPTIONS> nav_dynamics_pd_inertial_t =
    dynamics->getNavStateDynamicsPDWRInertialMeasurement(time, ref_nav_state, ref_state, ref_inertial_reading);
  const Eigen::Matrix<SCALAR,DIM_S::CONTROL_DIM,DIM_S::ERROR_DIM,OPTIONS> control_pd_error_state =
    controller->getControlPDWRErrorState(time, ref_nav_state, ref_state);
  const Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_NOISE_DIM,OPTIONS> truth_dynamics_pd_noise =
    dynamics->getTruthStateDynamicsPDWRNoise(time, ref_truth_state, ref_control_input);

  // Fill in dynamics matrix
  aug_dynamics_matrix.template topRows<DIM_S::TRUTH_DISP_DIM>().template leftCols<DIM_S::TRUTH_DISP_DIM>() =
    dynamics->getTruthStateDynamicsPDWRDispersionState(time, ref_truth_state, ref_control_input);

  if constexpr(openLoop(VERSION))
  {
    aug_dynamics_matrix.template topRows<DIM_S::TRUTH_DISP_DIM>().template rightCols<DIM_S::ERROR_DIM>().setZero();
  }
  else // Not open-loop
  {
    aug_dynamics_matrix.template topRows<DIM_S::TRUTH_DISP_DIM>().template rightCols<DIM_S::ERROR_DIM>().noalias() =
      dynamics->getTruthStateDynamicsPDWRControl(time, ref_truth_state, ref_control_input).transpose() *
      control_pd_error_state;
  }

  if constexpr(modelReplacement(VERSION))
  {
    aug_dynamics_matrix.template bottomRows<DIM_S::ERROR_DIM>().template leftCols<DIM_S::TRUTH_DISP_DIM>().noalias() =
      nav_dynamics_pd_inertial_t.transpose() *
      inertial_measurements->getMeasurementPDWRDispersionState(time, ref_truth_state, ref_control_input);
  }
  else // Not using model replacement
  {
    aug_dynamics_matrix.template bottomRows<DIM_S::ERROR_DIM>().template leftCols<DIM_S::TRUTH_DISP_DIM>().setZero();
  }

  aug_dynamics_matrix.template bottomRows<DIM_S::ERROR_DIM>().template rightCols<DIM_S::ERROR_DIM>() =
    dynamics->getNavStateDynamicsPDWRErrorState(time, ref_nav_state, ref_state, ref_inertial_reading);
  if constexpr(modelReplacement(VERSION) and (not openLoop(VERSION)))
  {
    aug_dynamics_matrix.template bottomRows<DIM_S::ERROR_DIM>().template rightCols<DIM_S::ERROR_DIM>() +=
      nav_dynamics_pd_inertial_t.transpose() *
      inertial_measurements->getMeasurementPDWRControl(time, ref_truth_state, ref_control_input) *
      control_pd_error_state;
  }

  // Fill in noise matrix
  additive_noise.setZero();

  additive_noise.template topRows<DIM_S::TRUTH_DISP_DIM>().template leftCols<DIM_S::TRUTH_DISP_DIM>().noalias() =
    truth_dynamics_pd_noise * truth_process_noise_cov * truth_dynamics_pd_noise.transpose();

  if constexpr(modelReplacement(VERSION))
  {
    const Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,DIM_S::INER_MEAS_NOISE_DIM,OPTIONS> inertial_measurement_pd_noise =
      inertial_measurements->getMeasurementPDWRNoise(time, ref_truth_state, ref_control_input);

    additive_noise.template bottomRows<DIM_S::ERROR_DIM>().template rightCols<DIM_S::ERROR_DIM>().noalias() =
      nav_dynamics_pd_inertial_t.transpose() *
      inertial_measurement_pd_noise *
      inertial_noise_cov *
      inertial_measurement_pd_noise.transpose() *
      nav_dynamics_pd_inertial_t;
  }

  // Propagate
  const std::function<Eigen::Matrix<SCALAR,DIM_S::LINCOV::AUG_DIM,DIM_S::LINCOV::AUG_DIM,OPTIONS>(
      const Eigen::Ref<const Eigen::Matrix<SCALAR,DIM_S::LINCOV::AUG_DIM,DIM_S::LINCOV::AUG_DIM,OPTIONS>>&)> aug_covariance_de =
    [&aug_dynamics_matrix, &additive_noise]
    (const Eigen::Ref<const Eigen::Matrix<SCALAR,DIM_S::LINCOV::AUG_DIM,DIM_S::LINCOV::AUG_DIM,OPTIONS>>& prev_aug_covariance)
    {
      return (aug_dynamics_matrix * prev_aug_covariance) +
             (prev_aug_covariance * aug_dynamics_matrix.transpose()) +
             additive_noise;
    };

  return integratorStep<VERSION,SCALAR>(aug_covariance_de, prev_aug_cov, simulation_dt);
}

#endif
/* propagate_augmented_covariance.hpp */
