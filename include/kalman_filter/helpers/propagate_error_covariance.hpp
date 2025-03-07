/**
 * @File: propagate_error_covariance.hpp
 * @Date: May 2022
 * @Author: James Swedeen
 *
 * @brief
 * Function used to propagate the error state covariance matrix.
 **/

#ifndef KALMAN_FILTER_HELPERS_PROPAGATE_ERROR_COVARIANCE_HPP
#define KALMAN_FILTER_HELPERS_PROPAGATE_ERROR_COVARIANCE_HPP

/* C++ Headers */
#include<functional>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<kalman_filter/helpers/versions.hpp>
#include<kalman_filter/sensors/inertial_measurements/inertial_measurement_base.hpp>
#include<kalman_filter/mappings/mappings_base.hpp>
#include<kalman_filter/dynamics/dynamics_base.hpp>
#include<kalman_filter/helpers/integrator_step.hpp>

namespace kf
{
/**
 * @propagateErrorCovariance
 *
 * @brief
 * The function that propagates the error state covariance matrix.
 *
 * @templates
 * DIM_S: The type of a Dimensions object or an inheriting object that has information
 *        about the size of the state vectors
 * VERSION: Controls what type of simulation will be ran
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 * ERROR_COV_DERIVED: The matrix type of the error state covariance
 * NAV_DERIVED: The matrix type of the navigation state
 * INER_MEAS_DERIVED: The matrix type of the inertial measurement state
 *
 * @parameters
 * prev_error_cov: The old error state covariance matrix
 * dynamics: Defines the truth and navigation state dynamics
 * inertial_measurements: A helper class that produces inertial measurement readings like the output of an IMU
 * mappings: A helper object that maps one state vector to another
 * simulation_dt: The time step between each state vector in the simulation
 * time: The current simulation time
 * nav_state: The current navigation state vector
 * ref_state: The reference state of the current time step
 * control_input: The current control vector
 * inertial_reading: The inertial measurements with biases and noise
 * truth_process_noise_cov: The covariance of the truth state process noise
 * inertial_noise_cov: The covariance of the noise from the inertial measurements
 *
 * @return
 * The new error state covariance matrix.
 **/
template<typename              DIM_S,
         Versions              VERSION,
         typename              SCALAR,
         Eigen::StorageOptions OPTIONS,
         typename              ERROR_COV_DERIVED,
         typename              NAV_DERIVED,
         typename              REF_DERIVED,
         typename              CONTROL_DERIVED,
         typename              INER_MEAS_DERIVED,
         typename              TRUTH_PROCESS_NOISE_COV_DERIVED,
         typename              INER_MEAS_COV_DERIVED>
inline typename ERROR_COV_DERIVED::PlainMatrix
  propagateErrorCovariance(const Eigen::MatrixBase<ERROR_COV_DERIVED>&                      prev_error_cov,
                           const dynamics::DynamicsBasePtr<          DIM_S,SCALAR,OPTIONS>& dynamics,
                           const sensors::InertialMeasurementBasePtr<DIM_S,SCALAR,OPTIONS>& inertial_measurements,
                           const map::MappingsBasePtr<               DIM_S,SCALAR,OPTIONS>& mappings,
                           const SCALAR                                                     simulation_dt,
                           const SCALAR                                                     time,
                           const Eigen::MatrixBase<NAV_DERIVED>&                            nav_state,
                           const Eigen::MatrixBase<REF_DERIVED>&                            ref_state,
                           const Eigen::MatrixBase<CONTROL_DERIVED>&                        control_input,
                           const Eigen::MatrixBase<INER_MEAS_DERIVED>&                      inertial_reading,
                           const Eigen::MatrixBase<TRUTH_PROCESS_NOISE_COV_DERIVED>&        truth_process_noise_cov,
                           const Eigen::MatrixBase<INER_MEAS_COV_DERIVED>&                  inertial_noise_cov);
} // namespace kf

template<typename              DIM_S,
         kf::Versions          VERSION,
         typename              SCALAR,
         Eigen::StorageOptions OPTIONS,
         typename              ERROR_COV_DERIVED,
         typename              NAV_DERIVED,
         typename              REF_DERIVED,
         typename              CONTROL_DERIVED,
         typename              INER_MEAS_DERIVED,
         typename              TRUTH_PROCESS_NOISE_COV_DERIVED,
         typename              INER_MEAS_COV_DERIVED>
inline typename ERROR_COV_DERIVED::PlainMatrix
  kf::propagateErrorCovariance(const Eigen::MatrixBase<ERROR_COV_DERIVED>&                      prev_error_cov,
                               const dynamics::DynamicsBasePtr<          DIM_S,SCALAR,OPTIONS>& dynamics,
                               const sensors::InertialMeasurementBasePtr<DIM_S,SCALAR,OPTIONS>& inertial_measurements,
                               const map::MappingsBasePtr<               DIM_S,SCALAR,OPTIONS>& mappings,
                               const SCALAR                                                     simulation_dt,
                               const SCALAR                                                     time,
                               const Eigen::MatrixBase<NAV_DERIVED>&                            nav_state,
                               const Eigen::MatrixBase<REF_DERIVED>&                            ref_state,
                               const Eigen::MatrixBase<CONTROL_DERIVED>&                        control_input,
                               const Eigen::MatrixBase<INER_MEAS_DERIVED>&                      inertial_reading,
                               const Eigen::MatrixBase<TRUTH_PROCESS_NOISE_COV_DERIVED>&        truth_process_noise_cov,
                               const Eigen::MatrixBase<INER_MEAS_COV_DERIVED>&                  inertial_noise_cov)
{
  // Matrix size checks
  static_assert((int(ERROR_COV_DERIVED::              RowsAtCompileTime) == DIM_S::ERROR_DIM)       or (int(ERROR_COV_DERIVED::              RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(ERROR_COV_DERIVED::              ColsAtCompileTime) == DIM_S::ERROR_DIM)       or (int(ERROR_COV_DERIVED::              ColsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(NAV_DERIVED::                    RowsAtCompileTime) == 1)                      or (int(NAV_DERIVED::                    RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(NAV_DERIVED::                    ColsAtCompileTime) == DIM_S::NAV_DIM)         or (int(NAV_DERIVED::                    ColsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(INER_MEAS_DERIVED::              RowsAtCompileTime) == 1)                      or (int(INER_MEAS_DERIVED::              RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(INER_MEAS_DERIVED::              ColsAtCompileTime) == DIM_S::INER_MEAS_DIM)   or (int(INER_MEAS_DERIVED::              ColsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(TRUTH_PROCESS_NOISE_COV_DERIVED::RowsAtCompileTime) == DIM_S::TRUTH_NOISE_DIM) or (int(TRUTH_PROCESS_NOISE_COV_DERIVED::RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(TRUTH_PROCESS_NOISE_COV_DERIVED::ColsAtCompileTime) == DIM_S::TRUTH_NOISE_DIM) or (int(TRUTH_PROCESS_NOISE_COV_DERIVED::ColsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(INER_MEAS_COV_DERIVED::          RowsAtCompileTime) == DIM_S::INER_MEAS_DIM)   or (int(INER_MEAS_COV_DERIVED::          RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(INER_MEAS_COV_DERIVED::          ColsAtCompileTime) == DIM_S::INER_MEAS_DIM)   or (int(INER_MEAS_COV_DERIVED::          ColsAtCompileTime) == Eigen::Dynamic));
  assert(prev_error_cov.         rows() == DIM_S::ERROR_DIM);
  assert(prev_error_cov.         cols() == DIM_S::ERROR_DIM);
  assert(nav_state.              rows() == 1);
  assert(nav_state.              cols() == DIM_S::NAV_DIM);
  assert(inertial_reading.       rows() == 1);
  assert(inertial_reading.       cols() == DIM_S::INER_MEAS_DIM);
  assert(truth_process_noise_cov.rows() == DIM_S::TRUTH_NOISE_DIM);
  assert(truth_process_noise_cov.cols() == DIM_S::TRUTH_NOISE_DIM);
  assert(inertial_noise_cov.     rows() == DIM_S::INER_MEAS_DIM);
  assert(inertial_noise_cov.     cols() == DIM_S::INER_MEAS_DIM);

  // Find needed variables
  const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS> truth_state = mappings->mapNavTruth(nav_state);
  const Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_NOISE_DIM,OPTIONS> truth_dynamics_pd_noise =
    dynamics->getTruthStateDynamicsPDWRNoise(time, truth_state, control_input);
  const Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS> truth_nav_map_pd_disp_state =
    mappings->getTruthNavMapPDWRDispersionState(truth_state);
  const Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS> linear_error_dynamics =
    dynamics->getNavStateDynamicsPDWRErrorState(time, nav_state, ref_state, inertial_reading);

  Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS> error_noise_cov =
    truth_nav_map_pd_disp_state *
    truth_dynamics_pd_noise *
    truth_process_noise_cov *
    truth_dynamics_pd_noise.transpose() *
    truth_nav_map_pd_disp_state.transpose();

  if constexpr(modelReplacement(VERSION))
  {
    const Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,DIM_S::ERROR_DIM,OPTIONS> nav_dynamics_pd_inertial_t =
      dynamics->getNavStateDynamicsPDWRInertialMeasurement(time, nav_state, ref_state, inertial_reading);
    const Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,DIM_S::INER_MEAS_NOISE_DIM,OPTIONS> inertial_reading_pd_noise =
      inertial_measurements->getMeasurementPDWRNoise(time, truth_state, control_input);

    error_noise_cov += nav_dynamics_pd_inertial_t.transpose() *
                       inertial_reading_pd_noise *
                       inertial_noise_cov *
                       inertial_reading_pd_noise.transpose() *
                       nav_dynamics_pd_inertial_t;
  }

  // Test for the linearized error state dynamics
/*  {
    Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,     OPTIONS> error_state;
    Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DISP_DIM,OPTIONS> truth_disp;

    error_state.setZero();
    std::cout << "error state poses" << std::endl;
    std::cin >> error_state[0] >> error_state[1] >> error_state[2];
    std::cout << "error state Euler" << std::endl;
    std::cin >> error_state[3] >> error_state[4] >> error_state[5];
    std::cout << "error state vels" << std::endl;
    std::cin >> error_state[6] >> error_state[7] >> error_state[8];

    truth_disp.template leftCols<6>() = error_state.template leftCols<6>();
    truth_disp[6] = error_state.template rightCols<3>().norm();

    // Inject error
    const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS> error_truth_state = mappings->injectErrors(truth_state, truth_disp);
    // Propagate states
    const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS> new_truth_state =
      integratorStep<VERSION,SCALAR,Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>(
        [&dynamics, &control_input, time]
        (const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& vec)
        {
          return dynamics->getTruthStateDynamics(time, vec, control_input, Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_NOISE_DIM,OPTIONS>::Zero());
        },
        error_truth_state,
        simulation_dt);
    const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS> new_nav_state =
      integratorStep<VERSION,SCALAR,Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>(
        [&dynamics, &inertial_reading, time]
        (const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& vec)
        {
          return dynamics->getNavStateDynamics(time, vec, inertial_reading);
        },
        nav_state,
        simulation_dt);
    const Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS> est_new_error_state =
      integratorStep<VERSION,SCALAR,Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS>>(
        [&linear_error_dynamics]
        (const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS>>& error_vec)
        {
          return linear_error_dynamics * error_vec.transpose();
        },
        error_state,
        simulation_dt);
    // Compare estimation errors
    const Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS> real_new_error_state = mappings->calculateErrorState(new_truth_state, new_nav_state);
    const Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS> new_error_diff = est_new_error_state - real_new_error_state;
    std::cout << "control_input:\n" << control_input << std::endl;
    std::cout << "inertial_reading:\n" << inertial_reading << std::endl;
    std::cout << "simulation_dt: " << simulation_dt << std::endl;
    std::cout << "nav_state:\n" << nav_state << std::endl;
    std::cout << "truth_state:\n" << truth_state << std::endl;
    std::cout << "error_state:\n" << error_state << std::endl;
    std::cout << "truth_disp:\n" << truth_disp << std::endl;
    std::cout << "error_truth_state:\n" << error_truth_state << std::endl;
    std::cout << "new_truth_state:\n" << new_truth_state << std::endl;
    std::cout << "new_nav_state:\n" << new_nav_state << std::endl;
    std::cout << "est_new_error_state:\n" << est_new_error_state << std::endl;
    std::cout << "real_new_error_state:\n" << real_new_error_state << std::endl;
    std::cout << "new_error_diff:\n" << new_error_diff << std::endl;
    std::cin.get();
  }*/

  const std::function<Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS>(
      const Eigen::Ref<const Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS>>&)> error_cov_de =
    [&linear_error_dynamics, &error_noise_cov]
    (const Eigen::Ref<const Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS>>& prev_error_cov)
    {
      return (linear_error_dynamics * prev_error_cov) +
             (prev_error_cov * linear_error_dynamics.transpose()) +
             error_noise_cov;
    };

  return integratorStep<VERSION,SCALAR>(error_cov_de, prev_error_cov, simulation_dt);
}

#endif
/* propagate_error_covariance.hpp */
