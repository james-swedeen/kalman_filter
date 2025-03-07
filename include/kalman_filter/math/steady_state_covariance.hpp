/**
 * @File: steady_state_covariance.hpp
 * @Date: June 2023
 * @Author: James Swedeen
 *
 * @brief
 * Header contains helper functions for calculating the steady state error state covariance.
 **/

#ifndef KALMAN_FILTER_MATH_STEADY_STATE_COVARIANCE_HPP
#define KALMAN_FILTER_MATH_STEADY_STATE_COVARIANCE_HPP

/* C++ Headers */
#include<tuple>
#include<vector>
#include<limits>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<kalman_filter/sensors/inertial_measurements/inertial_measurement_base.hpp>
#include<kalman_filter/sensors/measurements/measurement_base.hpp>
#include<kalman_filter/mappings/mappings_base.hpp>
#include<kalman_filter/dynamics/dynamics_base.hpp>

// Helper function that allows sorting of matrix columns/rows in Eigen
namespace Eigen {
  template<class T>
  inline void swap(T&& a, T&& b)
  {
    a.swap(b);
  }
}

namespace kf
{
namespace math
{
namespace ss
{
/**
 * @findSteadyStateErrorCov
 *
 * @brief
 * Function used to calculate the steady state covariance of a given system.
 *
 * @templates
 * DIM_S: The type of a Dimensions object or an inheriting object that has information
 *        about the size of the state vectors
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 *
 * @parameters
 * dynamics: Defines the truth and navigation state dynamics
 * inertial_measurements: A helper class that produces inertial measurement readings like the output of an IMU
 * mappings: A helper object that maps one state vector to another
 * one_dim_measurements: A vector containing information on all one dimensional measurements,
 *                       the vector contains: 1) the measurement object, 2) the measurement's noise covariance, 3) the measurement's period with zero meaning continuous
 * two_dim_measurements: A vector containing information on all two dimensional measurements,
 *                       the vector contains: 1) the measurement object, 2) the measurement's noise covariance, 3) the measurement's period with zero meaning continuous
 * three_dim_measurements: A vector containing information on all three dimensional measurements,
 *                         the vector contains: 1) the measurement object, 2) the measurement's noise covariance, 3) the measurement's period with zero meaning continuous
 * six_dim_measurements: A vector containing information on all six dimensional measurements,
 *                         the vector contains: 1) the measurement object, 2) the measurement's noise covariance, 3) the measurement's period with zero meaning continuous
 * truth_process_noise_cov: The covariance of the truth state process noise
 * inertial_noise_cov: The covariance of the noise from the inertial measurements, leave as default if not using model replacement
 * ss_nav_state: The navigation state vector at which to linearize the system to, leave as default if it doesn't matter to your system
 * ss_control_input: The current control vector at which to linearize the system to, leave as default if it doesn't matter to your system
 * ss_inertial_reading: The inertial measurements with biases and noise at which to linearize the system to, leave as default if it doesn't matter to your system
 * ss_time: The simulation time at which to linearize the system to, leave as default if it doesn't matter to your system
 *
 * @return
 * The steady-state error-state covariance.
 **/
template<Eigen::Index MEAS_SIZE, typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using MEASUREMENT_INFO_TUPLE = std::tuple<sensors::MeasurementBasePtr<MEAS_SIZE,DIM_S,SCALAR,OPTIONS>,
                                          Eigen::Matrix<SCALAR,MEAS_SIZE,MEAS_SIZE,OPTIONS>,
                                          SCALAR>;

template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS>
  findSteadyStateErrorCov(
    const dynamics::DynamicsBasePtr<           DIM_S,SCALAR,OPTIONS>&                                    dynamics,
    const sensors::InertialMeasurementBasePtr< DIM_S,SCALAR,OPTIONS>&                                    inertial_measurements,
    const map::MappingsBasePtr<                DIM_S,SCALAR,OPTIONS>&                                    mappings,
    const std::vector<MEASUREMENT_INFO_TUPLE<1,DIM_S,SCALAR,OPTIONS>>&                                   one_dim_measurements,
    const std::vector<MEASUREMENT_INFO_TUPLE<2,DIM_S,SCALAR,OPTIONS>>&                                   two_dim_measurements,
    const std::vector<MEASUREMENT_INFO_TUPLE<3,DIM_S,SCALAR,OPTIONS>>&                                   three_dim_measurements,
    const std::vector<MEASUREMENT_INFO_TUPLE<6,DIM_S,SCALAR,OPTIONS>>&                                   six_dim_measurements,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,DIM_S::TRUTH_NOISE_DIM,DIM_S::TRUTH_NOISE_DIM,OPTIONS>>& truth_process_noise_cov,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,  DIM_S::INER_MEAS_DIM,  OPTIONS>>& inertial_noise_cov  = Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,DIM_S::INER_MEAS_DIM,OPTIONS>::Constant(std::numeric_limits<SCALAR>::quiet_NaN()),
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::NAV_DIM,        OPTIONS>>& ss_nav_state        = Eigen::Matrix<SCALAR,1,                   DIM_S::NAV_DIM,      OPTIONS>::Constant(std::numeric_limits<SCALAR>::quiet_NaN()),
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::CONTROL_DIM,    OPTIONS>>& ss_control_input    = Eigen::Matrix<SCALAR,1,                   DIM_S::CONTROL_DIM,  OPTIONS>::Constant(std::numeric_limits<SCALAR>::quiet_NaN()),
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::INER_MEAS_DIM,  OPTIONS>>& ss_inertial_reading = Eigen::Matrix<SCALAR,1,                   DIM_S::INER_MEAS_DIM,OPTIONS>::Constant(std::numeric_limits<SCALAR>::quiet_NaN()),
    const SCALAR                                                                                         ss_time             = std::numeric_limits<SCALAR>::quiet_NaN());
/**
 * @hamiltonianMatrix
 *
 * @brief
 * Function used to calculate the Hamiltonian matrix.
 *
 * @templates
 * DIM_S: The type of a Dimensions object or an inheriting object that has information
 *        about the size of the state vectors
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 *
 * @parameters
 * dynamics: Defines the truth and navigation state dynamics
 * inertial_measurements: A helper class that produces inertial measurement readings like the output of an IMU
 * mappings: A helper object that maps one state vector to another
 * one_dim_measurements: A vector containing information on all one dimensional measurements,
 *                       the vector contains: 1) the measurement object, 2) the measurement's noise covariance, 3) the measurement's period with zero meaning continuous
 * two_dim_measurements: A vector containing information on all two dimensional measurements,
 *                       the vector contains: 1) the measurement object, 2) the measurement's noise covariance, 3) the measurement's period with zero meaning continuous
 * three_dim_measurements: A vector containing information on all three dimensional measurements,
 *                         the vector contains: 1) the measurement object, 2) the measurement's noise covariance, 3) the measurement's period with zero meaning continuous
 * six_dim_measurements: A vector containing information on all six dimensional measurements,
 *                         the vector contains: 1) the measurement object, 2) the measurement's noise covariance, 3) the measurement's period with zero meaning continuous
 * truth_process_noise_cov: The covariance of the truth state process noise
 * inertial_noise_cov: The covariance of the noise from the inertial measurements
 * nav_state: The navigation state vector at which to linearize the system to
 * control_input: The current control vector at which to linearize the system to
 * inertial_reading: The inertial measurements with biases and noise at which to linearize the system to
 * time: The simulation time at which to linearize the system to
 *
 * @return
 * The Hamiltonian matrix.
 **/
template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
inline Eigen::Matrix<SCALAR,2*DIM_S::ERROR_DIM,2*DIM_S::ERROR_DIM,OPTIONS>
  hamiltonianMatrix(
    const dynamics::DynamicsBasePtr<           DIM_S,SCALAR,OPTIONS>&                                    dynamics,
    const sensors::InertialMeasurementBasePtr< DIM_S,SCALAR,OPTIONS>&                                    inertial_measurements,
    const map::MappingsBasePtr<                DIM_S,SCALAR,OPTIONS>&                                    mappings,
    const std::vector<MEASUREMENT_INFO_TUPLE<1,DIM_S,SCALAR,OPTIONS>>&                                   one_dim_measurements,
    const std::vector<MEASUREMENT_INFO_TUPLE<2,DIM_S,SCALAR,OPTIONS>>&                                   two_dim_measurements,
    const std::vector<MEASUREMENT_INFO_TUPLE<3,DIM_S,SCALAR,OPTIONS>>&                                   three_dim_measurements,
    const std::vector<MEASUREMENT_INFO_TUPLE<6,DIM_S,SCALAR,OPTIONS>>&                                   six_dim_measurements,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,DIM_S::TRUTH_NOISE_DIM,DIM_S::TRUTH_NOISE_DIM,OPTIONS>>& truth_process_noise_cov,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,  DIM_S::INER_MEAS_DIM,  OPTIONS>>& inertial_noise_cov,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::NAV_DIM,        OPTIONS>>& nav_state,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::CONTROL_DIM,    OPTIONS>>& control_input,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::INER_MEAS_DIM,  OPTIONS>>& inertial_reading,
    const SCALAR                                                                                         time);
} // ss


template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS>
  ss::findSteadyStateErrorCov(
    const dynamics::DynamicsBasePtr<           DIM_S,SCALAR,OPTIONS>&                                    dynamics,
    const sensors::InertialMeasurementBasePtr< DIM_S,SCALAR,OPTIONS>&                                    inertial_measurements,
    const map::MappingsBasePtr<                DIM_S,SCALAR,OPTIONS>&                                    mappings,
    const std::vector<MEASUREMENT_INFO_TUPLE<1,DIM_S,SCALAR,OPTIONS>>&                                   one_dim_measurements,
    const std::vector<MEASUREMENT_INFO_TUPLE<2,DIM_S,SCALAR,OPTIONS>>&                                   two_dim_measurements,
    const std::vector<MEASUREMENT_INFO_TUPLE<3,DIM_S,SCALAR,OPTIONS>>&                                   three_dim_measurements,
    const std::vector<MEASUREMENT_INFO_TUPLE<6,DIM_S,SCALAR,OPTIONS>>&                                   six_dim_measurements,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,DIM_S::TRUTH_NOISE_DIM,DIM_S::TRUTH_NOISE_DIM,OPTIONS>>& truth_process_noise_cov,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,  DIM_S::INER_MEAS_DIM,  OPTIONS>>& inertial_noise_cov,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::NAV_DIM,        OPTIONS>>& ss_nav_state,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::CONTROL_DIM,    OPTIONS>>& ss_control_input,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::INER_MEAS_DIM,  OPTIONS>>& ss_inertial_reading,
    const SCALAR                                                                                         ss_time)
{
  // Find Hamiltonian
  const Eigen::Matrix<SCALAR,2*DIM_S::ERROR_DIM,2*DIM_S::ERROR_DIM,OPTIONS> hamiltonian_mat =
    hamiltonianMatrix<DIM_S,SCALAR,OPTIONS>(dynamics,
                                            inertial_measurements,
                                            mappings,
                                            one_dim_measurements,
                                            two_dim_measurements,
                                            three_dim_measurements,
                                            six_dim_measurements,
                                            truth_process_noise_cov,
                                            inertial_noise_cov,
                                            ss_nav_state,
                                            ss_control_input,
                                            ss_inertial_reading,
                                            ss_time);
  // Find eigen decomposition
  const Eigen::EigenSolver<Eigen::Matrix<SCALAR,2*DIM_S::ERROR_DIM,2*DIM_S::ERROR_DIM,OPTIONS>> eigen_solver(hamiltonian_mat);

  assert(Eigen::ComputationInfo::Success == eigen_solver.info());
  //assert(hamiltonian_mat.isApprox(eigen_solver.eigenvectors() * eigen_solver.eigenvalues().asDiagonal() * eigen_solver.eigenvectors().inverse()));
  assert((eigen_solver.eigenvalues().real().array() != 0).all());
  #ifndef NDEBUG
  for(Eigen::Index eigenvalue_it = 0; eigenvalue_it < (2*DIM_S::ERROR_DIM); ++eigenvalue_it)
  {
    assert(1 == (eigen_solver.eigenvalues().array() == eigen_solver.eigenvalues()[eigenvalue_it]).sum());
  }
  #endif

  // Sort eigen vectors to have all positive eigen values first and in order and then all negative eigenvalues in the same order
  Eigen::Matrix<std::complex<SCALAR>,(2*DIM_S::ERROR_DIM)+1,2*DIM_S::ERROR_DIM,OPTIONS> sorting_mat;
  sorting_mat.template topRows<1>()                     = eigen_solver.eigenvalues();
  sorting_mat.template bottomRows<2*DIM_S::ERROR_DIM>() = eigen_solver.eigenvectors();

  auto col_wise_list = sorting_mat.colwise();
  std::sort(col_wise_list.begin(), col_wise_list.end(),
            [] (const auto& col_left, const auto& col_right)
            {
              const bool left_pos  = 0 < col_left[ 0].real();
              const bool right_pos = 0 < col_right[0].real();

              if(left_pos and not right_pos)
              {
                return true;
              }
              if(right_pos and not left_pos)
              {
                return false;
              }
              if(left_pos) // Both positive
              {
                if(col_left[0].real() == col_right[0].real())
                {
                  return col_left[0].imag() < col_right[0].imag();
                }
                return col_left[0].real() < col_right[0].real();
              }
              // Both negative
              if(col_right[0].real() == col_left[0].real())
              {
                return col_right[0].imag() < col_left[0].imag();
              }
              return col_right[0].real() < col_left[0].real();
            });

  #ifndef NDEBUG
  for(Eigen::Index eigenvalue_it = 0; eigenvalue_it < DIM_S::ERROR_DIM; ++eigenvalue_it)
  {
    assert(0 < sorting_mat(0, eigenvalue_it).real());
    assert(0 > sorting_mat(0, DIM_S::ERROR_DIM+eigenvalue_it).real());
    const std::complex<SCALAR> diff = sorting_mat(0, eigenvalue_it) + sorting_mat(0, DIM_S::ERROR_DIM+eigenvalue_it);
    assert(std::abs(diff.real()) < 1e-8);
    assert(std::abs(diff.imag()) < 1e-8);
  }
  #endif

  // Compute steady state covariance
  const Eigen::Matrix<std::complex<SCALAR>,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS> output =
    sorting_mat.template block<DIM_S::ERROR_DIM,DIM_S::ERROR_DIM>(DIM_S::ERROR_DIM+1, 0) *
    sorting_mat.template block<DIM_S::ERROR_DIM,DIM_S::ERROR_DIM>(1,                  0).fullPivHouseholderQr().inverse();

  assert((output.imag().array().abs() < 1e-8).all());
  //assert(output.isApprox(output.transpose(), 1e-4));

  return output.real();
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,2*DIM_S::ERROR_DIM,2*DIM_S::ERROR_DIM,OPTIONS>
  ss::hamiltonianMatrix(
    const dynamics::DynamicsBasePtr<           DIM_S,SCALAR,OPTIONS>&                                    dynamics,
    const sensors::InertialMeasurementBasePtr< DIM_S,SCALAR,OPTIONS>&                                    inertial_measurements,
    const map::MappingsBasePtr<                DIM_S,SCALAR,OPTIONS>&                                    mappings,
    const std::vector<MEASUREMENT_INFO_TUPLE<1,DIM_S,SCALAR,OPTIONS>>&                                   one_dim_measurements,
    const std::vector<MEASUREMENT_INFO_TUPLE<2,DIM_S,SCALAR,OPTIONS>>&                                   two_dim_measurements,
    const std::vector<MEASUREMENT_INFO_TUPLE<3,DIM_S,SCALAR,OPTIONS>>&                                   three_dim_measurements,
    const std::vector<MEASUREMENT_INFO_TUPLE<6,DIM_S,SCALAR,OPTIONS>>&                                   six_dim_measurements,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,DIM_S::TRUTH_NOISE_DIM,DIM_S::TRUTH_NOISE_DIM,OPTIONS>>& truth_process_noise_cov,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,  DIM_S::INER_MEAS_DIM,  OPTIONS>>& inertial_noise_cov,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::NAV_DIM,        OPTIONS>>& nav_state,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::CONTROL_DIM,    OPTIONS>>& control_input,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::INER_MEAS_DIM,  OPTIONS>>& inertial_reading,
    const SCALAR                                                                                         time)
{
  // Find needed helper variables
  const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS> truth_state = mappings->mapNavTruth(nav_state);

  const Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_NOISE_DIM,OPTIONS>    truth_dynamics_pd_noise =
    dynamics->getTruthStateDynamicsPDWRNoise(time, truth_state, control_input);
  const Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS>               nav_dynamics_pd_error_state =
    dynamics->getNavStateDynamicsPDWRErrorState(time, nav_state, Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,OPTIONS>::Constant(std::numeric_limits<SCALAR>::quiet_NaN()), inertial_reading);
  const Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,DIM_S::ERROR_DIM,OPTIONS>           nav_dynamics_pd_inertial_t =
    dynamics->getNavStateDynamicsPDWRInertialMeasurement(time, nav_state, Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,OPTIONS>::Constant(std::numeric_limits<SCALAR>::quiet_NaN()), inertial_reading);
  const Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS>      inertial_reading_pd_disp_state =
    inertial_measurements->getMeasurementPDWRDispersionState(time, truth_state, control_input);
  const Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,DIM_S::INER_MEAS_NOISE_DIM,OPTIONS> inertial_reading_pd_noise =
    inertial_measurements->getMeasurementPDWRNoise(time, truth_state, control_input);
  const Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS>          truth_nav_map_pd_disp_state =
    mappings->getTruthNavMapPDWRDispersionState(truth_state);
  const Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::ERROR_DIM,OPTIONS>          nav_truth_map_pd_error_state =
    mappings->getNavTruthMapPDWRErrorState(nav_state);

  // Find driving error covariance from measurement noise
  Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS> all_meas_noise_cov = Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS>::Zero();
  std::for_each(one_dim_measurements.cbegin(), one_dim_measurements.cend(),
                [&all_meas_noise_cov, time, &nav_state] (const MEASUREMENT_INFO_TUPLE<1,DIM_S,SCALAR,OPTIONS>& meas_info) -> void
                {
                  // Convert discreet noise to continuous
                  Eigen::Matrix<SCALAR,1,1,OPTIONS> continuous_meas_noise_cov = std::get<1>(meas_info);
                  if(0 != std::get<2>(meas_info)) // Not already continuous noise
                  {
                    assert(0 < std::get<2>(meas_info));
                    continuous_meas_noise_cov.array() *= std::get<2>(meas_info);
                  }
                  // Compute helpers
                  const Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS> meas_est_pd_error_state =
                    std::get<0>(meas_info)->getMeasurementEstimatePDWRErrorState(time, nav_state);
                  // Add noise contribution
                  all_meas_noise_cov.noalias() += meas_est_pd_error_state.transpose() *
                                                  continuous_meas_noise_cov.inverse() *
                                                  meas_est_pd_error_state;
                });
  std::for_each(two_dim_measurements.cbegin(), two_dim_measurements.cend(),
                [&all_meas_noise_cov, time, &nav_state] (const MEASUREMENT_INFO_TUPLE<2,DIM_S,SCALAR,OPTIONS>& meas_info) -> void
                {
                  // Convert discreet noise to continuous
                  Eigen::Matrix<SCALAR,2,2,OPTIONS> continuous_meas_noise_cov = std::get<1>(meas_info);
                  if(0 != std::get<2>(meas_info)) // Not already continuous noise
                  {
                    assert(0 < std::get<2>(meas_info));
                    continuous_meas_noise_cov.array() *= std::get<2>(meas_info);
                  }
                  // Compute helpers
                  const Eigen::Matrix<SCALAR,2,DIM_S::ERROR_DIM,OPTIONS> meas_est_pd_error_state =
                    std::get<0>(meas_info)->getMeasurementEstimatePDWRErrorState(time, nav_state);
                  // Add noise contribution
                  all_meas_noise_cov.noalias() += meas_est_pd_error_state.transpose() *
                                                  continuous_meas_noise_cov.inverse() *
                                                  meas_est_pd_error_state;
                });
  std::for_each(three_dim_measurements.cbegin(), three_dim_measurements.cend(),
                [&all_meas_noise_cov, time, &nav_state] (const MEASUREMENT_INFO_TUPLE<3,DIM_S,SCALAR,OPTIONS>& meas_info) -> void
                {
                  // Convert discreet noise to continuous
                  Eigen::Matrix<SCALAR,3,3,OPTIONS> continuous_meas_noise_cov = std::get<1>(meas_info);
                  if(0 != std::get<2>(meas_info)) // Not already continuous noise
                  {
                    assert(0 < std::get<2>(meas_info));
                    continuous_meas_noise_cov.array() *= std::get<2>(meas_info);
                  }
                  // Compute helpers
                  const Eigen::Matrix<SCALAR,3,DIM_S::ERROR_DIM,OPTIONS> meas_est_pd_error_state =
                    std::get<0>(meas_info)->getMeasurementEstimatePDWRErrorState(time, nav_state);
                  // Add noise contribution
                  all_meas_noise_cov.noalias() += meas_est_pd_error_state.transpose() *
                                                  continuous_meas_noise_cov.inverse() *
                                                  meas_est_pd_error_state;
                });
  std::for_each(six_dim_measurements.cbegin(), six_dim_measurements.cend(),
                [&all_meas_noise_cov, time, &nav_state] (const MEASUREMENT_INFO_TUPLE<6,DIM_S,SCALAR,OPTIONS>& meas_info) -> void
                {
                  // Convert discreet noise to continuous
                  Eigen::Matrix<SCALAR,6,6,OPTIONS> continuous_meas_noise_cov = std::get<1>(meas_info);
                  if(0 != std::get<2>(meas_info)) // Not already continuous noise
                  {
                    assert(0 < std::get<2>(meas_info));
                    continuous_meas_noise_cov.array() *= std::get<2>(meas_info);
                  }
                  // Compute helpers
                  const Eigen::Matrix<SCALAR,6,DIM_S::ERROR_DIM,OPTIONS> meas_est_pd_error_state =
                    std::get<0>(meas_info)->getMeasurementEstimatePDWRErrorState(time, nav_state);
                  // Add noise contribution
                  all_meas_noise_cov.noalias() += meas_est_pd_error_state.transpose() *
                                                  continuous_meas_noise_cov.inverse() *
                                                  meas_est_pd_error_state;
                });

  // Find linear error dynamics matrix
  const Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS> linear_error_dynamics =
    nav_dynamics_pd_error_state +
    (nav_dynamics_pd_inertial_t.transpose() * inertial_reading_pd_disp_state * nav_truth_map_pd_error_state);

  // Find process noise covariance
  const Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS> all_process_noise_cov =
    (truth_nav_map_pd_disp_state *
     truth_dynamics_pd_noise *
     truth_process_noise_cov *
     truth_dynamics_pd_noise.transpose() *
     truth_nav_map_pd_disp_state.transpose()) +
    (nav_dynamics_pd_inertial_t.transpose() *
     inertial_reading_pd_noise *
     inertial_noise_cov *
     inertial_reading_pd_noise.transpose() *
     nav_dynamics_pd_inertial_t);

  // Build Hamiltonian matrix
  Eigen::Matrix<SCALAR,2*DIM_S::ERROR_DIM,2*DIM_S::ERROR_DIM,OPTIONS> output;

  output.template block<DIM_S::ERROR_DIM,DIM_S::ERROR_DIM>(0,                0)                = -linear_error_dynamics.transpose();
  output.template block<DIM_S::ERROR_DIM,DIM_S::ERROR_DIM>(DIM_S::ERROR_DIM, 0)                = all_process_noise_cov;
  output.template block<DIM_S::ERROR_DIM,DIM_S::ERROR_DIM>(0,                DIM_S::ERROR_DIM) = all_meas_noise_cov;
  output.template block<DIM_S::ERROR_DIM,DIM_S::ERROR_DIM>(DIM_S::ERROR_DIM, DIM_S::ERROR_DIM) = linear_error_dynamics;

  return output;
}
} // namespace math
} // namespace kf

#endif
/* steady_state_covariance.hpp */
