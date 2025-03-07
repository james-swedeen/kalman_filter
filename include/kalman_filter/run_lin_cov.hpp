/**
 * @File: run_lin_cov.hpp
 * @Date: May 2022
 * @Author: James Swedeen
 *
 * @brief
 * Defines a function that runs one Linear Covariance simulation.
 **/

#ifndef KALMAN_FILTER_RUN_LIN_COV_HPP
#define KALMAN_FILTER_RUN_LIN_COV_HPP

/* C++ Headers */
#include<cstdint>
#include<functional>
#include<vector>
#include<limits>
#include<execution>

/* Boost Headers */
#include<boost/range/irange.hpp>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<kalman_filter/helpers/versions.hpp>
#include<kalman_filter/helpers/integrator_step.hpp>
#include<kalman_filter/helpers/propagate_error_covariance.hpp>
#include<kalman_filter/helpers/propagate_augmented_covariance.hpp>
#include<kalman_filter/helpers/tools.hpp>

namespace kf
{
/**
 * @runLinCov
 *
 * @brief
 * The function that propagates the linearized kalman equations forward in time along a nominal trajectory.
 *
 * @templates
 * DIM_S: The type of a Dimensions object or an inheriting object that has information
 *        about the size of the state vectors
 * VERSION: Controls what type of simulation will be ran
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 *
 * @parameters
 * reference_trajectory: The nominal reference trajectory for this set of simulations
 * initial_time: The time that the simulation starts at
 * first_measurement_time: The time at which the first measurement will be taken (used to prevent bad conditions early on in the simulation)
 * initial_truth_cov: The starting truth dispersion state covariance for the simulation
 * tools: Holds all of the needed helper functions
 * output: The resulting propagated error and augmented state covariances
 **/
template<typename DIM_S, Versions VERSION, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
inline void
  runLinCov(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::REF_DIM,OPTIONS>>&               reference_trajectory,
            const SCALAR                                                                                       initial_time,
            const SCALAR                                                                                       first_measurement_time,
            const Eigen::Ref<const Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS>>& initial_truth_cov,
            const Tools<DIM_S,SCALAR,OPTIONS>&                                                                 tools,
            Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>&                        state_vector);
/**
 * @runLinCov
 *
 * @brief
 * The function that propagates the linearized kalman equations forward in time along a nominal trajectory.
 *
 * @templates
 * DIM_S: The type of a Dimensions object or an inheriting object that has information
 *        about the size of the state vectors
 * VERSION: Controls what type of simulation will be ran
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 *
 * @parameters
 * state_vector: The vector that holds all of the simulations states. See dimension_struct.hpp
 *               It is assumed that the first state and the reference trajectory is already set
 * tools: Holds all of the needed helper functions
 **/
template<typename DIM_S, Versions VERSION, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
inline void
  runLinCov(Eigen::Ref<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>> state_vector,
            const Tools<DIM_S,SCALAR,OPTIONS>&                                                     tools);
} // namespace kf

template<typename DIM_S, kf::Versions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void
  kf::runLinCov(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::REF_DIM,OPTIONS>>&               reference_trajectory,
                const SCALAR                                                                                       initial_time,
                const SCALAR                                                                                       first_measurement_time,
                const Eigen::Ref<const Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS>>& initial_truth_cov,
                const Tools<DIM_S,SCALAR,OPTIONS>&                                                                 tools,
                Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>&                        state_vector)
{
  state_vector.resize(reference_trajectory.rows(), Eigen::NoChange);
  // Set starting states
  state_vector.template topRows<1>()[DIM_S::TIME_IND] = initial_time;
  state_vector.template topRows<1>().template middleCols<DIM_S::NUM_MEAS_DIM>(DIM_S::NUM_MEAS_START_IND).setConstant(first_measurement_time);

  Eigen::Map<Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS>> error_cov(
    state_vector.template topRows<1>().template middleCols<DIM_S::ERROR_COV_LEN>(DIM_S::ERROR_COV_START_IND).data());
  const Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS> truth_nav_pdwr_disp =
    tools.mappings->getTruthNavMapPDWRDispersionState(tools.mappings->mapRefTruth(reference_trajectory.template topRows<1>()));
  error_cov = truth_nav_pdwr_disp * initial_truth_cov * truth_nav_pdwr_disp.transpose();

  Eigen::Map<Eigen::Matrix<double,DIM_S::LINCOV::AUG_DIM,DIM_S::LINCOV::AUG_DIM,Eigen::RowMajor>> initial_aug_covariance(
    state_vector.template block<1,DIM_S::LINCOV::AUG_COV_LEN>(0, DIM_S::LINCOV::AUG_COV_START_IND).data());
  initial_aug_covariance.setZero();
  initial_aug_covariance.template block<DIM_S::TRUTH_DISP_DIM,
                                        DIM_S::TRUTH_DISP_DIM>(DIM_S::LINCOV::TRUTH_DISP_START_IND,
                                                               DIM_S::LINCOV::TRUTH_DISP_START_IND) = initial_truth_cov;
  // Set reference trajectory
  state_vector.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND) = reference_trajectory;

  // Run LinCov
  runLinCov<DIM_S,VERSION,SCALAR,OPTIONS>(state_vector, tools);
}

template<typename DIM_S, kf::Versions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void
  kf::runLinCov(Eigen::Ref<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>> state_vector,
                const Tools<DIM_S,SCALAR,OPTIONS>&                                                     tools)
{
  linCovValid<VERSION>();

  const Eigen::Index state_vector_len = state_vector.rows();

  // Constant noise covariances
  const Eigen::Matrix<SCALAR,DIM_S::TRUTH_NOISE_DIM,DIM_S::TRUTH_NOISE_DIM,OPTIONS> truth_process_noise_cov =
    tools.truth_process_noise->getCovariance();
  const Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_NOISE_DIM,DIM_S::INER_MEAS_NOISE_DIM,OPTIONS> inertial_measurement_noise_cov =
    tools.inertial_measurements_noise->getCovariance();

  // Helper vectors
  Eigen::Matrix<SCALAR,Eigen::Dynamic,      DIM_S::TRUTH_DIM,OPTIONS> ref_truth_state(      state_vector_len,     DIM_S::TRUTH_DIM);
  Eigen::Matrix<SCALAR,Eigen::Dynamic,      DIM_S::NAV_DIM,  OPTIONS> ref_nav_state(        state_vector_len,     DIM_S::NAV_DIM);
  Eigen::Matrix<SCALAR,1,                   Eigen::Dynamic,  OPTIONS> time_step_vec(        1,                    state_vector_len);
  Eigen::Matrix<SCALAR,DIM_S::CONTROL_DIM,  Eigen::Dynamic,  OPTIONS> control_traj(         DIM_S::CONTROL_DIM,   state_vector_len);
  Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,Eigen::Dynamic,  OPTIONS> inertial_measurements(DIM_S::INER_MEAS_DIM, state_vector_len);

  {
    const boost::integer_range<Eigen::Index> state_vector_inds(0, state_vector_len);

    std::for_each(std::execution::unseq, state_vector_inds.begin(), state_vector_inds.end(),
                  [&state_vector, &tools, &ref_truth_state, &ref_nav_state] (const Eigen::Index ind) -> void
                  {
                    ref_truth_state.row(ind) =
                      tools.mappings->mapRefTruth(state_vector.template block<1,DIM_S::REF_DIM>(ind, DIM_S::REF_START_IND));
                    ref_nav_state.row(ind) =
                      tools.mappings->mapRefNav(state_vector.template block<1,DIM_S::REF_DIM>(ind, DIM_S::REF_START_IND));
                  });
    #ifndef NDEBUG
    time_step_vec[0] = std::numeric_limits<SCALAR>::quiet_NaN();
    control_traj.         col(0).setConstant(std::numeric_limits<SCALAR>::quiet_NaN());
    inertial_measurements.col(0).setConstant(std::numeric_limits<SCALAR>::quiet_NaN());
    #endif
    // Set time steps
    std::for_each(std::execution::unseq, std::next(state_vector_inds.begin()), state_vector_inds.end(),
                  [&state_vector, &tools, &time_step_vec] (const Eigen::Index ind) -> void
                  {
                    time_step_vec[ind] =
                      tools.dynamics->findTimeStep(state_vector.template block<1,DIM_S::REF_DIM>(ind-1, DIM_S::REF_START_IND),
                                                   state_vector.template block<1,DIM_S::REF_DIM>(ind,   DIM_S::REF_START_IND));
                  });
    // Set time
    std::for_each(std::next(state_vector_inds.begin()), state_vector_inds.end(),
                  [&state_vector, &tools, &time_step_vec] (const Eigen::Index ind) -> void
                  {
                    state_vector(ind, DIM_S::TIME_IND) = state_vector(ind-1, DIM_S::TIME_IND) + time_step_vec[ind];
                  });
    // Set control and inertial measurements
    std::for_each(std::execution::par_unseq, std::next(state_vector_inds.begin()), state_vector_inds.end(),
                  [&state_vector, &tools, &ref_nav_state, &ref_truth_state, &control_traj, &inertial_measurements, &time_step_vec] (const Eigen::Index ind) -> void
                  {
                    if constexpr(openLoop(VERSION))
                    {
                      control_traj.col(ind).setZero();
                    }
                    else // Not open loop
                    {
                      control_traj.col(ind) =
                        tools.controller->getControl(state_vector(ind, DIM_S::TIME_IND),
                                                     ref_nav_state.row(ind-1),
                                                     state_vector.template block<1,DIM_S::REF_DIM>(ind-1, DIM_S::REF_START_IND),
                                                     state_vector.template block<1,DIM_S::REF_DIM>(ind,   DIM_S::REF_START_IND),
                                                     time_step_vec[ind]);
                    }
                    inertial_measurements.col(ind) =
                      tools.inertial_measurements->getMeasurement(state_vector(ind, DIM_S::TIME_IND),
                                                                  ref_truth_state.row(ind),
                                                                  control_traj.col(ind),
                                                                  Eigen::Matrix<SCALAR,1,DIM_S::INER_MEAS_NOISE_DIM,OPTIONS>::Zero());
                  });
  }

  // Propagate covariance
  for(Eigen::Index row_it = 1; row_it < state_vector_len; ++row_it)
  {
    // Extract important vectors
    Eigen::Map<const Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS>> prev_error_cov(state_vector.template block<1,DIM_S::ERROR_COV_LEN>(row_it-1, DIM_S::ERROR_COV_START_IND).data());
    Eigen::Map<      Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS>> cur_error_cov( state_vector.template block<1,DIM_S::ERROR_COV_LEN>(row_it,   DIM_S::ERROR_COV_START_IND).data());

    const Eigen::Map<const Eigen::Matrix<SCALAR,DIM_S::LINCOV::AUG_DIM,DIM_S::LINCOV::AUG_DIM,OPTIONS>> prev_aug_cov(
      state_vector.template block<1,DIM_S::LINCOV::AUG_COV_LEN>(row_it-1, DIM_S::LINCOV::AUG_COV_START_IND).data());
          Eigen::Map<      Eigen::Matrix<SCALAR,DIM_S::LINCOV::AUG_DIM,DIM_S::LINCOV::AUG_DIM,OPTIONS>> cur_aug_cov(
      state_vector.template block<1,DIM_S::LINCOV::AUG_COV_LEN>(row_it, DIM_S::LINCOV::AUG_COV_START_IND).data());

    // Edge case condition
    if(0 == time_step_vec[row_it])
    {
      // Copy the old state
      state_vector.row(row_it) = state_vector.row(row_it-1);
      continue;
    }

    // Propagate error state covariance
    if constexpr((not runningErrorBudget(VERSION)) and (not DIM_S::USE_STEADY_STATE_ERROR_COV))
    {
      cur_error_cov = propagateErrorCovariance<DIM_S,VERSION,SCALAR,OPTIONS>(prev_error_cov,
                                                                             tools.dynamics,
                                                                             tools.inertial_measurements,
                                                                             tools.mappings,
                                                                             time_step_vec[row_it],
                                                                             state_vector(row_it, DIM_S::TIME_IND),
                                                                             ref_nav_state.row(row_it-1),
                                                                             state_vector.template block<1,DIM_S::REF_DIM>(row_it-1, DIM_S::REF_START_IND),
                                                                             control_traj.col(row_it).transpose(),
                                                                             inertial_measurements.col(row_it).transpose(),
                                                                             truth_process_noise_cov,
                                                                             inertial_measurement_noise_cov);
    }
    // Propagate augmented state covariance
    cur_aug_cov = propagateAugCovariance<DIM_S,VERSION,SCALAR,OPTIONS>(prev_aug_cov,
                                                                       tools.dynamics,
                                                                       tools.controller,
                                                                       tools.inertial_measurements,
                                                                       time_step_vec[row_it],
                                                                       state_vector(row_it, DIM_S::TIME_IND),
                                                                       state_vector.template block<1,DIM_S::REF_DIM>(row_it-1, DIM_S::REF_START_IND),
                                                                       ref_truth_state.row(row_it-1),
                                                                       ref_nav_state.row(row_it-1),
                                                                       control_traj.col(row_it).transpose(),
                                                                       inertial_measurements.col(row_it).transpose(),
                                                                       truth_process_noise_cov,
                                                                       inertial_measurement_noise_cov);

    // Handle discreet measurement updates
    if constexpr(runningErrorBudget(VERSION))
    {
      state_vector.template block<1,DIM_S::NUM_MEAS_DIM>(row_it, DIM_S::NUM_MEAS_START_IND) =
        tools.measurement_controller->applyMeasurementsErrorBudget(state_vector(row_it, DIM_S::TIME_IND),
                                                                   state_vector.template block<1,DIM_S::NUM_MEAS_DIM>(row_it-1, DIM_S::NUM_MEAS_START_IND),
                                                                   ref_truth_state.row(row_it),
                                                                   ref_nav_state.row(row_it),
                                                                   cur_error_cov,
                                                                   cur_aug_cov);
    }
    else if constexpr(DIM_S::USE_STEADY_STATE_ERROR_COV)
    {
      state_vector.template block<1,DIM_S::NUM_MEAS_DIM>(row_it, DIM_S::NUM_MEAS_START_IND) =
        tools.measurement_controller->applyMeasurementsErrorBudget(state_vector(row_it, DIM_S::TIME_IND),
                                                                   state_vector.template block<1,DIM_S::NUM_MEAS_DIM>(row_it-1, DIM_S::NUM_MEAS_START_IND),
                                                                   ref_truth_state.row(row_it),
                                                                   ref_nav_state.row(row_it),
                                                                   tools.ss_error_cov,
                                                                   cur_aug_cov);
    }
    else // Not running error budget or using steady state error covariance
    {
      state_vector.template block<1,DIM_S::NUM_MEAS_DIM>(row_it, DIM_S::NUM_MEAS_START_IND) =
        tools.measurement_controller->applyMeasurementsLinCov(state_vector(row_it, DIM_S::TIME_IND),
                                                              state_vector.template block<1,DIM_S::NUM_MEAS_DIM>(row_it-1, DIM_S::NUM_MEAS_START_IND),
                                                              ref_truth_state.row(row_it),
                                                              ref_nav_state.row(row_it),
                                                              cur_error_cov,
                                                              cur_aug_cov);
    }
  }
}

#endif
/* run_lin_cov.hpp */
