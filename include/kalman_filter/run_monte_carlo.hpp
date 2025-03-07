/**
 * @File: run_monte_carlo.hpp
 * @Date: April 2022
 * @Author: James Swedeen
 *
 * @brief
 * Defines a function that runs one Monte Carlo simulation.
 **/

#ifndef KALMAN_FILTER_RUN_MONTE_CARLO_HPP
#define KALMAN_FILTER_RUN_MONTE_CARLO_HPP

/* C++ Headers */
#include<cstdint>
#include<functional>
#include<vector>
#include<atomic>
#include<execution>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<kalman_filter/helpers/versions.hpp>
#include<kalman_filter/helpers/integrator_step.hpp>
#include<kalman_filter/helpers/propagate_error_covariance.hpp>
#include<kalman_filter/helpers/tools.hpp>
#include<kalman_filter/noise/normal_distribution.hpp>

namespace kf
{
/**
 * @runMonteCarloSims
 *
 * @brief
 * The function that propagates the kalman equations forward in time along a nominal trajectory.
 *
 * @templates
 * DIM_S: The type of a Dimensions object or an inheriting object that has information
 *        about the size of the state vectors
 * SCALAR: The object type that each dimension will be represented with
 * VERSION: Controls what type of simulation will be ran
 * OPTIONS: Eigen Matrix options
 *
 * @parameters
 * reference_trajectory: The nominal reference trajectory for this set of simulations
 * initial_time: The time that the simulation starts at
 * first_measurement_time: The time at which the first measurement will be taken (used to prevent bad conditions early on in the simulation)
 * nominal_initial_truth_state: The starting truth state for the simulation without noise
 * initial_nav_state: The starting navigation state for the simulation
 * initial_truth_cov: The starting truth dispersion covariance for the simulation
 * number_sim: The number of simulations to run
 * tools: Holds all of the needed helper functions
 * sims_output: The output of the requested simulations
 **/
template<typename DIM_S, Versions VERSION, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
inline void runMonteCarloSims(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::REF_DIM,OPTIONS>>&               reference_trajectory,
                              const SCALAR                                                                                       initial_time,
                              const SCALAR                                                                                       first_measurement_time,
                              const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                    DIM_S::TRUTH_DIM,     OPTIONS>>& nominal_initial_truth_state,
                              const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                    DIM_S::NAV_DIM,       OPTIONS>>& initial_nav_state,
                              const Eigen::Ref<const Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS>>& initial_truth_cov,
                              const size_t                                                                                       number_sim,
                              const Tools<DIM_S,SCALAR,OPTIONS>&                                                                 tools,
                              std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::MC::FULL_STATE_LEN,OPTIONS>>&               sims_output);
/**
 * @runMonteCarlo
 *
 * @brief
 * The function that propagates the kalman equations forward in time along a nominal trajectory.
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
inline void runMonteCarlo(Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::MC::FULL_STATE_LEN,OPTIONS>& state_vector,
                          const Tools<DIM_S,SCALAR,OPTIONS>&                                      tools);
} // namespace kf

template<typename DIM_S, kf::Versions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void kf::runMonteCarloSims(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::REF_DIM,OPTIONS>>&               reference_trajectory,
                                  const SCALAR                                                                                       initial_time,
                                  const SCALAR                                                                                       first_measurement_time,
                                  const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                    DIM_S::TRUTH_DIM,OPTIONS>>&      nominal_initial_truth_state,
                                  const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                    DIM_S::NAV_DIM,  OPTIONS>>&      initial_nav_state,
                                  const Eigen::Ref<const Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS>>& initial_truth_cov,
                                  const size_t                                                                                       number_sim,
                                  const Tools<DIM_S,SCALAR,OPTIONS>&                                                                 tools,
                                  std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::MC::FULL_STATE_LEN,OPTIONS>>&               sims_output)
{
  std::atomic<unsigned> rand_seed(0);

  const std::function<void(Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::MC::FULL_STATE_LEN,OPTIONS>&)> run_one_sim_func =
    [&reference_trajectory, initial_time, first_measurement_time, &nominal_initial_truth_state, &initial_nav_state, &initial_truth_cov, &tools, &rand_seed]
    (Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::MC::FULL_STATE_LEN,OPTIONS>& state_vector)
    {
      state_vector.resize(reference_trajectory.rows(), Eigen::NoChange);
      // Set starting states
      state_vector.template topRows<1>()[DIM_S::TIME_IND] = initial_time;
      state_vector.template topRows<1>().template middleCols<DIM_S::NUM_MEAS_DIM>(DIM_S::NUM_MEAS_START_IND).setConstant(first_measurement_time);

      Eigen::Map<Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS>> error_cov(
        state_vector.template topRows<1>().template middleCols<DIM_S::ERROR_COV_LEN>(DIM_S::ERROR_COV_START_IND).data());
      const Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS> truth_nav_pdwr_disp =
        tools.mappings->getTruthNavMapPDWRDispersionState(nominal_initial_truth_state);
      error_cov = truth_nav_pdwr_disp * initial_truth_cov * truth_nav_pdwr_disp.transpose();

      state_vector.template topRows<1>().template middleCols<DIM_S::TRUTH_DIM>(DIM_S::MC::TRUTH_START_IND) = nominal_initial_truth_state;
      state_vector.template topRows<1>().template middleCols<DIM_S::NAV_DIM>(  DIM_S::MC::NAV_START_IND)   = initial_nav_state;

      // Set reference trajectory
      state_vector.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND) = reference_trajectory;
      // Add starting noise
      kf::noise::NormalDistribution<DIM_S::TRUTH_DISP_DIM,true,true,false,SCALAR,OPTIONS> init_noise(
        Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DISP_DIM,OPTIONS>::Zero(),
        initial_truth_cov,
        rand_seed.fetch_add(1));

      state_vector.template topRows<1>().template middleCols<DIM_S::TRUTH_DIM>(DIM_S::MC::TRUTH_START_IND) =
        tools.mappings->injectErrors(state_vector.template topRows<1>().template middleCols<DIM_S::TRUTH_DIM>(DIM_S::MC::TRUTH_START_IND),
                                     init_noise.getNoise());

      // Run simulation
      runMonteCarlo<DIM_S,VERSION,SCALAR,OPTIONS>(state_vector, tools);
     };

  // Get ready to run
  sims_output.resize(number_sim);
  // Run
  std::for_each(std::execution::par_unseq, sims_output.begin(), sims_output.end(), run_one_sim_func);
}

template<typename DIM_S, kf::Versions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void kf::runMonteCarlo(Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::MC::FULL_STATE_LEN,OPTIONS>& state_vector,
                              const Tools<DIM_S,SCALAR,OPTIONS>&                                      tools)
{
  monteCarloValid<VERSION>();

  // Constant noise covariances
  const Eigen::Matrix<SCALAR,DIM_S::TRUTH_NOISE_DIM,DIM_S::TRUTH_NOISE_DIM,OPTIONS> truth_process_noise_cov =
    tools.truth_process_noise->getCovariance();
  const Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_NOISE_DIM,DIM_S::INER_MEAS_NOISE_DIM,OPTIONS> inertial_measurement_noise_cov =
    tools.inertial_measurements_noise->getCovariance();

  const Eigen::Index state_vector_len = state_vector.rows();
  for(Eigen::Index row_it = 1; row_it < state_vector_len; ++row_it)
  {
    // Extract important vectors
    const SCALAR prev_time = state_vector(row_it-1, DIM_S::TIME_IND);

    Eigen::Map<const Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS>> prev_error_covariance(state_vector.template block<1,DIM_S::ERROR_COV_LEN>(row_it-1, DIM_S::ERROR_COV_START_IND).data());
    Eigen::Map<      Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS>> cur_error_covariance( state_vector.template block<1,DIM_S::ERROR_COV_LEN>(row_it, DIM_S::ERROR_COV_START_IND).data());

    // Find time step
    const SCALAR time_step =
      tools.dynamics->findTimeStep(state_vector.template block<1,DIM_S::REF_DIM>(row_it-1, DIM_S::REF_START_IND),
                                   state_vector.template block<1,DIM_S::REF_DIM>(row_it,   DIM_S::REF_START_IND));
    if(0 == time_step)
    {
      // Copy the old state
      state_vector.row(row_it) = state_vector.row(row_it-1);
      continue;
    }
    // Set current time
    const SCALAR cur_time = prev_time + time_step;
    state_vector(row_it, DIM_S::TIME_IND) = cur_time;
    // Find the control input from the previous time step
    Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS> control_vec;
    if constexpr(openLoop(VERSION))
    {
      control_vec.setZero();
    }
    else // Not open loop
    {
      control_vec =
        tools.controller->getControl(cur_time,
                                     state_vector.template block<1,DIM_S::NAV_DIM>(row_it-1, DIM_S::MC::NAV_START_IND),
                                     state_vector.template block<1,DIM_S::REF_DIM>(row_it-1, DIM_S::REF_START_IND),
                                     state_vector.template block<1,DIM_S::REF_DIM>(row_it,   DIM_S::REF_START_IND),
                                     time_step);
    }
    // Propagate truth state
    const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_NOISE_DIM,OPTIONS> process_noise = tools.truth_process_noise->getContinuousNoise(time_step);

    state_vector.template block<1,DIM_S::TRUTH_DIM>(row_it, DIM_S::MC::TRUTH_START_IND) =
      integratorStep<VERSION,SCALAR,Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>(
        std::bind(&dynamics::DynamicsBase<DIM_S,SCALAR,OPTIONS>::getTruthStateDynamics,
                  tools.dynamics.get(),
                  cur_time,
                  std::placeholders::_1,
                  control_vec,
                  process_noise),
        state_vector.template block<1,DIM_S::TRUTH_DIM>(row_it-1, DIM_S::MC::TRUTH_START_IND),
        time_step);
    // Calculate inertial measurements
    const Eigen::Matrix<SCALAR,1,DIM_S::INER_MEAS_NOISE_DIM,OPTIONS> inertial_measurements_noise =
      tools.inertial_measurements_noise->getContinuousNoise(time_step);
    const Eigen::Matrix<SCALAR,1,DIM_S::INER_MEAS_DIM,OPTIONS> inertial_measurements =
      tools.inertial_measurements->getMeasurement(cur_time,
                                                  state_vector.template block<1,DIM_S::TRUTH_DIM>(row_it, DIM_S::MC::TRUTH_START_IND),
                                                  control_vec,
                                                  inertial_measurements_noise);
    // Propagate navigation state
    state_vector.template block<1,DIM_S::NAV_DIM>(row_it, DIM_S::MC::NAV_START_IND) =
      integratorStep<VERSION,SCALAR,Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>(
        std::bind(&dynamics::DynamicsBase<DIM_S,SCALAR,OPTIONS>::getNavStateDynamics,
                  tools.dynamics.get(),
                  cur_time,
                  std::placeholders::_1,
                  state_vector.template block<1,DIM_S::REF_DIM>(row_it-1, DIM_S::REF_START_IND),
                  inertial_measurements),
        state_vector.template block<1,DIM_S::NAV_DIM>(row_it-1, DIM_S::MC::NAV_START_IND),
        time_step);

    // Propagate error state covariance
    if constexpr(not DIM_S::USE_STEADY_STATE_ERROR_COV)
    {
      cur_error_covariance =
        propagateErrorCovariance<DIM_S,VERSION,SCALAR,OPTIONS>(prev_error_covariance,
                                                               tools.dynamics,
                                                               tools.inertial_measurements,
                                                               tools.mappings,
                                                               time_step,
                                                               cur_time,
                                                               state_vector.template block<1,DIM_S::NAV_DIM>(row_it-1, DIM_S::MC::NAV_START_IND),
                                                               state_vector.template block<1,DIM_S::REF_DIM>(row_it-1, DIM_S::REF_START_IND),
                                                               control_vec,
                                                               inertial_measurements,
                                                               truth_process_noise_cov,
                                                               inertial_measurement_noise_cov);
    }
    // Handle discreet measurement updates
    if constexpr(DIM_S::USE_STEADY_STATE_ERROR_COV)
    {
      state_vector.template block<1,DIM_S::NUM_MEAS_DIM>(row_it, DIM_S::NUM_MEAS_START_IND) =
        tools.measurement_controller->applyMeasurementsSSErrorCov(tools.mappings,
                                                                  cur_time,
                                                                  state_vector.template block<1,DIM_S::NUM_MEAS_DIM>(row_it-1, DIM_S::NUM_MEAS_START_IND),
                                                                  state_vector.template block<1,DIM_S::TRUTH_DIM>(   row_it,   DIM_S::MC::TRUTH_START_IND),
                                                                  tools->ss_error_cov,
                                                                  state_vector.template block<1,DIM_S::NAV_DIM>(     row_it,   DIM_S::MC::NAV_START_IND));
    }
    else // Don't use steady state error covariance
    {
      state_vector.template block<1,DIM_S::NUM_MEAS_DIM>(row_it, DIM_S::NUM_MEAS_START_IND) =
        tools.measurement_controller->applyMeasurements(tools.mappings,
                                                        cur_time,
                                                        state_vector.template block<1,DIM_S::NUM_MEAS_DIM>(row_it-1, DIM_S::NUM_MEAS_START_IND),
                                                        state_vector.template block<1,DIM_S::TRUTH_DIM>(   row_it,   DIM_S::MC::TRUTH_START_IND),
                                                        state_vector.template block<1,DIM_S::NAV_DIM>(     row_it,   DIM_S::MC::NAV_START_IND),
                                                        cur_error_covariance);
    }
    // Apply state bounding if needed
    if constexpr(applyStateBounds(VERSION))
    {
      auto bounded_states = state_vector.template block<1,DIM_S::NAV_DIM>(row_it, DIM_S::MC::NAV_START_IND)(tools.bounded_indexes);

      bounded_states = (bounded_states.array() > tools.upper_bound.array()).select(tools.upper_bound, bounded_states);
      bounded_states = (bounded_states.array() < tools.lower_bound.array()).select(tools.lower_bound, bounded_states);
    }
  }
}

#endif
/* run_monte_carlo.hpp */
