/**
 * @File: rocket_sled_demo.cpp
 * @Author: James Swedeen
 * @Date: March 2023
 *
 * @brief
 * A demonstration some of the primary uses of this package. Notably, performing Monte Carlo simulations and running
 * Lincov analysis.
 **/

/* C++ Headers */
#include<memory>
#include<vector>
#include<iostream>
#include<utility>

/* Local Headers */
#include<kalman_filter/dynamics/one_dim_integrator.hpp>
#include<kalman_filter/sensors/inertial_measurements/one_dim_integrator_accelerometer.hpp>
#include<kalman_filter/mappings/linear_mapping.hpp>
#include<kalman_filter/controllers/one_dim_integrator_controller.hpp>
#include<kalman_filter/sensors/measurements/one_dim_fix.hpp>
#include<kalman_filter/sensors/measurements/controllers/one_dim_integrator_measurement_controller.hpp>
#include<kalman_filter/run_lin_cov.hpp>
#include<kalman_filter/run_monte_carlo.hpp>
#include<kalman_filter/math/performance_evaluation.hpp>
#include<kalman_filter/noise/normal_distribution.hpp>
#include<kalman_filter/helpers/plot_statistics.hpp>

int main()
{
  using DIM_S = kf::dynamics::OneDimIntegratorDim;

  /// Declare simulation variables
  const double       start_time      = 0;
  const double       end_time        = 150;
  const double       end_point       = 25;
  const double       nom_vel         = end_point/end_time;
  const double       sim_dt          = 0.1;
  const double       first_meas_time = 0.1;
  const Eigen::Index sim_len         = end_time/sim_dt;
  const size_t       num_mc_sims     = 250;

  /// Declare noise parameters
  const double wind_std                      = double(1e-8)/double(3);              // Meters
  const double dist_accelerations_ss_std     = double(0.01)/double(3);              // Meters/Second^2
  const double dist_acceleration_time_const  = 20;                                  // Seconds
  const double accel_bias_ss_std             = double(900*(1e-6)*9.81)/double(3);   // Meters/Second^2
  const double accel_bias_time_const         = 60;                                  // Seconds
  const double accel_scale_factor_ss_std     = double(900*(1e-6))/double(3);        // Unitless
  const double accel_scale_factor_time_const = 60;                                  // Seconds
  const double actuator_bias_ss_std          = double(0.03)/double(3);              // Meters/Second^2
  const double actuator_bias_time_const      = 60;                                  // Seconds
  const double accel_measurement_std         = double(150*(1e-6)*9.81)/double(3);   // Meters/Second
  const double pos_measument_std             = double(0.1)/double(3);               // Meters/Second
  const double pos_samp_rate                 = 1;                                   // Seconds
  const double vel_measument_std             = double(0.01)/double(3);              // Meters/Second
  const double vel_samp_rate                 = 0.5;                                 // Seconds

  /// Put noise parameters in units of variance
  const double wind_var               = std::pow(wind_std, 2);
  const double dist_accelerations_var = (double(2)*std::pow(dist_accelerations_ss_std, 2))/dist_acceleration_time_const;
  const double accel_bias_var         = (double(2)*std::pow(accel_bias_ss_std,         2))/accel_bias_time_const;
  const double accel_scale_factor_var = (double(2)*std::pow(accel_scale_factor_ss_std, 2))/accel_scale_factor_time_const;
  const double actuator_bias_var      = (double(2)*std::pow(actuator_bias_ss_std,      2))/actuator_bias_time_const;
  const double accel_measurement_var  = std::pow(accel_measurement_std, 2);
  const double pos_measument_var      = std::pow(pos_measument_std,     2);
  const double vel_measument_var      = std::pow(vel_measument_std,     2);

  /// Make Kalman filter helper objects
  kf::Tools<DIM_S> tools;
  // Dynamics
  tools.dynamics = std::make_shared<kf::dynamics::OneDimIntegrator<>>(sim_dt,
                                                                      dist_acceleration_time_const,
                                                                      accel_bias_time_const,
                                                                      accel_scale_factor_time_const,
                                                                      actuator_bias_time_const);
  // Process noise
  tools.truth_process_noise = std::make_shared<kf::noise::NormalDistribution<5,true,true,false>>(
                                Eigen::Matrix<double,1,5,Eigen::RowMajor>::Zero(),
                                Eigen::Matrix<double,1,5,Eigen::RowMajor>({wind_var,
                                                                           dist_accelerations_var,
                                                                           accel_bias_var,
                                                                           accel_scale_factor_var,
                                                                           actuator_bias_var}).matrix().asDiagonal());
  // Inertial measurements
  tools.inertial_measurements = std::make_shared<kf::sensors::OneDimIntegratorAccelerometer<>>();
  // Inertial measurement noise
  tools.inertial_measurements_noise = std::make_shared<kf::noise::NormalDistribution<1,true,true,false>>(
                                       Eigen::Matrix<double,1,1,Eigen::RowMajor>::Zero(),
                                       Eigen::Matrix<double,1,1,Eigen::RowMajor>(accel_measurement_var));
  // Discrete measurements
  tools.measurement_controller = std::make_shared<kf::sensors::OneDimIntegratorMeasurementController<>>(
                                   std::make_shared<kf::sensors::OneDimFix<DIM_S::NAV::POS_IND,DIM_S>>(
                                     pos_samp_rate),
                                   std::make_shared<kf::noise::NormalDistribution<1,true,true,false>>(
                                     Eigen::Matrix<double,1,1,Eigen::RowMajor>::Zero(),
                                     Eigen::Matrix<double,1,1,Eigen::RowMajor>(pos_measument_var)),
                                   std::make_shared<kf::sensors::OneDimFix<DIM_S::NAV::VEL_IND,DIM_S>>(
                                     vel_samp_rate),
                                   std::make_shared<kf::noise::NormalDistribution<1,true,true,false>>(
                                     Eigen::Matrix<double,1,1,Eigen::RowMajor>::Zero(),
                                     Eigen::Matrix<double,1,1,Eigen::RowMajor>(vel_measument_var)));
  // State mappings
  tools.mappings = std::make_shared<kf::map::LinearMapping<DIM_S>>();
  // Controller
  tools.controller = std::make_shared<kf::control::OneDimIntegratorController<>>(
                      Eigen::Matrix<double,1,6,Eigen::RowMajor>({3,4,1,0,0,1}));

  /// Setup initial simulation state
  // Make reference trajectory
  Eigen::Matrix<double,Eigen::Dynamic,2,Eigen::RowMajor> reference_trajectory(sim_len, 2);
  reference_trajectory.col(0).setLinSpaced(0, end_point);
  reference_trajectory.col(1).setConstant(nom_vel);

  // Make starting state
  Eigen::Matrix<double,1,DIM_S::TRUTH_DIM,Eigen::RowMajor> init_truth_state;
  Eigen::Matrix<double,1,DIM_S::NAV_DIM,  Eigen::RowMajor> init_nav_state;

  init_truth_state.setZero();
  init_nav_state.  setZero();

  // Make starting covariance
  const Eigen::Matrix<double,DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_DISP_DIM,Eigen::RowMajor> init_truth_cov =
    Eigen::Matrix<double,DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_DISP_DIM,Eigen::RowMajor>::Zero();

  // Run Monte Carlo simulation
  std::vector<Eigen::Matrix<double,Eigen::Dynamic,DIM_S::MC::FULL_STATE_LEN,Eigen::RowMajor>> mc_output;

  std::cout << "Running Monte Carlo simulation" << std::endl;
  std::chrono::high_resolution_clock::time_point start_timing = std::chrono::high_resolution_clock::now();

  kf::runMonteCarloSims<DIM_S,
                        kf::Versions(kf::Versions::MODEL_REPLACEMENT bitor kf::Versions::RK4_INTEGRATION),
                        double,
                        Eigen::RowMajor>(reference_trajectory,
                                         start_time,
                                         first_meas_time,
                                         init_truth_state,
                                         init_nav_state,
                                         init_truth_cov,
                                         num_mc_sims,
                                         tools,
                                         mc_output);

  std::chrono::high_resolution_clock::time_point stop_timing = std::chrono::high_resolution_clock::now();
  std::cout << "MC simulation took "
            << std::chrono::duration_cast<std::chrono::milliseconds>(stop_timing - start_timing).count()
            << " milliseconds"
            << std::endl;

  // Run LinCov
  Eigen::Matrix<double,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor> lc_output;

  std::cout << "Running LinCov" << std::endl;
  start_timing = std::chrono::high_resolution_clock::now();

  kf::runLinCov<DIM_S,
                kf::Versions(kf::Versions::MODEL_REPLACEMENT bitor kf::Versions::RK4_INTEGRATION),
                double,
                Eigen::RowMajor>(reference_trajectory,
                                 start_time,
                                 first_meas_time,
                                 init_truth_cov,
                                 tools,
                                 lc_output);

  stop_timing = std::chrono::high_resolution_clock::now();
  std::cout << "LC simulation took "
            << std::chrono::duration_cast<std::chrono::milliseconds>(stop_timing - start_timing).count()
            << " milliseconds" << std::endl;

  /// Make plots of Monte Carlo and LinCov results
  std::cout << "Plotting MC and LC results" << std::endl;

  const auto plot_map_func = [](const Eigen::Matrix<double,Eigen::Dynamic,DIM_S::TRUTH_DIM,Eigen::RowMajor>& input) ->
                             Eigen::Matrix<double,Eigen::Dynamic,DIM_S::TRUTH_DISP_DIM,Eigen::RowMajor>
    {
      return input;
    };

  kf::plot::plotAllStatistics<DIM_S,double,Eigen::RowMajor>(
      mc_output,
      lc_output,
      tools.mappings,
      plot_map_func,
      plot_map_func,
      {
        std::make_pair("Position", std::make_tuple(1,
                                                   DIM_S::REF::       DISIRED_POS_IND,
                                                   DIM_S::TRUTH_DISP::POS_IND,
                                                   DIM_S::ERROR::     POS_IND)),
        std::make_pair("Velocity", std::make_tuple(1,
                                                   DIM_S::REF::       DISIRED_VEL_IND,
                                                   DIM_S::TRUTH_DISP::VEL_IND,
                                                   DIM_S::ERROR::     VEL_IND)),
        std::make_pair("Disturbance Acceleration", std::make_tuple(1,
                                                                   -1,
                                                                   DIM_S::TRUTH_DISP::DISTURBANCE_ACCEL_IND,
                                                                   DIM_S::ERROR::     DISTURBANCE_ACCEL_IND)),
        std::make_pair("Accel Bias", std::make_tuple(1,
                                                     -1,
                                                     DIM_S::TRUTH_DISP::ACCEL_BIAS_IND,
                                                     DIM_S::ERROR::     ACCEL_BIAS_IND)),
        std::make_pair("Accel Scale Factor", std::make_tuple(1,
                                                             -1,
                                                             DIM_S::TRUTH_DISP::ACCEL_SCALE_FACTOR_IND,
                                                             DIM_S::ERROR::     ACCEL_SCALE_FACTOR_IND)),
        std::make_pair("Actuator Bias", std::make_tuple(1,
                                                        -1,
                                                        DIM_S::TRUTH_DISP::ACTUATOR_BIAS_IND,
                                                        DIM_S::ERROR::     ACTUATOR_BIAS_IND)),
      },
      {true,true,false,true,false,false},
      1,
      true);

  exit(EXIT_SUCCESS);
}

/* rocket_sled_demo.cpp */
