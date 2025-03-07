/**
 * @File: finite_time_lqr_evaluation.cpp
 * @Author: James Swedeen
 * @Date: April 2023
 *
 * @brief
 * Tests the Finite-Time LQR controller in a Monte Carlo simulation.
 **/

/* C++ Headers */
#include<utility>
#include<algorithm>
#include<memory>
#include<vector>
#include<iostream>

/* Local Headers */
#include<kalman_filter/dynamics/one_dim_integrator.hpp>
#include<kalman_filter/sensors/inertial_measurements/one_dim_integrator_accelerometer.hpp>
#include<kalman_filter/mappings/linear_mapping.hpp>
#include<kalman_filter/controllers/finite_time_lqr.hpp>
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

  /// Declare LQR parameters
  const double                                    eps = 0;
  const Eigen::Matrix<double,6,6,Eigen::RowMajor> Q   = Eigen::Matrix<double,1,6,Eigen::RowMajor>({1,1,eps,eps,eps,eps}).matrix().asDiagonal();
  const Eigen::Matrix<double,1,1,Eigen::RowMajor> R   = Eigen::Matrix<double,1,1,Eigen::RowMajor>::Constant(1);
  const Eigen::Matrix<double,6,6,Eigen::RowMajor> S   = Eigen::Matrix<double,1,6,Eigen::RowMajor>({1,1,eps,eps,eps,eps}).matrix().asDiagonal();

  /// Declare simulation variables
  const double       start_time      = 0;
  const double       end_time        = 250;
  const double       end_point       = 25;
  const double       sim_dt          = 0.05;
  const double       first_meas_time = 0;
  const Eigen::Index sim_len         = end_time/sim_dt;
  const size_t       num_mc_sims     = 250;

  /// Declare noise parameters
  const double wind_std                            = double(1e-3)/double(3);              // Meters
  const double disturbance_accelerations_ss_std    = (double(0.01)/double(3));            // Meters/Second^2
  const double disturbance_acceleration_time_const = 20;                                  // Seconds
  const double accel_bias_ss_std                   = (double(900*(1e-6)*9.81)/double(3)); // Meters/Second^2
  const double accel_bias_time_const               = 60;                                  // Seconds
  const double accel_scale_factor_ss_std           = (double(900*(1e-6))/double(3));      // Unitless
  const double accel_scale_factor_time_const       = 60;                                  // Seconds
  const double actuator_bias_ss_std                = (double(0.03)/double(3));            // Meters/Second^2
  const double actuator_bias_time_const            = 60;                                  // Seconds
  const double accel_measurement_std               = (double(150*(1e-6)*9.81)/double(3)); // Meters/Second
  const double pos_measument_std                   = double(0.1)/double(3);               // Meters/Second
  const double pos_samp_rate                       = 1;                                   // Seconds
  const double vel_measument_std                   = double(0.01)/double(3);              // Meters/Second
  const double vel_samp_rate                       = 0.5;                                 // Seconds

  /// Put noise parameters in units of variance
  const double wind_var                      = std::pow(wind_std, 2);
  const double disturbance_accelerations_var = (double(2)*std::pow(disturbance_accelerations_ss_std, 2))/disturbance_acceleration_time_const;
  const double accel_bias_var                = (double(2)*std::pow(accel_bias_ss_std,                2))/accel_bias_time_const;
  const double accel_scale_factor_var        = (double(2)*std::pow(accel_scale_factor_ss_std,        2))/accel_scale_factor_time_const;
  const double actuator_bias_var             = (double(2)*std::pow(actuator_bias_ss_std,             2))/actuator_bias_time_const;
  const double accel_measurement_var         = std::pow(accel_measurement_std, 2);
  const double pos_measument_var             = std::pow(pos_measument_std,     2);
  const double vel_measument_var             = std::pow(vel_measument_std,     2);

  /// Make Kalman filter helper objects
  kf::Tools<DIM_S> tools;
  // Dynamics
  tools.dynamics = std::make_shared<kf::dynamics::OneDimIntegrator<>>(sim_dt,
                                                                      disturbance_acceleration_time_const,
                                                                      accel_bias_time_const,
                                                                      accel_scale_factor_time_const,
                                                                      actuator_bias_time_const);
  // Process noise
  tools.truth_process_noise = std::make_shared<kf::noise::NormalDistribution<5,true,true,false>>(
                                Eigen::Matrix<double,1,5,Eigen::RowMajor>::Zero(),
                                Eigen::Matrix<double,1,5,Eigen::RowMajor>({wind_var,disturbance_accelerations_var,accel_bias_var,accel_scale_factor_var,actuator_bias_var}).matrix().asDiagonal());
  // Inertial measurements
  tools.inertial_measurements = std::make_shared<kf::sensors::OneDimIntegratorAccelerometer<>>();
  // Inertial measurement noise
  tools.inertial_measurements_noise = std::make_shared<kf::noise::NormalDistribution<1,true,true,false>>(
                                       Eigen::Matrix<double,1,1,Eigen::RowMajor>::Zero(),
                                       Eigen::Matrix<double,1,1,Eigen::RowMajor>(accel_measurement_var).matrix().asDiagonal());
  // Discrete measurements
  tools.measurement_controller = std::make_shared<kf::sensors::OneDimIntegratorMeasurementController<>>(
                                   std::make_shared<kf::sensors::OneDimFix<DIM_S::NAV::POS_IND,DIM_S>>(
                                     pos_samp_rate),
                                   std::make_shared<kf::noise::NormalDistribution<1,true,true,false>>(
                                     Eigen::Matrix<double,1,1,Eigen::RowMajor>::Zero(),
                                     Eigen::Matrix<double,1,1,Eigen::RowMajor>(pos_measument_var).matrix().asDiagonal()),
                                   std::make_shared<kf::sensors::OneDimFix<DIM_S::NAV::VEL_IND,DIM_S>>(
                                     vel_samp_rate),
                                   std::make_shared<kf::noise::NormalDistribution<1,true,true,false>>(
                                     Eigen::Matrix<double,1,1,Eigen::RowMajor>::Zero(),
                                     Eigen::Matrix<double,1,1,Eigen::RowMajor>(vel_measument_var).matrix().asDiagonal()));
  // State mappings
  tools.mappings = std::make_shared<kf::map::LinearMapping<DIM_S>>();
  // Controller
  tools.controller = std::make_shared<kf::control::FiniteTimeLQR<DIM_S>>(
                       start_time,
                       end_time,
                       sim_dt,
                       tools.dynamics->getTruthStateDynamicsPDWRDispersionState(std::numeric_limits<double>::quiet_NaN(),
                                                                                Eigen::Matrix<double,1,6,Eigen::RowMajor>::Constant(std::numeric_limits<double>::quiet_NaN()),
                                                                                Eigen::Matrix<double,1,1,Eigen::RowMajor>::Constant(std::numeric_limits<double>::quiet_NaN())),
                       tools.dynamics->getTruthStateDynamicsPDWRControl(std::numeric_limits<double>::quiet_NaN(),
                                                                        Eigen::Matrix<double,1,6,Eigen::RowMajor>::Constant(std::numeric_limits<double>::quiet_NaN()),
                                                                        Eigen::Matrix<double,1,1,Eigen::RowMajor>::Constant(std::numeric_limits<double>::quiet_NaN())),
                       Q,
                       R,
                       S,
                       tools.mappings);

  /// Setup initial simulation state
  // Make reference trajectory
  Eigen::Matrix<double,Eigen::Dynamic,2,Eigen::RowMajor> reference_trajectory(sim_len, 2);
  reference_trajectory.col(0).setConstant(end_point);
  reference_trajectory.col(1).setConstant(0);

  // Make starting state
  Eigen::Matrix<double,1,DIM_S::TRUTH_DIM,Eigen::RowMajor> init_truth_state;
  Eigen::Matrix<double,1,DIM_S::NAV_DIM,  Eigen::RowMajor> init_nav_state;

  init_truth_state.setZero();
  init_nav_state.  setZero();

  // Make starting covariance
  Eigen::Matrix<double,DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_DISP_DIM,Eigen::RowMajor> init_truth_cov;
  init_truth_cov.setZero();
  init_truth_cov(DIM_S::TRUTH_DISP::DISTURBANCE_ACCEL_IND,
                 DIM_S::TRUTH_DISP::DISTURBANCE_ACCEL_IND) = std::pow(disturbance_accelerations_ss_std, 2);
  init_truth_cov(DIM_S::TRUTH_DISP::ACCEL_BIAS_IND,
                 DIM_S::TRUTH_DISP::ACCEL_BIAS_IND) = std::pow(accel_bias_ss_std, 2);
  init_truth_cov(DIM_S::TRUTH_DISP::ACCEL_SCALE_FACTOR_IND,
                 DIM_S::TRUTH_DISP::ACCEL_SCALE_FACTOR_IND) = std::pow(accel_scale_factor_ss_std, 2);
  init_truth_cov(DIM_S::TRUTH_DISP::ACTUATOR_BIAS_IND,
                 DIM_S::TRUTH_DISP::ACTUATOR_BIAS_IND) = std::pow(actuator_bias_ss_std, 2);

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

  std::cout << "MC simulation took " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_timing).count() << " milliseconds" << std::endl;

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

  std::cout << "LC simulation took " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_timing).count() << " milliseconds" << std::endl;

  /// Print ending dispersions
  const Eigen::Map<const Eigen::Matrix<double,DIM_S::LINCOV::AUG_DIM,DIM_S::LINCOV::AUG_DIM,Eigen::RowMajor>> end_aug_cov(lc_output.bottomRows<1>().middleCols<DIM_S::LINCOV::AUG_COV_LEN>(DIM_S::LINCOV::AUG_COV_START_IND).data());
  const Eigen::Matrix<double,DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_DISP_DIM,Eigen::RowMajor> end_truth_disp_cov =
    end_aug_cov.block<DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_DISP_DIM>(DIM_S::LINCOV::TRUTH_DISP_START_IND, DIM_S::LINCOV::TRUTH_DISP_START_IND);

  std::cout << "Position Truth Dispersion:                   " << double(3) * std::sqrt(end_truth_disp_cov(DIM_S::TRUTH_DISP::POS_IND,               DIM_S::TRUTH_DISP::POS_IND))                << std::endl;
  std::cout << "Velocity Truth Dispersion:                   " << double(3) * std::sqrt(end_truth_disp_cov(DIM_S::TRUTH_DISP::VEL_IND,               DIM_S::TRUTH_DISP::VEL_IND))                << std::endl;
  std::cout << "Disturbance Accelerations Truth Dispersion:  " << double(3) * std::sqrt(end_truth_disp_cov(DIM_S::TRUTH_DISP::DISTURBANCE_ACCEL_IND, DIM_S::TRUTH_DISP::DISTURBANCE_ACCEL_IND))  << std::endl;
  std::cout << "Accelerometer Bias Truth Dispersion:         " << double(3) * std::sqrt(end_truth_disp_cov(DIM_S::TRUTH_DISP::ACCEL_BIAS_IND,        DIM_S::TRUTH_DISP::ACCEL_BIAS_IND))         << std::endl;
  std::cout << "Accelerometer Scale Factor Truth Dispersion: " << double(3) * std::sqrt(end_truth_disp_cov(DIM_S::TRUTH_DISP::ACCEL_SCALE_FACTOR_IND,DIM_S::TRUTH_DISP::ACCEL_SCALE_FACTOR_IND)) << std::endl;
  std::cout << "Actuator Bias Truth Dispersion:              " << double(3) * std::sqrt(end_truth_disp_cov(DIM_S::TRUTH_DISP::ACTUATOR_BIAS_IND,     DIM_S::TRUTH_DISP::ACTUATOR_BIAS_IND))      << std::endl;

  /// Make plots of Monte Carlo and LinCov results
  std::cout << "Plotting MC and LC results" << std::endl;

  const auto plot_map_func = [](const Eigen::Matrix<double,Eigen::Dynamic,DIM_S::TRUTH_DIM,Eigen::RowMajor>& input) -> Eigen::Matrix<double,Eigen::Dynamic,DIM_S::TRUTH_DISP_DIM,Eigen::RowMajor>
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
      std::max<size_t>(1, std::floor(double(num_mc_sims)/double(100))),
      true);

  exit(EXIT_SUCCESS);
}

/* finite_time_lqr_evaluation.cpp */
