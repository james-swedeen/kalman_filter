/**
 * @File: kalman_filter.hpp
 * @Date: May 2022
 * @Author: James Swedeen
 *
 * @brief
 * A convenience header that includes everything needed to use the kalman filter in this package.
 **/

#ifndef KALMAN_FILTER_KALMAN_FILTER_HPP
#define KALMAN_FILTER_KALMAN_FILTER_HPP

/* Math Headers */
#include<kalman_filter/math/helpers.hpp>
#include<kalman_filter/math/quaternion.hpp>
#include<kalman_filter/math/performance_evaluation.hpp>
#include<kalman_filter/math/covariance_model_fitting.hpp>
#include<kalman_filter/math/steady_state_covariance.hpp>

/* Helpers Headers */
#include<kalman_filter/helpers/dimension_struct.hpp>
#include<kalman_filter/helpers/measurement_update.hpp>
#include<kalman_filter/helpers/propagate_error_covariance.hpp>
#include<kalman_filter/helpers/propagate_augmented_covariance.hpp>
#include<kalman_filter/helpers/integrator_step.hpp>
#include<kalman_filter/helpers/versions.hpp>
#include<kalman_filter/helpers/tools.hpp>
#include<kalman_filter/helpers/plot_statistics.hpp>
#include<kalman_filter/helpers/plot_statistics_tools.hpp>

/* Noise Headers */
#include<kalman_filter/noise/noise_base.hpp>
#include<kalman_filter/noise/normal_distribution.hpp>
#include<kalman_filter/noise/noise_wrapper.hpp>
#include<kalman_filter/noise/multi_noise.hpp>

/* Sensors Headers */
// Inertial Measurements Headers
#include<kalman_filter/sensors/inertial_measurements/inertial_measurement_base.hpp>
#include<kalman_filter/sensors/inertial_measurements/open_loop_imu.hpp>
#include<kalman_filter/sensors/inertial_measurements/one_dim_integrator_accelerometer.hpp>
// Measurements Headers
#include<kalman_filter/sensors/measurements/measurement_base.hpp>
#include<kalman_filter/sensors/measurements/one_dim_fix.hpp>
#include<kalman_filter/sensors/measurements/gps.hpp>
#include<kalman_filter/sensors/measurements/heading.hpp>
#include<kalman_filter/sensors/measurements/absolute_pressure.hpp>
#include<kalman_filter/sensors/measurements/ground_velocity.hpp>
#include<kalman_filter/sensors/measurements/feature_range.hpp>
#include<kalman_filter/sensors/measurements/feature_bearing.hpp>
// Measurement Controllers Headers
#include<kalman_filter/sensors/measurements/controllers/measurement_controller_base.hpp>
#include<kalman_filter/sensors/measurements/controllers/no_sensors_controller.hpp>
#include<kalman_filter/sensors/measurements/controllers/all_sensors_controller.hpp>
#include<kalman_filter/sensors/measurements/controllers/gps_heading_altitude_controller.hpp>
#include<kalman_filter/sensors/measurements/controllers/one_dim_integrator_measurement_controller.hpp>

/* Mappings Headers */
#include<kalman_filter/mappings/mappings_base.hpp>
#include<kalman_filter/mappings/simple_mapping.hpp>
#include<kalman_filter/mappings/linear_mapping.hpp>

/* Dynamics Headers */
#include<kalman_filter/dynamics/dynamics_base.hpp>
#include<kalman_filter/dynamics/basic_model.hpp>
#include<kalman_filter/dynamics/one_dim_integrator.hpp>

/* Controllers Headers */
#include<kalman_filter/controllers/controller_base.hpp>
#include<kalman_filter/controllers/open_loop_controller.hpp>
#include<kalman_filter/controllers/one_dim_integrator_controller.hpp>
#include<kalman_filter/controllers/finite_time_lqr.hpp>

/* Top Level Headers */
#include<kalman_filter/run_monte_carlo.hpp>
#include<kalman_filter/run_lin_cov.hpp>

#endif
/* kalman_filter.hpp */
