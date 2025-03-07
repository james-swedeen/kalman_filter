/**
 * @File: all_sensors_controller.hpp
 * @Date: September 2022
 * @Author: James Swedeen
 *
 * @brief
 * A class for controlling when all implemented sensors are used.
 **/

#ifndef KALMAN_FILTER_SENSORS_MEASUREMENTS_CONTROLLERS_ALL_SENSORS_CONTROLLER_HPP
#define KALMAN_FILTER_SENSORS_MEASUREMENTS_CONTROLLERS_ALL_SENSORS_CONTROLLER_HPP

/* C++ Headers */
#include<cstdint>
#include<vector>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<kalman_filter/mappings/mappings_base.hpp>
#include<kalman_filter/noise/noise_base.hpp>
#include<kalman_filter/helpers/measurement_update.hpp>
#include<kalman_filter/sensors/measurements/controllers/measurement_controller_base.hpp>
#include<kalman_filter/sensors/measurements/measurement_base.hpp>
#include<kalman_filter/sensors/measurements/absolute_pressure.hpp>
#include<kalman_filter/sensors/measurements/gps.hpp>
#include<kalman_filter/sensors/measurements/heading.hpp>
#include<kalman_filter/sensors/measurements/ground_velocity.hpp>
#include<kalman_filter/sensors/measurements/feature_range.hpp>
#include<kalman_filter/sensors/measurements/feature_bearing.hpp>

namespace kf
{
namespace sensors
{
template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
class AllSensorsController;

template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using AllSensorsControllerPtr = std::shared_ptr<AllSensorsController<DIM_S,SCALAR,OPTIONS>>;

/**
 * @DIM_S
 * The type of a Dimensions object or an inheriting object that has information about the size of the state vectors.
 *
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options.
 **/
template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class AllSensorsController
 : public MeasurementControllerBase<DIM_S,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  AllSensorsController() = delete;
  /**
   * @Copy Constructor
   **/
  AllSensorsController(const AllSensorsController&) = default;
  /**
   * @Move Constructor
   **/
  AllSensorsController(AllSensorsController&&) = default;
  /**
   * @Constructor
   *
   * @brief
   * Initializes the object for use.
   *
   * @parameters
   * gps: The gps measurement object
   * gps_noise: Noise on the gps measurement
   * heading: The heading measurement object
   * heading_noise: The noise on the heading measurement
   * absolute_pressure: The absolute_pressure measurement object
   * absolute_pressure_noise: The noise on the absolute pressure measurement
   * ground_velocity: The ground_velocity measurement object
   * ground_velocity_noise: The noise on the ground velocity measurement
   * feature_range: A vector of feature_range measurements
   * feature_range_noise: A vector of feature_range measurement noises
   * feature_bearing: A vector of feature_bearing measurements
   * feature_bearing_noise: A vector of feature_bearing measurement noises
   **/
  AllSensorsController(const MeasurementBasePtr<             3,DIM_S,SCALAR,OPTIONS>&  gps,
                       const noise::NoiseBasePtr<            3,      SCALAR,OPTIONS>&  gps_noise,
                       const MeasurementBasePtr<             1,DIM_S,SCALAR,OPTIONS>&  heading,
                       const noise::NoiseBasePtr<            1,      SCALAR,OPTIONS>&  heading_noise,
                       const MeasurementBasePtr<             1,DIM_S,SCALAR,OPTIONS>&  absolute_pressure,
                       const noise::NoiseBasePtr<            1,      SCALAR,OPTIONS>&  absolute_pressure_noise,
                       const MeasurementBasePtr<             1,DIM_S,SCALAR,OPTIONS>&  ground_velocity,
                       const noise::NoiseBasePtr<            1,      SCALAR,OPTIONS>&  ground_velocity_noise,
                       const std::vector<MeasurementBasePtr< 1,DIM_S,SCALAR,OPTIONS>>& feature_range,
                       const std::vector<noise::NoiseBasePtr<1,      SCALAR,OPTIONS>>& feature_range_noise,
                       const std::vector<MeasurementBasePtr< 2,DIM_S,SCALAR,OPTIONS>>& feature_bearing,
                       const std::vector<noise::NoiseBasePtr<2,      SCALAR,OPTIONS>>& feature_bearing_noise);
  /**
   * @Deconstructor
   **/
  ~AllSensorsController() override = default;
  /**
   * @Assignment Operators
   **/
  AllSensorsController& operator=(const AllSensorsController&)  = default;
  AllSensorsController& operator=(      AllSensorsController&&) = default;
  /**
   * @applyMeasurements
   *
   * @brief
   * Used to apply and all of the measurements that need to be applied at the given time.
   *
   * @parameters
   * mappings: A helper object that maps one state vector to another
   * time: The current simulation time
   * meas_update_buff: Buffer that holds information about the next time that a measurement should be applied
   * truth_state: The current truth state vector
   * nav_state: The current navigation state vector
   * error_covariance: The current a priori covariance matrix of the error state vector
   *
   * @return
   * The updated measurement update buffer.
   **/
  inline Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS>
    applyMeasurements(const map::MappingsBasePtr<DIM_S,SCALAR,OPTIONS>&                                           mappings,
                      const SCALAR                                                                                time,
                      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,               DIM_S::NUM_MEAS_DIM,OPTIONS>>& meas_update_buff,
                      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,               DIM_S::TRUTH_DIM,   OPTIONS>>& truth_state,
                            Eigen::Ref<      Eigen::Matrix<SCALAR,1,               DIM_S::NAV_DIM,     OPTIONS>>  nav_state,
                            Eigen::Ref<      Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,   OPTIONS>>  error_covariance) override;
  /**
   * @applyMeasurementsSSErrorCov
   *
   * @brief
   * Used to apply and all of the measurements that need to be applied at the given time.
   *
   * @parameters
   * mappings: A helper object that maps one state vector to another
   * time: The current simulation time
   * meas_update_buff: Buffer that holds information about the next time that a measurement should be applied
   * truth_state: The current truth state vector
   * ss_error_covariance: The steady state covariance matrix of the error state vector
   * nav_state: The current navigation state vector
   *
   * @return
   * The updated measurement update buffer.
   **/
  inline Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS>
    applyMeasurementsSSErrorCov(const map::MappingsBasePtr<DIM_S,SCALAR,OPTIONS>&                                           mappings,
                                const SCALAR                                                                                time,
                                const Eigen::Ref<const Eigen::Matrix<SCALAR,1,               DIM_S::NUM_MEAS_DIM,OPTIONS>>& meas_update_buff,
                                const Eigen::Ref<const Eigen::Matrix<SCALAR,1,               DIM_S::TRUTH_DIM,   OPTIONS>>& truth_state,
                                const Eigen::Ref<const Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,   OPTIONS>>& ss_error_covariance,
                                      Eigen::Ref<      Eigen::Matrix<SCALAR,1,               DIM_S::NAV_DIM,     OPTIONS>>  nav_state) override;
  /**
   * @applyMeasurementsLinCov
   *
   * @brief
   * Used to apply and all of the measurements that need to be applied at the given time.
   *
   * @parameters
   * time: The current simulation time
   * meas_update_buff: Buffer that holds information about the next time that a measurement should be applied
   * ref_truth_state: The current state from the reference trajectory mapped into a truth state vector
   * ref_nav_state: The current state from the reference trajectory mapped into a navigation state vector
   * error_covariance: The current a priori covariance matrix of the error state vector
   * aug_covariance: The current a priori covariance matrix of the augmented state vector
   *
   * @return
   * The updated measurement update buffer.
   **/
  inline Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS>
    applyMeasurementsLinCov(const SCALAR                                                                                         time,
                            const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::NUM_MEAS_DIM,   OPTIONS>>& meas_update_buff,
                            const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::TRUTH_DIM,      OPTIONS>>& ref_truth_state,
                            const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::NAV_DIM,        OPTIONS>>& ref_nav_state,
                                  Eigen::Ref<      Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,      DIM_S::ERROR_DIM,      OPTIONS>>  error_covariance,
                                  Eigen::Ref<      Eigen::Matrix<SCALAR,DIM_S::LINCOV::AUG_DIM,DIM_S::LINCOV::AUG_DIM,OPTIONS>>  aug_covariance) override;
  /**
   * @applyMeasurementsErrorBudget
   *
   * @brief
   * Used to apply and all of the measurements that need to be applied at the given time.
   *
   * @parameters
   * time: The current simulation time
   * meas_update_buff: Buffer that holds information about the next time that a measurement should be applied
   * ref_truth_state: The current state from the reference trajectory mapped into a truth state vector
   * ref_nav_state: The current state from the reference trajectory mapped into a navigation state vector
   * error_covariance: The current a priori covariance matrix of the error state vector
   * aug_covariance: The current a priori covariance matrix of the augmented state vector
   *
   * @return
   * The updated measurement update buffer.
   **/
  inline Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS>
    applyMeasurementsErrorBudget(const SCALAR                                                                                         time,
                                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::NUM_MEAS_DIM,   OPTIONS>>& meas_update_buff,
                                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::TRUTH_DIM,      OPTIONS>>& ref_truth_state,
                                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::NAV_DIM,        OPTIONS>>& ref_nav_state,
                                 const Eigen::Ref<const Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,      DIM_S::ERROR_DIM,      OPTIONS>>& error_covariance,
                                       Eigen::Ref<      Eigen::Matrix<SCALAR,DIM_S::LINCOV::AUG_DIM,DIM_S::LINCOV::AUG_DIM,OPTIONS>>  aug_covariance) override;
private:
  MeasurementBasePtr<             3,DIM_S,SCALAR,OPTIONS>  gps;
  noise::NoiseBasePtr<            3,      SCALAR,OPTIONS>  gps_noise;
  MeasurementBasePtr<             1,DIM_S,SCALAR,OPTIONS>  heading;
  noise::NoiseBasePtr<            1,      SCALAR,OPTIONS>  heading_noise;
  MeasurementBasePtr<             1,DIM_S,SCALAR,OPTIONS>  absolute_pressure;
  noise::NoiseBasePtr<            1,      SCALAR,OPTIONS>  absolute_pressure_noise;
  MeasurementBasePtr<             1,DIM_S,SCALAR,OPTIONS>  ground_velocity;
  noise::NoiseBasePtr<            1,      SCALAR,OPTIONS>  ground_velocity_noise;
  std::vector<MeasurementBasePtr< 1,DIM_S,SCALAR,OPTIONS>> feature_range;
  std::vector<noise::NoiseBasePtr<1,      SCALAR,OPTIONS>> feature_range_noise;
  std::vector<MeasurementBasePtr< 2,DIM_S,SCALAR,OPTIONS>> feature_bearing;
  std::vector<noise::NoiseBasePtr<2,      SCALAR,OPTIONS>> feature_bearing_noise;
};

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
AllSensorsController<DIM_S,SCALAR,OPTIONS>::
  AllSensorsController(const MeasurementBasePtr<             3,DIM_S,SCALAR,OPTIONS>&  gps,
                       const noise::NoiseBasePtr<            3,      SCALAR,OPTIONS>&  gps_noise,
                       const MeasurementBasePtr<             1,DIM_S,SCALAR,OPTIONS>&  heading,
                       const noise::NoiseBasePtr<            1,      SCALAR,OPTIONS>&  heading_noise,
                       const MeasurementBasePtr<             1,DIM_S,SCALAR,OPTIONS>&  absolute_pressure,
                       const noise::NoiseBasePtr<            1,      SCALAR,OPTIONS>&  absolute_pressure_noise,
                       const MeasurementBasePtr<             1,DIM_S,SCALAR,OPTIONS>&  ground_velocity,
                       const noise::NoiseBasePtr<            1,      SCALAR,OPTIONS>&  ground_velocity_noise,
                       const std::vector<MeasurementBasePtr< 1,DIM_S,SCALAR,OPTIONS>>& feature_range,
                       const std::vector<noise::NoiseBasePtr<1,      SCALAR,OPTIONS>>& feature_range_noise,
                       const std::vector<MeasurementBasePtr< 2,DIM_S,SCALAR,OPTIONS>>& feature_bearing,
                       const std::vector<noise::NoiseBasePtr<2,      SCALAR,OPTIONS>>& feature_bearing_noise)
 : MeasurementControllerBase<DIM_S,SCALAR,OPTIONS>(),
   gps(gps),
   gps_noise(gps_noise),
   heading(heading),
   heading_noise(heading_noise),
   absolute_pressure(absolute_pressure),
   absolute_pressure_noise(absolute_pressure_noise),
   ground_velocity(ground_velocity),
   ground_velocity_noise(ground_velocity_noise),
   feature_range(feature_range),
   feature_range_noise(feature_range_noise),
   feature_bearing(feature_bearing),
   feature_bearing_noise(feature_bearing_noise)
{}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS> AllSensorsController<DIM_S,SCALAR,OPTIONS>::
  applyMeasurements(const map::MappingsBasePtr<DIM_S,SCALAR,OPTIONS>&                                           mappings,
                    const SCALAR                                                                                time,
                    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,               DIM_S::NUM_MEAS_DIM,OPTIONS>>& meas_update_buff,
                    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,               DIM_S::TRUTH_DIM,   OPTIONS>>& truth_state,
                          Eigen::Ref<      Eigen::Matrix<SCALAR,1,               DIM_S::NAV_DIM,     OPTIONS>>  nav_state,
                          Eigen::Ref<      Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,   OPTIONS>>  error_covariance)
{
  Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS> output_meas_update_buff;
  // GPS
  if(this->gps->measurementReady(time, meas_update_buff[DIM_S::NUM_MEAS::GPS_IND], truth_state))
  {
    applyMeasurementMonteCarlo<3,false,DIM_S,false,SCALAR,OPTIONS>(
      this->gps,
      this->gps_noise,
      mappings,
      time,
      truth_state,
      nav_state,
      error_covariance);
  }
  output_meas_update_buff[DIM_S::NUM_MEAS::GPS_IND] =
    this->gps->updateNextMeasurementTime(time, meas_update_buff[DIM_S::NUM_MEAS::GPS_IND]);
  // Heading
  if(this->heading->measurementReady(time, meas_update_buff[DIM_S::NUM_MEAS::HEADING_IND], truth_state))
  {
    applyMeasurementMonteCarlo<1,true,DIM_S,false,SCALAR,OPTIONS>(
      this->heading,
      this->heading_noise,
      mappings,
      time,
      truth_state,
      nav_state,
      error_covariance);
  }
  output_meas_update_buff[DIM_S::NUM_MEAS::HEADING_IND] =
    this->heading->updateNextMeasurementTime(time, meas_update_buff[DIM_S::NUM_MEAS::HEADING_IND]);
  // Absolute Pressure
  if(this->absolute_pressure->measurementReady(time, meas_update_buff[DIM_S::NUM_MEAS::ABS_PRESSURE_IND], truth_state))
  {
    applyMeasurementMonteCarlo<1,false,DIM_S,false,SCALAR,OPTIONS>(
      this->absolute_pressure,
      this->absolute_pressure_noise,
      mappings,
      time,
      truth_state,
      nav_state,
      error_covariance);
  }
  output_meas_update_buff[DIM_S::NUM_MEAS::ABS_PRESSURE_IND] =
    this->absolute_pressure->updateNextMeasurementTime(time, meas_update_buff[DIM_S::NUM_MEAS::ABS_PRESSURE_IND]);
  // Ground Velocity
  if(this->ground_velocity->measurementReady(time, meas_update_buff[DIM_S::NUM_MEAS::GROUND_VELOCITY_IND], truth_state))
  {
    applyMeasurementMonteCarlo<1,false,DIM_S,false,SCALAR,OPTIONS>(
      this->ground_velocity,
      this->ground_velocity_noise,
      mappings,
      time,
      truth_state,
      nav_state,
      error_covariance);
  }
  output_meas_update_buff[DIM_S::NUM_MEAS::GROUND_VELOCITY_IND] =
    this->ground_velocity->updateNextMeasurementTime(time, meas_update_buff[DIM_S::NUM_MEAS::GROUND_VELOCITY_IND]);
  // Feature Range
  const size_t feature_range_size = this->feature_range.size();
  if(0 != feature_range_size)
  {
    for(size_t it = 0; it < feature_range_size; ++it)
    {
      if(this->feature_range[it]->measurementReady(time, meas_update_buff[DIM_S::NUM_MEAS::FEATURE_RANGE_IND], truth_state))
      {
        applyMeasurementMonteCarlo<1,false,DIM_S,false,SCALAR,OPTIONS>(
          this->feature_range[it],
          this->feature_range_noise[it],
          mappings,
          time,
          truth_state,
          nav_state,
          error_covariance);
      }
    }
    output_meas_update_buff[DIM_S::NUM_MEAS::FEATURE_RANGE_IND] =
      this->feature_range[0]->updateNextMeasurementTime(time, meas_update_buff[DIM_S::NUM_MEAS::FEATURE_RANGE_IND]);
  }
  // Feature Bearing
  const size_t feature_bearing_size = this->feature_bearing.size();
  if(0 != feature_bearing_size)
  {
    for(size_t it = 0; it < feature_bearing_size; ++it)
    {
      if(this->feature_bearing[it]->measurementReady(time, meas_update_buff[DIM_S::NUM_MEAS::FEATURE_BEARING_IND], truth_state))
      {
        applyMeasurementMonteCarlo<2,false,DIM_S,false,SCALAR,OPTIONS>(
          this->feature_bearing[it],
          this->feature_bearing_noise[it],
          mappings,
          time,
          truth_state,
          nav_state,
          error_covariance);
      }
    }
    output_meas_update_buff[DIM_S::NUM_MEAS::FEATURE_BEARING_IND] =
      this->feature_bearing[0]->updateNextMeasurementTime(time, meas_update_buff[DIM_S::NUM_MEAS::FEATURE_BEARING_IND]);
  }

  return output_meas_update_buff;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS> AllSensorsController<DIM_S,SCALAR,OPTIONS>::
  applyMeasurementsSSErrorCov(const map::MappingsBasePtr<DIM_S,SCALAR,OPTIONS>&                                           mappings,
                              const SCALAR                                                                                time,
                              const Eigen::Ref<const Eigen::Matrix<SCALAR,1,               DIM_S::NUM_MEAS_DIM,OPTIONS>>& meas_update_buff,
                              const Eigen::Ref<const Eigen::Matrix<SCALAR,1,               DIM_S::TRUTH_DIM,   OPTIONS>>& truth_state,
                              const Eigen::Ref<const Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,   OPTIONS>>& ss_error_covariance,
                                    Eigen::Ref<      Eigen::Matrix<SCALAR,1,               DIM_S::NAV_DIM,     OPTIONS>>  nav_state)
{

  Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS> output_meas_update_buff;
  // GPS
  if(this->gps->measurementReady(time, meas_update_buff[DIM_S::NUM_MEAS::GPS_IND], truth_state))
  {
    applyMeasurementMonteCarloSS<3,false,DIM_S,false,SCALAR,OPTIONS>(
      this->gps,
      this->gps_noise,
      mappings,
      time,
      truth_state,
      ss_error_covariance,
      nav_state);
  }
  output_meas_update_buff[DIM_S::NUM_MEAS::GPS_IND] =
    this->gps->updateNextMeasurementTime(time, meas_update_buff[DIM_S::NUM_MEAS::GPS_IND]);
  // Heading
  if(this->heading->measurementReady(time, meas_update_buff[DIM_S::NUM_MEAS::HEADING_IND], truth_state))
  {
    applyMeasurementMonteCarloSS<1,true,DIM_S,false,SCALAR,OPTIONS>(
      this->heading,
      this->heading_noise,
      mappings,
      time,
      truth_state,
      ss_error_covariance,
      nav_state);
  }
  output_meas_update_buff[DIM_S::NUM_MEAS::HEADING_IND] =
    this->heading->updateNextMeasurementTime(time, meas_update_buff[DIM_S::NUM_MEAS::HEADING_IND]);
  // Absolute Pressure
  if(this->absolute_pressure->measurementReady(time, meas_update_buff[DIM_S::NUM_MEAS::ABS_PRESSURE_IND], truth_state))
  {
    applyMeasurementMonteCarloSS<1,false,DIM_S,false,SCALAR,OPTIONS>(
      this->absolute_pressure,
      this->absolute_pressure_noise,
      mappings,
      time,
      truth_state,
      ss_error_covariance,
      nav_state);
  }
  output_meas_update_buff[DIM_S::NUM_MEAS::ABS_PRESSURE_IND] =
    this->absolute_pressure->updateNextMeasurementTime(time, meas_update_buff[DIM_S::NUM_MEAS::ABS_PRESSURE_IND]);
  // Ground Velocity
  if(this->ground_velocity->measurementReady(time, meas_update_buff[DIM_S::NUM_MEAS::GROUND_VELOCITY_IND], truth_state))
  {
    applyMeasurementMonteCarloSS<1,false,DIM_S,false,SCALAR,OPTIONS>(
      this->ground_velocity,
      this->ground_velocity_noise,
      mappings,
      time,
      truth_state,
      ss_error_covariance,
      nav_state);
  }
  output_meas_update_buff[DIM_S::NUM_MEAS::GROUND_VELOCITY_IND] =
    this->ground_velocity->updateNextMeasurementTime(time, meas_update_buff[DIM_S::NUM_MEAS::GROUND_VELOCITY_IND]);
  // Feature Range
  const size_t feature_range_size = this->feature_range.size();
  if(0 != feature_range_size)
  {
    for(size_t it = 0; it < feature_range_size; ++it)
    {
      if(this->feature_range[it]->measurementReady(time, meas_update_buff[DIM_S::NUM_MEAS::FEATURE_RANGE_IND], truth_state))
      {
        applyMeasurementMonteCarloSS<1,false,DIM_S,false,SCALAR,OPTIONS>(
          this->feature_range[it],
          this->feature_range_noise[it],
          mappings,
          time,
          truth_state,
          ss_error_covariance,
          nav_state);
      }
    }
    output_meas_update_buff[DIM_S::NUM_MEAS::FEATURE_RANGE_IND] =
      this->feature_range[0]->updateNextMeasurementTime(time, meas_update_buff[DIM_S::NUM_MEAS::FEATURE_RANGE_IND]);
  }
  // Feature Bearing
  const size_t feature_bearing_size = this->feature_bearing.size();
  if(0 != feature_bearing_size)
  {
    for(size_t it = 0; it < feature_bearing_size; ++it)
    {
      if(this->feature_bearing[it]->measurementReady(time, meas_update_buff[DIM_S::NUM_MEAS::FEATURE_BEARING_IND], truth_state))
      {
        applyMeasurementMonteCarloSS<2,false,DIM_S,false,SCALAR,OPTIONS>(
          this->feature_bearing[it],
          this->feature_bearing_noise[it],
          mappings,
          time,
          truth_state,
          ss_error_covariance,
          nav_state);
      }
    }
    output_meas_update_buff[DIM_S::NUM_MEAS::FEATURE_BEARING_IND] =
      this->feature_bearing[0]->updateNextMeasurementTime(time, meas_update_buff[DIM_S::NUM_MEAS::FEATURE_BEARING_IND]);
  }

  return output_meas_update_buff;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS> AllSensorsController<DIM_S,SCALAR,OPTIONS>::
  applyMeasurementsLinCov(const SCALAR                                                                                         time,
                          const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::NUM_MEAS_DIM,   OPTIONS>>& meas_update_buff,
                          const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::TRUTH_DIM,      OPTIONS>>& ref_truth_state,
                          const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::NAV_DIM,        OPTIONS>>& ref_nav_state,
                                Eigen::Ref<      Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,      DIM_S::ERROR_DIM,      OPTIONS>>  error_covariance,
                                Eigen::Ref<      Eigen::Matrix<SCALAR,DIM_S::LINCOV::AUG_DIM,DIM_S::LINCOV::AUG_DIM,OPTIONS>>  aug_covariance)
{
  Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS> output_meas_update_buff;
  // GPS
  if(this->gps->measurementReady(time, meas_update_buff[DIM_S::NUM_MEAS::GPS_IND], ref_truth_state))
  {
    applyMeasurementLinCov<3,DIM_S,false,SCALAR,OPTIONS>(
      this->gps,
      this->gps_noise,
      time,
      ref_truth_state,
      ref_nav_state,
      error_covariance,
      aug_covariance);
  }
  output_meas_update_buff[DIM_S::NUM_MEAS::GPS_IND] =
    this->gps->updateNextMeasurementTime(time, meas_update_buff[DIM_S::NUM_MEAS::GPS_IND]);
  // Heading
  if(this->heading->measurementReady(time, meas_update_buff[DIM_S::NUM_MEAS::HEADING_IND], ref_truth_state))
  {
    applyMeasurementLinCov<1,DIM_S,false,SCALAR,OPTIONS>(
      this->heading,
      this->heading_noise,
      time,
      ref_truth_state,
      ref_nav_state,
      error_covariance,
      aug_covariance);
  }
  output_meas_update_buff[DIM_S::NUM_MEAS::HEADING_IND] =
    this->heading->updateNextMeasurementTime(time, meas_update_buff[DIM_S::NUM_MEAS::HEADING_IND]);
  // Absolute Pressure
  if(this->absolute_pressure->measurementReady(time, meas_update_buff[DIM_S::NUM_MEAS::ABS_PRESSURE_IND], ref_truth_state))
  {
    applyMeasurementLinCov<1,DIM_S,false,SCALAR,OPTIONS>(
      this->absolute_pressure,
      this->absolute_pressure_noise,
      time,
      ref_truth_state,
      ref_nav_state,
      error_covariance,
      aug_covariance);
  }
  output_meas_update_buff[DIM_S::NUM_MEAS::ABS_PRESSURE_IND] =
    this->absolute_pressure->updateNextMeasurementTime(time, meas_update_buff[DIM_S::NUM_MEAS::ABS_PRESSURE_IND]);
  // Ground Velocity
  if(this->ground_velocity->measurementReady(time, meas_update_buff[DIM_S::NUM_MEAS::GROUND_VELOCITY_IND], ref_truth_state))
  {
    applyMeasurementLinCov<1,DIM_S,false,SCALAR,OPTIONS>(
      this->ground_velocity,
      this->ground_velocity_noise,
      time,
      ref_truth_state,
      ref_nav_state,
      error_covariance,
      aug_covariance);
  }
  output_meas_update_buff[DIM_S::NUM_MEAS::GROUND_VELOCITY_IND] =
    this->ground_velocity->updateNextMeasurementTime(time, meas_update_buff[DIM_S::NUM_MEAS::GROUND_VELOCITY_IND]);
  // Feature Range
  const size_t feature_range_size = this->feature_range.size();
  if(0 != feature_range_size)
  {
    for(size_t it = 0; it < feature_range_size; ++it)
    {
      if(this->feature_range[it]->measurementReady(time, meas_update_buff[DIM_S::NUM_MEAS::FEATURE_RANGE_IND], ref_truth_state))
      {
        applyMeasurementLinCov<1,DIM_S,false,SCALAR,OPTIONS>(
          this->feature_range[it],
          this->feature_range_noise[it],
          time,
          ref_truth_state,
          ref_nav_state,
          error_covariance,
          aug_covariance);
      }
    }
    output_meas_update_buff[DIM_S::NUM_MEAS::FEATURE_RANGE_IND] =
      this->feature_range[0]->updateNextMeasurementTime(time, meas_update_buff[DIM_S::NUM_MEAS::FEATURE_RANGE_IND]);
  }
  // Feature Bearing
  const size_t feature_bearing_size = this->feature_bearing.size();
  if(0 != feature_bearing_size)
  {
    for(size_t it = 0; it < feature_bearing_size; ++it)
    {
      if(this->feature_bearing[it]->measurementReady(time, meas_update_buff[DIM_S::NUM_MEAS::FEATURE_BEARING_IND], ref_truth_state))
      {
        applyMeasurementLinCov<2,DIM_S,false,SCALAR,OPTIONS>(
          this->feature_bearing[it],
          this->feature_bearing_noise[it],
          time,
          ref_truth_state,
          ref_nav_state,
          error_covariance,
          aug_covariance);
      }
    }
    output_meas_update_buff[DIM_S::NUM_MEAS::FEATURE_BEARING_IND] =
      this->feature_bearing[0]->updateNextMeasurementTime(time, meas_update_buff[DIM_S::NUM_MEAS::FEATURE_BEARING_IND]);
  }

  return output_meas_update_buff;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS> AllSensorsController<DIM_S,SCALAR,OPTIONS>::
  applyMeasurementsErrorBudget(const SCALAR                                                                                         time,
                               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::NUM_MEAS_DIM,   OPTIONS>>& meas_update_buff,
                               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::TRUTH_DIM,      OPTIONS>>& ref_truth_state,
                               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::NAV_DIM,        OPTIONS>>& ref_nav_state,
                               const Eigen::Ref<const Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,      DIM_S::ERROR_DIM,      OPTIONS>>& error_covariance,
                                     Eigen::Ref<      Eigen::Matrix<SCALAR,DIM_S::LINCOV::AUG_DIM,DIM_S::LINCOV::AUG_DIM,OPTIONS>>  aug_covariance)
{
  Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS> output_meas_update_buff;
  // GPS
  if(this->gps->measurementReady(time, meas_update_buff[DIM_S::NUM_MEAS::GPS_IND], ref_truth_state))
  {
    applyMeasurementErrorBudget<3,DIM_S,false,SCALAR,OPTIONS>(
      this->gps,
      this->gps_noise,
      time,
      ref_truth_state,
      ref_nav_state,
      error_covariance,
      aug_covariance);
  }
  output_meas_update_buff[DIM_S::NUM_MEAS::GPS_IND] =
    this->gps->updateNextMeasurementTime(time, meas_update_buff[DIM_S::NUM_MEAS::GPS_IND]);
  // Heading
  if(this->heading->measurementReady(time, meas_update_buff[DIM_S::NUM_MEAS::HEADING_IND], ref_truth_state))
  {
    applyMeasurementErrorBudget<1,DIM_S,false,SCALAR,OPTIONS>(
      this->heading,
      this->heading_noise,
      time,
      ref_truth_state,
      ref_nav_state,
      error_covariance,
      aug_covariance);
  }
  output_meas_update_buff[DIM_S::NUM_MEAS::HEADING_IND] =
    this->heading->updateNextMeasurementTime(time, meas_update_buff[DIM_S::NUM_MEAS::HEADING_IND]);
  // Absolute Pressure
  if(this->absolute_pressure->measurementReady(time, meas_update_buff[DIM_S::NUM_MEAS::ABS_PRESSURE_IND], ref_truth_state))
  {
    applyMeasurementErrorBudget<1,DIM_S,false,SCALAR,OPTIONS>(
      this->absolute_pressure,
      this->absolute_pressure_noise,
      time,
      ref_truth_state,
      ref_nav_state,
      error_covariance,
      aug_covariance);
  }
  output_meas_update_buff[DIM_S::NUM_MEAS::ABS_PRESSURE_IND] =
    this->absolute_pressure->updateNextMeasurementTime(time, meas_update_buff[DIM_S::NUM_MEAS::ABS_PRESSURE_IND]);
  // Ground Velocity
  if(this->ground_velocity->measurementReady(time, meas_update_buff[DIM_S::NUM_MEAS::GROUND_VELOCITY_IND], ref_truth_state))
  {
    applyMeasurementErrorBudget<1,DIM_S,false,SCALAR,OPTIONS>(
      this->ground_velocity,
      this->ground_velocity_noise,
      time,
      ref_truth_state,
      ref_nav_state,
      error_covariance,
      aug_covariance);
  }
  output_meas_update_buff[DIM_S::NUM_MEAS::GROUND_VELOCITY_IND] =
    this->ground_velocity->updateNextMeasurementTime(time, meas_update_buff[DIM_S::NUM_MEAS::GROUND_VELOCITY_IND]);
  // Feature Range
  const size_t feature_range_size = this->feature_range.size();
  if(0 != feature_range_size)
  {
    for(size_t it = 0; it < feature_range_size; ++it)
    {
      if(this->feature_range[it]->measurementReady(time, meas_update_buff[DIM_S::NUM_MEAS::FEATURE_RANGE_IND], ref_truth_state))
      {
        applyMeasurementErrorBudget<1,DIM_S,false,SCALAR,OPTIONS>(
          this->feature_range[it],
          this->feature_range_noise[it],
          time,
          ref_truth_state,
          ref_nav_state,
          error_covariance,
          aug_covariance);
      }
    }
    output_meas_update_buff[DIM_S::NUM_MEAS::FEATURE_RANGE_IND] =
      this->feature_range[0]->updateNextMeasurementTime(time, meas_update_buff[DIM_S::NUM_MEAS::FEATURE_RANGE_IND]);
  }
  // Feature Bearing
  const size_t feature_bearing_size = this->feature_bearing.size();
  if(0 != feature_bearing_size)
  {
    for(size_t it = 0; it < feature_bearing_size; ++it)
    {
      if(this->feature_bearing[it]->measurementReady(time, meas_update_buff[DIM_S::NUM_MEAS::FEATURE_BEARING_IND], ref_truth_state))
      {
        applyMeasurementErrorBudget<2,DIM_S,false,SCALAR,OPTIONS>(
          this->feature_bearing[it],
          this->feature_bearing_noise[it],
          time,
          ref_truth_state,
          ref_nav_state,
          error_covariance,
          aug_covariance);
      }
    }
    output_meas_update_buff[DIM_S::NUM_MEAS::FEATURE_BEARING_IND] =
      this->feature_bearing[0]->updateNextMeasurementTime(time, meas_update_buff[DIM_S::NUM_MEAS::FEATURE_BEARING_IND]);
  }

  return output_meas_update_buff;
}
} // namespace sensors
} // namespace kf

#endif
/* all_sensors_controller.hpp */
