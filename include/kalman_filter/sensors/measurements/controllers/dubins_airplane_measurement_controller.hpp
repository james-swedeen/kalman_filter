/**
 * @File: dubins_airplane_measurement_controller.hpp
 * @Date: July 2023
 * @Author: James Swedeen
 *
 * @brief
 * A class for controlling when the GPS, Heading, and Altitude measurements are applied.
 **/

#ifndef KALMAN_FILTER_SENSORS_MEASUREMENTS_CONTROLLERS_DUBINS_AIRPLANE_MEASUREMENT_CONTROLLER_HPP
#define KALMAN_FILTER_SENSORS_MEASUREMENTS_CONTROLLERS_DUBINS_AIRPLANE_MEASUREMENT_CONTROLLER_HPP

/* C++ Headers */
#include<cstdint>

/* Boost Headers */
#include<boost/range/irange.hpp>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<kalman_filter/sensors/measurements/measurement_base.hpp>
#include<kalman_filter/noise/noise_base.hpp>
#include<kalman_filter/helpers/measurement_update.hpp>
#include<kalman_filter/sensors/measurements/controllers/measurement_controller_base.hpp>
#include<kalman_filter/noise/normal_distribution.hpp>
#include<kalman_filter/math/performance_evaluation.hpp>
#include<kalman_filter/mappings/dubins_airplane_mapping.hpp>

namespace kf
{
namespace sensors
{
template<typename DIM_S, bool USE_SS_KALMAN_GAIN, typename SCALAR, Eigen::StorageOptions OPTIONS>
class DubinsAirplaneMeasurementController;

template<typename DIM_S, bool USE_SS_KALMAN_GAIN = false, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using DubinsAirplaneMeasurementControllerPtr = std::shared_ptr<DubinsAirplaneMeasurementController<DIM_S,USE_SS_KALMAN_GAIN,SCALAR,OPTIONS>>;

/**
 * @DIM_S
 * The type of a Dimensions object or an inheriting object that has information about the size of the state vectors.
 *
 * @USE_SS_KALMAN_GAIN
 * If true this object with use the steady state kalman gains held in the measurement objects
 *
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options.
 **/
template<typename DIM_S, bool USE_SS_KALMAN_GAIN = false, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class DubinsAirplaneMeasurementController
 : public MeasurementControllerBase<DIM_S,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  DubinsAirplaneMeasurementController() = delete;
  /**
   * @Copy Constructor
   **/
  DubinsAirplaneMeasurementController(const DubinsAirplaneMeasurementController&) noexcept = default;
  /**
   * @Move Constructor
   **/
  DubinsAirplaneMeasurementController(DubinsAirplaneMeasurementController&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * Initializes the object for use.
   *
   * @parameters
   * gps_pos: The gps measurement object
   * gps_pos_noise: The gps measurement noise
   * gps_heading: The gps heading measurement object
   * gps_heading_noise: The gps heading measurement noise
   * compass: The compass heading measurement object
   * compass_noise: The heading measurement noise
   * altitude: The altitude measurement object
   * altitude_noise: The altitude measurement noise
   * air_speed: The air speed measurement object
   * air_speed_noise: The air speed measurement noise
   * feature_range: A vector of feature_range measurements
   * feature_range_noise: A vector of feature_range measurement noises
   * feature_bearing: A vector of feature_bearing measurements
   * feature_bearing_noise: A vector of feature_bearing measurement noises
   **/
  DubinsAirplaneMeasurementController(const MeasurementBasePtr<             3,DIM_S,SCALAR,OPTIONS>&  gps_pos,
                                      const noise::NoiseBasePtr<            3,      SCALAR,OPTIONS>&  gps_pos_noise,
                                      const MeasurementBasePtr<             1,DIM_S,SCALAR,OPTIONS>&  gps_heading,
                                      const noise::NoiseBasePtr<            1,      SCALAR,OPTIONS>&  gps_heading_noise,
                                      const MeasurementBasePtr<             1,DIM_S,SCALAR,OPTIONS>&  compass,
                                      const noise::NoiseBasePtr<            1,      SCALAR,OPTIONS>&  compass_noise,
                                      const MeasurementBasePtr<             1,DIM_S,SCALAR,OPTIONS>&  altitude,
                                      const noise::NoiseBasePtr<            1,      SCALAR,OPTIONS>&  altitude_noise,
                                      const MeasurementBasePtr<             1,DIM_S,SCALAR,OPTIONS>&  air_speed,
                                      const noise::NoiseBasePtr<            1,      SCALAR,OPTIONS>&  air_speed_noise,
                                      const std::vector<MeasurementBasePtr< 1,DIM_S,SCALAR,OPTIONS>>& feature_range,
                                      const std::vector<noise::NoiseBasePtr<1,      SCALAR,OPTIONS>>& feature_range_noise,
                                      const std::vector<MeasurementBasePtr< 2,DIM_S,SCALAR,OPTIONS>>& feature_bearing,
                                      const std::vector<noise::NoiseBasePtr<2,      SCALAR,OPTIONS>>& feature_bearing_noise);
  /**
   * @Deconstructor
   **/
  ~DubinsAirplaneMeasurementController() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  DubinsAirplaneMeasurementController& operator=(const DubinsAirplaneMeasurementController&)  noexcept = default;
  DubinsAirplaneMeasurementController& operator=(      DubinsAirplaneMeasurementController&&) noexcept = default;
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
   * Used to apply and all of the measurements that need to be applied at the given time assuming a steady state
   * error covariance.
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
  /**
   * @helpers
   **/
MeasurementBasePtr<             3,DIM_S,SCALAR,OPTIONS>& getGPSMeasurement()
{ return this->gps_pos; }
std::vector<MeasurementBasePtr< 2,DIM_S,SCALAR,OPTIONS>>& getFeatureMeasurement()
{ return this->feature_bearing; }
private:
  MeasurementBasePtr<             3,DIM_S,SCALAR,OPTIONS>  gps_pos;
  noise::NoiseBasePtr<            3,      SCALAR,OPTIONS>  gps_pos_noise;
  MeasurementBasePtr<             1,DIM_S,SCALAR,OPTIONS>  gps_heading;
  noise::NoiseBasePtr<            1,      SCALAR,OPTIONS>  gps_heading_noise;
  MeasurementBasePtr<             1,DIM_S,SCALAR,OPTIONS>  compass;
  noise::NoiseBasePtr<            1,      SCALAR,OPTIONS>  compass_noise;
  MeasurementBasePtr<             1,DIM_S,SCALAR,OPTIONS>  altitude;
  noise::NoiseBasePtr<            1,      SCALAR,OPTIONS>  altitude_noise;
  MeasurementBasePtr<             1,DIM_S,SCALAR,OPTIONS>  air_speed;
  noise::NoiseBasePtr<            1,      SCALAR,OPTIONS>  air_speed_noise;
  std::vector<MeasurementBasePtr< 1,DIM_S,SCALAR,OPTIONS>> feature_range;
  std::vector<noise::NoiseBasePtr<1,      SCALAR,OPTIONS>> feature_range_noise;
  std::vector<MeasurementBasePtr< 2,DIM_S,SCALAR,OPTIONS>> feature_bearing;
  std::vector<noise::NoiseBasePtr<2,      SCALAR,OPTIONS>> feature_bearing_noise;
};

template<typename DIM_S, bool USE_SS_KALMAN_GAIN, typename SCALAR, Eigen::StorageOptions OPTIONS>
DubinsAirplaneMeasurementController<DIM_S,USE_SS_KALMAN_GAIN,SCALAR,OPTIONS>::
  DubinsAirplaneMeasurementController(const MeasurementBasePtr<             3,DIM_S,SCALAR,OPTIONS>&  gps_pos,
                                      const noise::NoiseBasePtr<            3,      SCALAR,OPTIONS>&  gps_pos_noise,
                                      const MeasurementBasePtr<             1,DIM_S,SCALAR,OPTIONS>&  gps_heading,
                                      const noise::NoiseBasePtr<            1,      SCALAR,OPTIONS>&  gps_heading_noise,
                                      const MeasurementBasePtr<             1,DIM_S,SCALAR,OPTIONS>&  compass,
                                      const noise::NoiseBasePtr<            1,      SCALAR,OPTIONS>&  compass_noise,
                                      const MeasurementBasePtr<             1,DIM_S,SCALAR,OPTIONS>&  altitude,
                                      const noise::NoiseBasePtr<            1,      SCALAR,OPTIONS>&  altitude_noise,
                                      const MeasurementBasePtr<             1,DIM_S,SCALAR,OPTIONS>&  air_speed,
                                      const noise::NoiseBasePtr<            1,      SCALAR,OPTIONS>&  air_speed_noise,
                                      const std::vector<MeasurementBasePtr< 1,DIM_S,SCALAR,OPTIONS>>& feature_range,
                                      const std::vector<noise::NoiseBasePtr<1,      SCALAR,OPTIONS>>& feature_range_noise,
                                      const std::vector<MeasurementBasePtr< 2,DIM_S,SCALAR,OPTIONS>>& feature_bearing,
                                      const std::vector<noise::NoiseBasePtr<2,      SCALAR,OPTIONS>>& feature_bearing_noise)
 : MeasurementControllerBase<DIM_S,SCALAR,OPTIONS>(),
   gps_pos(gps_pos),
   gps_pos_noise(gps_pos_noise),
   gps_heading(gps_heading),
   gps_heading_noise(gps_heading_noise),
   compass(compass),
   compass_noise(compass_noise),
   altitude(altitude),
   altitude_noise(altitude_noise),
   air_speed(air_speed),
   air_speed_noise(air_speed_noise),
   feature_range(feature_range),
   feature_range_noise(feature_range_noise),
   feature_bearing(feature_bearing),
   feature_bearing_noise(feature_bearing_noise)
{}

template<typename DIM_S, bool USE_SS_KALMAN_GAIN, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS> DubinsAirplaneMeasurementController<DIM_S,USE_SS_KALMAN_GAIN,SCALAR,OPTIONS>::
  applyMeasurements(const map::MappingsBasePtr<DIM_S,SCALAR,OPTIONS>&                                           mappings,
                    const SCALAR                                                                                time,
                    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,               DIM_S::NUM_MEAS_DIM,OPTIONS>>& meas_update_buff,
                    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,               DIM_S::TRUTH_DIM,   OPTIONS>>& truth_state,
                          Eigen::Ref<      Eigen::Matrix<SCALAR,1,               DIM_S::NAV_DIM,     OPTIONS>>  nav_state,
                          Eigen::Ref<      Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,   OPTIONS>>  error_covariance)
{
  Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS> output_meas_update_buff = meas_update_buff;
  Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS> cur_meas_update_buff;

  const size_t features_size = this->feature_range.size();

  do
  {
    cur_meas_update_buff = output_meas_update_buff;
    output_meas_update_buff.setConstant(std::numeric_limits<SCALAR>::quiet_NaN());

  if(this->gps_pos->measurementReady(time, cur_meas_update_buff[DIM_S::NUM_MEAS::GPS_IND], truth_state))
  {
    applyMeasurementMonteCarlo<3,false,DIM_S,USE_SS_KALMAN_GAIN,SCALAR,OPTIONS>(
      this->gps_pos,
      this->gps_pos_noise,
      mappings,
      time,
      truth_state,
      nav_state,
      error_covariance);
    applyMeasurementMonteCarlo<1,true,DIM_S,USE_SS_KALMAN_GAIN,SCALAR,OPTIONS>(
      this->gps_heading,
      this->gps_heading_noise,
      mappings,
      time,
      truth_state,
      nav_state,
      error_covariance);
  }
  output_meas_update_buff[DIM_S::NUM_MEAS::GPS_IND] =
    this->gps_pos->updateNextMeasurementTime(time, cur_meas_update_buff[DIM_S::NUM_MEAS::GPS_IND]);

  if(this->compass->measurementReady(time, cur_meas_update_buff[DIM_S::NUM_MEAS::COMPASS_IND], truth_state))
  {
    applyMeasurementMonteCarlo<1,true,DIM_S,USE_SS_KALMAN_GAIN,SCALAR,OPTIONS>(
      this->compass,
      this->compass_noise,
      mappings,
      time,
      truth_state,
      nav_state,
      error_covariance);
  }
  output_meas_update_buff[DIM_S::NUM_MEAS::COMPASS_IND] =
    this->compass->updateNextMeasurementTime(time, cur_meas_update_buff[DIM_S::NUM_MEAS::COMPASS_IND]);

  if(this->altitude->measurementReady(time, cur_meas_update_buff[DIM_S::NUM_MEAS::ABS_PRESSURE_IND], truth_state))
  {
    applyMeasurementMonteCarlo<1,false,DIM_S,USE_SS_KALMAN_GAIN,SCALAR,OPTIONS>(
      this->altitude,
      this->altitude_noise,
      mappings,
      time,
      truth_state,
      nav_state,
      error_covariance);
  }
  output_meas_update_buff[DIM_S::NUM_MEAS::ABS_PRESSURE_IND] =
    this->altitude->updateNextMeasurementTime(time, cur_meas_update_buff[DIM_S::NUM_MEAS::ABS_PRESSURE_IND]);

  if(this->air_speed->measurementReady(time, cur_meas_update_buff[DIM_S::NUM_MEAS::DIFF_PRESSURE_IND], truth_state))
  {
    applyMeasurementMonteCarlo<1,false,DIM_S,false,SCALAR,OPTIONS>(
      this->air_speed,
      this->air_speed_noise,
      mappings,
      time,
      truth_state,
      nav_state,
      error_covariance);
  }
  output_meas_update_buff[DIM_S::NUM_MEAS::DIFF_PRESSURE_IND] =
    this->air_speed->updateNextMeasurementTime(time, cur_meas_update_buff[DIM_S::NUM_MEAS::DIFF_PRESSURE_IND]);

  for(size_t it = 0; it < features_size; ++it)
  {
    if(this->feature_range[it]->measurementReady(time, cur_meas_update_buff[DIM_S::NUM_MEAS::FEATURE_IND], truth_state))
    {
      applyMeasurementMonteCarlo<2,false,DIM_S,false,SCALAR,OPTIONS>(
        this->feature_bearing[it],
        this->feature_bearing_noise[it],
        mappings,
        time,
        truth_state,
        nav_state,
        error_covariance);
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
  output_meas_update_buff[DIM_S::NUM_MEAS::FEATURE_IND] =
    this->feature_range[0]->updateNextMeasurementTime(time, cur_meas_update_buff[DIM_S::NUM_MEAS::FEATURE_IND]);
  } while(output_meas_update_buff != cur_meas_update_buff);

  return output_meas_update_buff;
}

template<typename DIM_S, bool USE_SS_KALMAN_GAIN, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS> DubinsAirplaneMeasurementController<DIM_S,USE_SS_KALMAN_GAIN,SCALAR,OPTIONS>::
  applyMeasurementsSSErrorCov(const map::MappingsBasePtr<DIM_S,SCALAR,OPTIONS>&                                           mappings,
                              const SCALAR                                                                                time,
                              const Eigen::Ref<const Eigen::Matrix<SCALAR,1,               DIM_S::NUM_MEAS_DIM,OPTIONS>>& meas_update_buff,
                              const Eigen::Ref<const Eigen::Matrix<SCALAR,1,               DIM_S::TRUTH_DIM,   OPTIONS>>& truth_state,
                              const Eigen::Ref<const Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,   OPTIONS>>& ss_error_covariance,
                                    Eigen::Ref<      Eigen::Matrix<SCALAR,1,               DIM_S::NAV_DIM,     OPTIONS>>  nav_state)
{
  Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS> output_meas_update_buff = meas_update_buff;
  Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS> cur_meas_update_buff;

  const size_t features_size = this->feature_range.size();

  do
  {
    cur_meas_update_buff = output_meas_update_buff;
    output_meas_update_buff.setConstant(std::numeric_limits<SCALAR>::quiet_NaN());

  if(this->gps_pos->measurementReady(time, cur_meas_update_buff[DIM_S::NUM_MEAS::GPS_IND], truth_state))
  {
    applyMeasurementMonteCarloSS<3,false,DIM_S,USE_SS_KALMAN_GAIN,SCALAR,OPTIONS>(
      this->gps_pos,
      this->gps_pos_noise,
      mappings,
      time,
      truth_state,
      ss_error_covariance,
      nav_state);
    applyMeasurementMonteCarloSS<1,true,DIM_S,USE_SS_KALMAN_GAIN,SCALAR,OPTIONS>(
      this->gps_heading,
      this->gps_heading_noise,
      mappings,
      time,
      truth_state,
      ss_error_covariance,
      nav_state);
  }
  output_meas_update_buff[DIM_S::NUM_MEAS::GPS_IND] =
    this->gps_pos->updateNextMeasurementTime(time, cur_meas_update_buff[DIM_S::NUM_MEAS::GPS_IND]);

  if(this->compass->measurementReady(time, cur_meas_update_buff[DIM_S::NUM_MEAS::COMPASS_IND], truth_state))
  {
    applyMeasurementMonteCarloSS<1,true,DIM_S,USE_SS_KALMAN_GAIN,SCALAR,OPTIONS>(
      this->compass,
      this->compass_noise,
      mappings,
      time,
      truth_state,
      ss_error_covariance,
      nav_state);
  }
  output_meas_update_buff[DIM_S::NUM_MEAS::COMPASS_IND] =
    this->compass->updateNextMeasurementTime(time, cur_meas_update_buff[DIM_S::NUM_MEAS::COMPASS_IND]);

  if(this->altitude->measurementReady(time, cur_meas_update_buff[DIM_S::NUM_MEAS::ABS_PRESSURE_IND], truth_state))
  {
    applyMeasurementMonteCarloSS<1,false,DIM_S,USE_SS_KALMAN_GAIN,SCALAR,OPTIONS>(
      this->altitude,
      this->altitude_noise,
      mappings,
      time,
      truth_state,
      ss_error_covariance,
      nav_state);
  }
  output_meas_update_buff[DIM_S::NUM_MEAS::ABS_PRESSURE_IND] =
    this->altitude->updateNextMeasurementTime(time, cur_meas_update_buff[DIM_S::NUM_MEAS::ABS_PRESSURE_IND]);

  if(this->air_speed->measurementReady(time, cur_meas_update_buff[DIM_S::NUM_MEAS::DIFF_PRESSURE_IND], truth_state))
  {
    applyMeasurementMonteCarloSS<1,false,DIM_S,false,SCALAR,OPTIONS>(
      this->air_speed,
      this->air_speed_noise,
      mappings,
      time,
      truth_state,
      ss_error_covariance,
      nav_state);
  }
  output_meas_update_buff[DIM_S::NUM_MEAS::DIFF_PRESSURE_IND] =
    this->air_speed->updateNextMeasurementTime(time, cur_meas_update_buff[DIM_S::NUM_MEAS::DIFF_PRESSURE_IND]);

  for(size_t it = 0; it < features_size; ++it)
  {
    if(this->feature_range[it]->measurementReady(time, cur_meas_update_buff[DIM_S::NUM_MEAS::FEATURE_IND], truth_state))
    {
      applyMeasurementMonteCarloSS<2,false,DIM_S,false,SCALAR,OPTIONS>(
        this->feature_bearing[it],
        this->feature_bearing_noise[it],
        mappings,
        time,
        truth_state,
        ss_error_covariance,
        nav_state);
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
  output_meas_update_buff[DIM_S::NUM_MEAS::FEATURE_IND] =
    this->feature_range[0]->updateNextMeasurementTime(time, cur_meas_update_buff[DIM_S::NUM_MEAS::FEATURE_IND]);
  } while(output_meas_update_buff != cur_meas_update_buff);

  return output_meas_update_buff;
}

template<typename DIM_S, bool USE_SS_KALMAN_GAIN, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS> DubinsAirplaneMeasurementController<DIM_S,USE_SS_KALMAN_GAIN,SCALAR,OPTIONS>::
  applyMeasurementsLinCov(const SCALAR                                                                                         time,
                          const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::NUM_MEAS_DIM,   OPTIONS>>& meas_update_buff,
                          const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::TRUTH_DIM,      OPTIONS>>& ref_truth_state,
                          const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::NAV_DIM,        OPTIONS>>& ref_nav_state,
                                Eigen::Ref<      Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,      DIM_S::ERROR_DIM,      OPTIONS>>  error_covariance,
                                Eigen::Ref<      Eigen::Matrix<SCALAR,DIM_S::LINCOV::AUG_DIM,DIM_S::LINCOV::AUG_DIM,OPTIONS>>  aug_covariance)
{
  Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS> output_meas_update_buff = meas_update_buff;
  Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS> cur_meas_update_buff;

  const size_t features_size = this->feature_range.size();

  do
  {
    cur_meas_update_buff = output_meas_update_buff;
    output_meas_update_buff.setConstant(std::numeric_limits<SCALAR>::quiet_NaN());

  if(this->gps_pos->measurementReady(time, cur_meas_update_buff[DIM_S::NUM_MEAS::GPS_IND], ref_truth_state))
  {
    applyMeasurementLinCov<3,DIM_S,USE_SS_KALMAN_GAIN,SCALAR,OPTIONS>(
      this->gps_pos,
      this->gps_pos_noise,
      time,
      ref_truth_state,
      ref_nav_state,
      error_covariance,
      aug_covariance);
    applyMeasurementLinCov<1,DIM_S,USE_SS_KALMAN_GAIN,SCALAR,OPTIONS>(
      this->gps_heading,
      this->gps_heading_noise,
      time,
      ref_truth_state,
      ref_nav_state,
      error_covariance,
      aug_covariance);
  }
  output_meas_update_buff[DIM_S::NUM_MEAS::GPS_IND] =
    this->gps_pos->updateNextMeasurementTime(time, cur_meas_update_buff[DIM_S::NUM_MEAS::GPS_IND]);

  if(this->compass->measurementReady(time, cur_meas_update_buff[DIM_S::NUM_MEAS::COMPASS_IND], ref_truth_state))
  {
    applyMeasurementLinCov<1,DIM_S,USE_SS_KALMAN_GAIN,SCALAR,OPTIONS>(
      this->compass,
      this->compass_noise,
      time,
      ref_truth_state,
      ref_nav_state,
      error_covariance,
      aug_covariance);
  }
  output_meas_update_buff[DIM_S::NUM_MEAS::COMPASS_IND] =
    this->compass->updateNextMeasurementTime(time, cur_meas_update_buff[DIM_S::NUM_MEAS::COMPASS_IND]);

  if(this->altitude->measurementReady(time, cur_meas_update_buff[DIM_S::NUM_MEAS::ABS_PRESSURE_IND], ref_truth_state))
  {
    applyMeasurementLinCov<1,DIM_S,USE_SS_KALMAN_GAIN,SCALAR,OPTIONS>(
      this->altitude,
      this->altitude_noise,
      time,
      ref_truth_state,
      ref_nav_state,
      error_covariance,
      aug_covariance);
  }
  output_meas_update_buff[DIM_S::NUM_MEAS::ABS_PRESSURE_IND] =
    this->altitude->updateNextMeasurementTime(time, cur_meas_update_buff[DIM_S::NUM_MEAS::ABS_PRESSURE_IND]);

  if(this->air_speed->measurementReady(time, cur_meas_update_buff[DIM_S::NUM_MEAS::DIFF_PRESSURE_IND], ref_truth_state))
  {
    applyMeasurementLinCov<1,DIM_S,false,SCALAR,OPTIONS>(
      this->air_speed,
      this->air_speed_noise,
      time,
      ref_truth_state,
      ref_nav_state,
      error_covariance,
      aug_covariance);
  }
  output_meas_update_buff[DIM_S::NUM_MEAS::DIFF_PRESSURE_IND] =
    this->air_speed->updateNextMeasurementTime(time, cur_meas_update_buff[DIM_S::NUM_MEAS::DIFF_PRESSURE_IND]);

  for(size_t it = 0; it < features_size; ++it)
  {
    if(this->feature_range[it]->measurementReady(time, cur_meas_update_buff[DIM_S::NUM_MEAS::FEATURE_IND], ref_truth_state))
    {
      applyMeasurementLinCov<2,DIM_S,false,SCALAR,OPTIONS>(
        this->feature_bearing[it],
        this->feature_bearing_noise[it],
        time,
        ref_truth_state,
        ref_nav_state,
        error_covariance,
        aug_covariance);
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
  output_meas_update_buff[DIM_S::NUM_MEAS::FEATURE_IND] =
    this->feature_range[0]->updateNextMeasurementTime(time, cur_meas_update_buff[DIM_S::NUM_MEAS::FEATURE_IND]);
  } while(output_meas_update_buff != cur_meas_update_buff);

  return output_meas_update_buff;
}

template<typename DIM_S, bool USE_SS_KALMAN_GAIN, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS> DubinsAirplaneMeasurementController<DIM_S,USE_SS_KALMAN_GAIN,SCALAR,OPTIONS>::
  applyMeasurementsErrorBudget(const SCALAR                                                                                         time,
                               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::NUM_MEAS_DIM,   OPTIONS>>& meas_update_buff,
                               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::TRUTH_DIM,      OPTIONS>>& ref_truth_state,
                               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::NAV_DIM,        OPTIONS>>& ref_nav_state,
                               const Eigen::Ref<const Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,      DIM_S::ERROR_DIM,      OPTIONS>>& error_covariance,
                                     Eigen::Ref<      Eigen::Matrix<SCALAR,DIM_S::LINCOV::AUG_DIM,DIM_S::LINCOV::AUG_DIM,OPTIONS>>  aug_covariance)
{
  Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS> output_meas_update_buff = meas_update_buff;
  Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS> cur_meas_update_buff;

  const size_t features_size = this->feature_range.size();

  do
  {
    cur_meas_update_buff = output_meas_update_buff;
    output_meas_update_buff.setConstant(std::numeric_limits<SCALAR>::quiet_NaN());

  if(this->gps_pos->measurementReady(time, cur_meas_update_buff[DIM_S::NUM_MEAS::GPS_IND], ref_truth_state))
  {
    applyMeasurementErrorBudget<3,DIM_S,USE_SS_KALMAN_GAIN,SCALAR,OPTIONS>(
      this->gps_pos,
      this->gps_pos_noise,
      time,
      ref_truth_state,
      ref_nav_state,
      error_covariance,
      aug_covariance);
    applyMeasurementErrorBudget<1,DIM_S,USE_SS_KALMAN_GAIN,SCALAR,OPTIONS>(
      this->gps_heading,
      this->gps_heading_noise,
      time,
      ref_truth_state,
      ref_nav_state,
      error_covariance,
      aug_covariance);
  }
  output_meas_update_buff[DIM_S::NUM_MEAS::GPS_IND] =
    this->gps_pos->updateNextMeasurementTime(time, cur_meas_update_buff[DIM_S::NUM_MEAS::GPS_IND]);

  if(this->compass->measurementReady(time, cur_meas_update_buff[DIM_S::NUM_MEAS::COMPASS_IND], ref_truth_state))
  {
    applyMeasurementErrorBudget<1,DIM_S,USE_SS_KALMAN_GAIN,SCALAR,OPTIONS>(
      this->compass,
      this->compass_noise,
      time,
      ref_truth_state,
      ref_nav_state,
      error_covariance,
      aug_covariance);
  }
  output_meas_update_buff[DIM_S::NUM_MEAS::COMPASS_IND] =
    this->compass->updateNextMeasurementTime(time, cur_meas_update_buff[DIM_S::NUM_MEAS::COMPASS_IND]);

  if(this->altitude->measurementReady(time, cur_meas_update_buff[DIM_S::NUM_MEAS::ABS_PRESSURE_IND], ref_truth_state))
  {
    applyMeasurementErrorBudget<1,DIM_S,USE_SS_KALMAN_GAIN,SCALAR,OPTIONS>(
      this->altitude,
      this->altitude_noise,
      time,
      ref_truth_state,
      ref_nav_state,
      error_covariance,
      aug_covariance);
  }
  output_meas_update_buff[DIM_S::NUM_MEAS::ABS_PRESSURE_IND] =
    this->altitude->updateNextMeasurementTime(time, cur_meas_update_buff[DIM_S::NUM_MEAS::ABS_PRESSURE_IND]);

  if(this->air_speed->measurementReady(time, cur_meas_update_buff[DIM_S::NUM_MEAS::DIFF_PRESSURE_IND], ref_truth_state))
  {
    applyMeasurementErrorBudget<1,DIM_S,false,SCALAR,OPTIONS>(
      this->air_speed,
      this->air_speed_noise,
      time,
      ref_truth_state,
      ref_nav_state,
      error_covariance,
      aug_covariance);
  }
  output_meas_update_buff[DIM_S::NUM_MEAS::DIFF_PRESSURE_IND] =
    this->air_speed->updateNextMeasurementTime(time, cur_meas_update_buff[DIM_S::NUM_MEAS::DIFF_PRESSURE_IND]);

  for(size_t it = 0; it < features_size; ++it)
  {
    if(this->feature_range[it]->measurementReady(time, cur_meas_update_buff[DIM_S::NUM_MEAS::FEATURE_IND], ref_truth_state))
    {
      applyMeasurementErrorBudget<2,DIM_S,false,SCALAR,OPTIONS>(
        this->feature_bearing[it],
        this->feature_bearing_noise[it],
        time,
        ref_truth_state,
        ref_nav_state,
        error_covariance,
        aug_covariance);
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
  output_meas_update_buff[DIM_S::NUM_MEAS::FEATURE_IND] =
    this->feature_range[0]->updateNextMeasurementTime(time, cur_meas_update_buff[DIM_S::NUM_MEAS::FEATURE_IND]);
  } while(output_meas_update_buff != cur_meas_update_buff);

  return output_meas_update_buff;
}
} // namespace sensors
} // namespace kf

#endif
/* dubins_airplane_measurement_controller.hpp */
