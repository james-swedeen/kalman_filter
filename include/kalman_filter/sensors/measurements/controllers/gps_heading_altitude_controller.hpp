/**
 * @File: gps_heading_altitude_controller.hpp
 * @Date: May 2022
 * @Author: James Swedeen
 *
 * @brief
 * A class for controlling when the GPS, Heading, and Altitude measurements are applied.
 **/

#ifndef KALMAN_FILTER_SENSORS_MEASUREMENTS_CONTROLLERS_GPS_HEADING_ALTITUDE_CONTROLLER_HPP
#define KALMAN_FILTER_SENSORS_MEASUREMENTS_CONTROLLERS_GPS_HEADING_ALTITUDE_CONTROLLER_HPP

/* C++ Headers */
#include<cstdint>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<kalman_filter/mappings/mappings_base.hpp>
#include<kalman_filter/noise/noise_base.hpp>
#include<kalman_filter/helpers/measurement_update.hpp>
#include<kalman_filter/sensors/measurements/controllers/measurement_controller_base.hpp>
#include<kalman_filter/sensors/measurements/measurement_base.hpp>

namespace kf
{
namespace sensors
{
template<typename DIM_S, bool USE_SS_KALMAN_GAIN, typename SCALAR, Eigen::StorageOptions OPTIONS>
class GPSHeadingAltitudeController;

template<typename DIM_S, bool USE_SS_KALMAN_GAIN = false, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using GPSHeadingAltitudeControllerPtr = std::shared_ptr<GPSHeadingAltitudeController<DIM_S,USE_SS_KALMAN_GAIN,SCALAR,OPTIONS>>;

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
class GPSHeadingAltitudeController
 : public MeasurementControllerBase<DIM_S,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  GPSHeadingAltitudeController() = delete;
  /**
   * @Copy Constructor
   **/
  GPSHeadingAltitudeController(const GPSHeadingAltitudeController&) = default;
  /**
   * @Move Constructor
   **/
  GPSHeadingAltitudeController(GPSHeadingAltitudeController&&) = default;
  /**
   * @Constructor
   *
   * @brief
   * Initializes the object for use.
   *
   * @parameters
   * gps: The gps measurement object
   * gps_noise: The gps measurement noise
   * heading: The heading measurement object
   * heading_noise: The heading measurement noise
   * altitude: The altitude measurement object
   * altitude_noise: The altitude measurement noise
   **/
  GPSHeadingAltitudeController(const MeasurementBasePtr< 3,DIM_S,SCALAR,OPTIONS>& gps,
                               const noise::NoiseBasePtr<3,      SCALAR,OPTIONS>& gps_noise,
                               const MeasurementBasePtr< 1,DIM_S,SCALAR,OPTIONS>& heading,
                               const noise::NoiseBasePtr<1,      SCALAR,OPTIONS>& heading_noise,
                               const MeasurementBasePtr< 1,DIM_S,SCALAR,OPTIONS>& altitude,
                               const noise::NoiseBasePtr<1,      SCALAR,OPTIONS>& altitude_noise);
  /**
   * @Deconstructor
   **/
  ~GPSHeadingAltitudeController() override = default;
  /**
   * @Assignment Operators
   **/
  GPSHeadingAltitudeController& operator=(const GPSHeadingAltitudeController&)  = default;
  GPSHeadingAltitudeController& operator=(      GPSHeadingAltitudeController&&) = default;
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
private:
  MeasurementBasePtr< 3,DIM_S,SCALAR,OPTIONS> gps;
  noise::NoiseBasePtr<3,      SCALAR,OPTIONS> gps_noise;
  MeasurementBasePtr< 1,DIM_S,SCALAR,OPTIONS> heading;
  noise::NoiseBasePtr<1,      SCALAR,OPTIONS> heading_noise;
  MeasurementBasePtr< 1,DIM_S,SCALAR,OPTIONS> altitude;
  noise::NoiseBasePtr<1,      SCALAR,OPTIONS> altitude_noise;
};

template<typename DIM_S, bool USE_SS_KALMAN_GAIN, typename SCALAR, Eigen::StorageOptions OPTIONS>
GPSHeadingAltitudeController<DIM_S,USE_SS_KALMAN_GAIN,SCALAR,OPTIONS>::
  GPSHeadingAltitudeController(const MeasurementBasePtr<3,DIM_S,SCALAR,OPTIONS>& gps,
                               const noise::NoiseBasePtr<3,      SCALAR,OPTIONS>& gps_noise,
                               const MeasurementBasePtr< 1,DIM_S,SCALAR,OPTIONS>& heading,
                               const noise::NoiseBasePtr<1,      SCALAR,OPTIONS>& heading_noise,
                               const MeasurementBasePtr< 1,DIM_S,SCALAR,OPTIONS>& altitude,
                               const noise::NoiseBasePtr<1,      SCALAR,OPTIONS>& altitude_noise)
 : MeasurementControllerBase<DIM_S,SCALAR,OPTIONS>(),
   gps(gps),
   gps_noise(gps_noise),
   heading(heading),
   heading_noise(heading_noise),
   altitude(altitude),
   altitude_noise(altitude_noise)
{}

template<typename DIM_S, bool USE_SS_KALMAN_GAIN, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS> GPSHeadingAltitudeController<DIM_S,USE_SS_KALMAN_GAIN,SCALAR,OPTIONS>::
  applyMeasurements(const map::MappingsBasePtr<DIM_S,SCALAR,OPTIONS>&                                           mappings,
                    const SCALAR                                                                                time,
                    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,               DIM_S::NUM_MEAS_DIM,OPTIONS>>& meas_update_buff,
                    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,               DIM_S::TRUTH_DIM,   OPTIONS>>& truth_state,
                          Eigen::Ref<      Eigen::Matrix<SCALAR,1,               DIM_S::NAV_DIM,     OPTIONS>>  nav_state,
                          Eigen::Ref<      Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,   OPTIONS>>  error_covariance)
{
  Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS> output_meas_update_buff;

  if(this->gps->measurementReady(time, meas_update_buff[DIM_S::NUM_MEAS::GPS_IND], truth_state))
  {
    applyMeasurementMonteCarlo<3,false,DIM_S,USE_SS_KALMAN_GAIN,SCALAR,OPTIONS>(
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

  if(this->heading->measurementReady(time, meas_update_buff[DIM_S::NUM_MEAS::HEADING_IND], truth_state))
  {
    applyMeasurementMonteCarlo<1,true,DIM_S,USE_SS_KALMAN_GAIN,SCALAR,OPTIONS>(
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

  if(this->altitude->measurementReady(time, meas_update_buff[DIM_S::NUM_MEAS::ALTITUDE_IND], truth_state))
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
  output_meas_update_buff[DIM_S::NUM_MEAS::ALTITUDE_IND] =
    this->altitude->updateNextMeasurementTime(time, meas_update_buff[DIM_S::NUM_MEAS::ALTITUDE_IND]);

  return output_meas_update_buff;
}

template<typename DIM_S, bool USE_SS_KALMAN_GAIN, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS> GPSHeadingAltitudeController<DIM_S,USE_SS_KALMAN_GAIN,SCALAR,OPTIONS>::
  applyMeasurementsSSErrorCov(const map::MappingsBasePtr<DIM_S,SCALAR,OPTIONS>&                                           mappings,
                              const SCALAR                                                                                time,
                              const Eigen::Ref<const Eigen::Matrix<SCALAR,1,               DIM_S::NUM_MEAS_DIM,OPTIONS>>& meas_update_buff,
                              const Eigen::Ref<const Eigen::Matrix<SCALAR,1,               DIM_S::TRUTH_DIM,   OPTIONS>>& truth_state,
                              const Eigen::Ref<const Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,   OPTIONS>>& ss_error_covariance,
                                    Eigen::Ref<      Eigen::Matrix<SCALAR,1,               DIM_S::NAV_DIM,     OPTIONS>>  nav_state)
{
  Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS> output_meas_update_buff;

  if(this->gps->measurementReady(time, meas_update_buff[DIM_S::NUM_MEAS::GPS_IND], truth_state))
  {
    applyMeasurementMonteCarloSS<3,false,DIM_S,USE_SS_KALMAN_GAIN,SCALAR,OPTIONS>(
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

  if(this->heading->measurementReady(time, meas_update_buff[DIM_S::NUM_MEAS::HEADING_IND], truth_state))
  {
    applyMeasurementMonteCarloSS<1,true,DIM_S,USE_SS_KALMAN_GAIN,SCALAR,OPTIONS>(
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

  if(this->altitude->measurementReady(time, meas_update_buff[DIM_S::NUM_MEAS::ALTITUDE_IND], truth_state))
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
  output_meas_update_buff[DIM_S::NUM_MEAS::ALTITUDE_IND] =
    this->altitude->updateNextMeasurementTime(time, meas_update_buff[DIM_S::NUM_MEAS::ALTITUDE_IND]);

  return output_meas_update_buff;
}

template<typename DIM_S, bool USE_SS_KALMAN_GAIN, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS> GPSHeadingAltitudeController<DIM_S,USE_SS_KALMAN_GAIN,SCALAR,OPTIONS>::
  applyMeasurementsLinCov(const SCALAR                                                                                         time,
                          const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::NUM_MEAS_DIM,   OPTIONS>>& meas_update_buff,
                          const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::TRUTH_DIM,      OPTIONS>>& ref_truth_state,
                          const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::NAV_DIM,        OPTIONS>>& ref_nav_state,
                                Eigen::Ref<      Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,      DIM_S::ERROR_DIM,      OPTIONS>>  error_covariance,
                                Eigen::Ref<      Eigen::Matrix<SCALAR,DIM_S::LINCOV::AUG_DIM,DIM_S::LINCOV::AUG_DIM,OPTIONS>>  aug_covariance)
{
  Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS> output_meas_update_buff;

  if(this->gps->measurementReady(time, meas_update_buff[DIM_S::NUM_MEAS::GPS_IND], ref_truth_state))
  {
    applyMeasurementLinCov<3,DIM_S,USE_SS_KALMAN_GAIN,SCALAR,OPTIONS>(
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

  if(this->heading->measurementReady(time, meas_update_buff[DIM_S::NUM_MEAS::HEADING_IND], ref_truth_state))
  {
    applyMeasurementLinCov<1,DIM_S,USE_SS_KALMAN_GAIN,SCALAR,OPTIONS>(
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

  if(this->altitude->measurementReady(time, meas_update_buff[DIM_S::NUM_MEAS::ALTITUDE_IND], ref_truth_state))
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
  output_meas_update_buff[DIM_S::NUM_MEAS::ALTITUDE_IND] =
    this->altitude->updateNextMeasurementTime(time, meas_update_buff[DIM_S::NUM_MEAS::ALTITUDE_IND]);

  return output_meas_update_buff;
}

template<typename DIM_S, bool USE_SS_KALMAN_GAIN, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS> GPSHeadingAltitudeController<DIM_S,USE_SS_KALMAN_GAIN,SCALAR,OPTIONS>::
  applyMeasurementsErrorBudget(const SCALAR                                                                                         time,
                               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::NUM_MEAS_DIM,   OPTIONS>>& meas_update_buff,
                               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::TRUTH_DIM,      OPTIONS>>& ref_truth_state,
                               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::NAV_DIM,        OPTIONS>>& ref_nav_state,
                               const Eigen::Ref<const Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,      DIM_S::ERROR_DIM,      OPTIONS>>& error_covariance,
                                     Eigen::Ref<      Eigen::Matrix<SCALAR,DIM_S::LINCOV::AUG_DIM,DIM_S::LINCOV::AUG_DIM,OPTIONS>>  aug_covariance)
{
  Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS> output_meas_update_buff;

  if(this->gps->measurementReady(time, meas_update_buff[DIM_S::NUM_MEAS::GPS_IND], ref_truth_state))
  {
    applyMeasurementErrorBudget<3,DIM_S,USE_SS_KALMAN_GAIN,SCALAR,OPTIONS>(
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

  if(this->heading->measurementReady(time, meas_update_buff[DIM_S::NUM_MEAS::HEADING_IND], ref_truth_state))
  {
    applyMeasurementErrorBudget<1,DIM_S,USE_SS_KALMAN_GAIN,SCALAR,OPTIONS>(
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

  if(this->altitude->measurementReady(time, meas_update_buff[DIM_S::NUM_MEAS::ALTITUDE_IND], ref_truth_state))
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
  output_meas_update_buff[DIM_S::NUM_MEAS::ALTITUDE_IND] =
    this->altitude->updateNextMeasurementTime(time, meas_update_buff[DIM_S::NUM_MEAS::ALTITUDE_IND]);

  return output_meas_update_buff;
}
} // namespace sensors
} // namespace kf

#endif
/* gps_heading_altitude_controller.hpp */
