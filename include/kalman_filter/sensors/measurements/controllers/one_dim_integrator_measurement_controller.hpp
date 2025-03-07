/**
 * @File: one_dim_integrator_measurement_controller.hpp
 * @Date: May 2022
 * @Author: James Swedeen
 *
 * @brief
 **/

#ifndef KALMAN_FILTER_SENSORS_MEASUREMENTS_CONTROLLERS_ONE_DIM_INTEGRATOR_MEASUREMENT_CONTROLLER_HPP
#define KALMAN_FILTER_SENSORS_MEASUREMENTS_CONTROLLERS_ONE_DIM_INTEGRATOR_MEASUREMENT_CONTROLLER_HPP

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
#include<kalman_filter/dynamics/one_dim_integrator.hpp>

namespace kf
{
namespace sensors
{
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
class OneDimIntegratorMeasurementController;

template<typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using OneDimIntegratorMeasurementControllerPtr = std::shared_ptr<OneDimIntegratorMeasurementController<SCALAR,OPTIONS>>;

/**
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options.
 **/
template<typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class OneDimIntegratorMeasurementController
 : public MeasurementControllerBase<dynamics::OneDimIntegratorDim,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  OneDimIntegratorMeasurementController() = delete;
  /**
   * @Copy Constructor
   **/
  OneDimIntegratorMeasurementController(const OneDimIntegratorMeasurementController&) noexcept = default;
  /**
   * @Move Constructor
   **/
  OneDimIntegratorMeasurementController(OneDimIntegratorMeasurementController&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * Initializes the object for use.
   *
   * @parameters
   * position_measurement: The position measurement
   * position_measurement_noise: The position measurement noise
   * velocity_measurement: The velocity measurement
   * velocity_measurement_noise: The velocity measurement noise
   **/
  OneDimIntegratorMeasurementController(const MeasurementBasePtr< 1,dynamics::OneDimIntegratorDim,SCALAR,OPTIONS>& position_measurement,
                                        const noise::NoiseBasePtr<1,                              SCALAR,OPTIONS>& position_measurement_noise,
                                        const MeasurementBasePtr< 1,dynamics::OneDimIntegratorDim,SCALAR,OPTIONS>& velocity_measurement,
                                        const noise::NoiseBasePtr<1,                              SCALAR,OPTIONS>& velocity_measurement_noise);
  /**
   * @Deconstructor
   **/
  ~OneDimIntegratorMeasurementController() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  OneDimIntegratorMeasurementController& operator=(const OneDimIntegratorMeasurementController&)  noexcept = default;
  OneDimIntegratorMeasurementController& operator=(      OneDimIntegratorMeasurementController&&) noexcept = default;
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
  inline Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::NUM_MEAS_DIM,OPTIONS>
    applyMeasurements(const map::MappingsBasePtr<dynamics::OneDimIntegratorDim,SCALAR,OPTIONS>&                                                                   mappings,
                      const SCALAR                                                                                                                                time,
                      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                                       dynamics::OneDimIntegratorDim::NUM_MEAS_DIM,OPTIONS>>& meas_update_buff,
                      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                                       dynamics::OneDimIntegratorDim::TRUTH_DIM,   OPTIONS>>& truth_state,
                            Eigen::Ref<      Eigen::Matrix<SCALAR,1,                                       dynamics::OneDimIntegratorDim::NAV_DIM,     OPTIONS>>  nav_state,
                            Eigen::Ref<      Eigen::Matrix<SCALAR,dynamics::OneDimIntegratorDim::ERROR_DIM,dynamics::OneDimIntegratorDim::ERROR_DIM,   OPTIONS>>  error_covariance) override;
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
  inline Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::NUM_MEAS_DIM,OPTIONS>
    applyMeasurementsSSErrorCov(const map::MappingsBasePtr<dynamics::OneDimIntegratorDim,SCALAR,OPTIONS>&                                                                   mappings,
                                const SCALAR                                                                                                                                time,
                                const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                                       dynamics::OneDimIntegratorDim::NUM_MEAS_DIM,OPTIONS>>& meas_update_buff,
                                const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                                       dynamics::OneDimIntegratorDim::TRUTH_DIM,   OPTIONS>>& truth_state,
                                const Eigen::Ref<const Eigen::Matrix<SCALAR,dynamics::OneDimIntegratorDim::ERROR_DIM,dynamics::OneDimIntegratorDim::ERROR_DIM,   OPTIONS>>& ss_error_covariance,
                                      Eigen::Ref<      Eigen::Matrix<SCALAR,1,                                       dynamics::OneDimIntegratorDim::NAV_DIM,     OPTIONS>>  nav_state) override;
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
  inline Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::NUM_MEAS_DIM,OPTIONS>
    applyMeasurementsLinCov(const SCALAR                                                                                                                                         time,
                            const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                                             dynamics::OneDimIntegratorDim::NUM_MEAS_DIM,   OPTIONS>>& meas_update_buff,
                            const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                                             dynamics::OneDimIntegratorDim::TRUTH_DIM,      OPTIONS>>& ref_truth_state,
                            const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                                             dynamics::OneDimIntegratorDim::NAV_DIM,        OPTIONS>>& ref_nav_state,
                                  Eigen::Ref<      Eigen::Matrix<SCALAR,dynamics::OneDimIntegratorDim::ERROR_DIM,      dynamics::OneDimIntegratorDim::ERROR_DIM,      OPTIONS>>  error_covariance,
                                  Eigen::Ref<      Eigen::Matrix<SCALAR,dynamics::OneDimIntegratorDim::LINCOV::AUG_DIM,dynamics::OneDimIntegratorDim::LINCOV::AUG_DIM,OPTIONS>>  aug_covariance) override;
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
  inline Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::NUM_MEAS_DIM,OPTIONS>
    applyMeasurementsErrorBudget(const SCALAR                                                                                                                                         time,
                                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                                             dynamics::OneDimIntegratorDim::NUM_MEAS_DIM,   OPTIONS>>& meas_update_buff,
                                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                                             dynamics::OneDimIntegratorDim::TRUTH_DIM,      OPTIONS>>& ref_truth_state,
                                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                                             dynamics::OneDimIntegratorDim::NAV_DIM,        OPTIONS>>& ref_nav_state,
                                 const Eigen::Ref<const Eigen::Matrix<SCALAR,dynamics::OneDimIntegratorDim::ERROR_DIM,      dynamics::OneDimIntegratorDim::ERROR_DIM,      OPTIONS>>& error_covariance,
                                       Eigen::Ref<      Eigen::Matrix<SCALAR,dynamics::OneDimIntegratorDim::LINCOV::AUG_DIM,dynamics::OneDimIntegratorDim::LINCOV::AUG_DIM,OPTIONS>>  aug_covariance) override;
private:
  MeasurementBasePtr< 1,dynamics::OneDimIntegratorDim,SCALAR,OPTIONS> position_measurement;
  noise::NoiseBasePtr<1,                              SCALAR,OPTIONS> position_measurement_noise;
  MeasurementBasePtr< 1,dynamics::OneDimIntegratorDim,SCALAR,OPTIONS> velocity_measurement;
  noise::NoiseBasePtr<1,                              SCALAR,OPTIONS> velocity_measurement_noise;
};

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
OneDimIntegratorMeasurementController<SCALAR,OPTIONS>::
  OneDimIntegratorMeasurementController(const MeasurementBasePtr< 1,dynamics::OneDimIntegratorDim,SCALAR,OPTIONS>& position_measurement,
                                        const noise::NoiseBasePtr<1,                              SCALAR,OPTIONS>& position_measurement_noise,
                                        const MeasurementBasePtr< 1,dynamics::OneDimIntegratorDim,SCALAR,OPTIONS>& velocity_measurement,
                                        const noise::NoiseBasePtr<1,                              SCALAR,OPTIONS>& velocity_measurement_noise)
 : MeasurementControllerBase<dynamics::OneDimIntegratorDim,SCALAR,OPTIONS>(),
   position_measurement(position_measurement),
   position_measurement_noise(position_measurement_noise),
   velocity_measurement(velocity_measurement),
   velocity_measurement_noise(velocity_measurement_noise)
{}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::NUM_MEAS_DIM,OPTIONS> OneDimIntegratorMeasurementController<SCALAR,OPTIONS>::
  applyMeasurements(const map::MappingsBasePtr<dynamics::OneDimIntegratorDim,SCALAR,OPTIONS>&                                                                   mappings,
                    const SCALAR                                                                                                                                time,
                    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                                       dynamics::OneDimIntegratorDim::NUM_MEAS_DIM,OPTIONS>>& meas_update_buff,
                    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                                       dynamics::OneDimIntegratorDim::TRUTH_DIM,   OPTIONS>>& truth_state,
                          Eigen::Ref<      Eigen::Matrix<SCALAR,1,                                       dynamics::OneDimIntegratorDim::NAV_DIM,     OPTIONS>>  nav_state,
                          Eigen::Ref<      Eigen::Matrix<SCALAR,dynamics::OneDimIntegratorDim::ERROR_DIM,dynamics::OneDimIntegratorDim::ERROR_DIM,   OPTIONS>>  error_covariance)
{
  Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::NUM_MEAS_DIM,OPTIONS> output_meas_update_buff;

  if(this->position_measurement->measurementReady(time, meas_update_buff[dynamics::OneDimIntegratorDim::NUM_MEAS::POS_IND], truth_state))
  {
    applyMeasurementMonteCarlo<1,false,dynamics::OneDimIntegratorDim,false,SCALAR,OPTIONS>(
      this->position_measurement,
      this->position_measurement_noise,
      mappings,
      time,
      truth_state,
      nav_state,
      error_covariance);
  }
  output_meas_update_buff[dynamics::OneDimIntegratorDim::NUM_MEAS::POS_IND] =
    this->position_measurement->updateNextMeasurementTime(time, meas_update_buff[dynamics::OneDimIntegratorDim::NUM_MEAS::POS_IND]);

  if(this->velocity_measurement->measurementReady(time, meas_update_buff[dynamics::OneDimIntegratorDim::NUM_MEAS::VEL_IND], truth_state))
  {
    applyMeasurementMonteCarlo<1,false,dynamics::OneDimIntegratorDim,false,SCALAR,OPTIONS>(
      this->velocity_measurement,
      this->velocity_measurement_noise,
      mappings,
      time,
      truth_state,
      nav_state,
      error_covariance);
  }
  output_meas_update_buff[dynamics::OneDimIntegratorDim::NUM_MEAS::VEL_IND] =
    this->velocity_measurement->updateNextMeasurementTime(time, meas_update_buff[dynamics::OneDimIntegratorDim::NUM_MEAS::VEL_IND]);

  return output_meas_update_buff;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::NUM_MEAS_DIM,OPTIONS> OneDimIntegratorMeasurementController<SCALAR,OPTIONS>::
  applyMeasurementsSSErrorCov(const map::MappingsBasePtr<dynamics::OneDimIntegratorDim,SCALAR,OPTIONS>&                                                                   mappings,
                              const SCALAR                                                                                                                                time,
                              const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                                       dynamics::OneDimIntegratorDim::NUM_MEAS_DIM,OPTIONS>>& meas_update_buff,
                              const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                                       dynamics::OneDimIntegratorDim::TRUTH_DIM,   OPTIONS>>& truth_state,
                              const Eigen::Ref<const Eigen::Matrix<SCALAR,dynamics::OneDimIntegratorDim::ERROR_DIM,dynamics::OneDimIntegratorDim::ERROR_DIM,   OPTIONS>>& ss_error_covariance,
                                    Eigen::Ref<      Eigen::Matrix<SCALAR,1,                                       dynamics::OneDimIntegratorDim::NAV_DIM,     OPTIONS>>  nav_state)
{
  Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::NUM_MEAS_DIM,OPTIONS> output_meas_update_buff;

  if(this->position_measurement->measurementReady(time, meas_update_buff[dynamics::OneDimIntegratorDim::NUM_MEAS::POS_IND], truth_state))
  {
    applyMeasurementMonteCarloSS<1,false,dynamics::OneDimIntegratorDim,false,SCALAR,OPTIONS>(
      this->position_measurement,
      this->position_measurement_noise,
      mappings,
      time,
      truth_state,
      ss_error_covariance,
      nav_state);
  }
  output_meas_update_buff[dynamics::OneDimIntegratorDim::NUM_MEAS::POS_IND] =
    this->position_measurement->updateNextMeasurementTime(time, meas_update_buff[dynamics::OneDimIntegratorDim::NUM_MEAS::POS_IND]);

  if(this->velocity_measurement->measurementReady(time, meas_update_buff[dynamics::OneDimIntegratorDim::NUM_MEAS::VEL_IND], truth_state))
  {
    applyMeasurementMonteCarloSS<1,false,dynamics::OneDimIntegratorDim,false,SCALAR,OPTIONS>(
      this->velocity_measurement,
      this->velocity_measurement_noise,
      mappings,
      time,
      truth_state,
      ss_error_covariance,
      nav_state);
  }
  output_meas_update_buff[dynamics::OneDimIntegratorDim::NUM_MEAS::VEL_IND] =
    this->velocity_measurement->updateNextMeasurementTime(time, meas_update_buff[dynamics::OneDimIntegratorDim::NUM_MEAS::VEL_IND]);

  return output_meas_update_buff;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::NUM_MEAS_DIM,OPTIONS> OneDimIntegratorMeasurementController<SCALAR,OPTIONS>::
  applyMeasurementsLinCov(const SCALAR                                                                                                                                         time,
                          const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                                             dynamics::OneDimIntegratorDim::NUM_MEAS_DIM,   OPTIONS>>& meas_update_buff,
                          const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                                             dynamics::OneDimIntegratorDim::TRUTH_DIM,      OPTIONS>>& ref_truth_state,
                          const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                                             dynamics::OneDimIntegratorDim::NAV_DIM,        OPTIONS>>& ref_nav_state,
                                Eigen::Ref<      Eigen::Matrix<SCALAR,dynamics::OneDimIntegratorDim::ERROR_DIM,      dynamics::OneDimIntegratorDim::ERROR_DIM,      OPTIONS>>  error_covariance,
                                Eigen::Ref<      Eigen::Matrix<SCALAR,dynamics::OneDimIntegratorDim::LINCOV::AUG_DIM,dynamics::OneDimIntegratorDim::LINCOV::AUG_DIM,OPTIONS>>  aug_covariance)
{
  Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::NUM_MEAS_DIM,OPTIONS> output_meas_update_buff;

  if(this->position_measurement->measurementReady(time, meas_update_buff[dynamics::OneDimIntegratorDim::NUM_MEAS::POS_IND], ref_truth_state))
  {
    applyMeasurementLinCov<1,dynamics::OneDimIntegratorDim,false,SCALAR,OPTIONS>(
      this->position_measurement,
      this->position_measurement_noise,
      time,
      ref_truth_state,
      ref_nav_state,
      error_covariance,
      aug_covariance);
  }
  output_meas_update_buff[dynamics::OneDimIntegratorDim::NUM_MEAS::POS_IND] =
    this->position_measurement->updateNextMeasurementTime(time, meas_update_buff[dynamics::OneDimIntegratorDim::NUM_MEAS::POS_IND]);

  if(this->velocity_measurement->measurementReady(time, meas_update_buff[dynamics::OneDimIntegratorDim::NUM_MEAS::VEL_IND], ref_truth_state))
  {
    applyMeasurementLinCov<1,dynamics::OneDimIntegratorDim,false,SCALAR,OPTIONS>(
      this->velocity_measurement,
      this->velocity_measurement_noise,
      time,
      ref_truth_state,
      ref_nav_state,
      error_covariance,
      aug_covariance);
  }
  output_meas_update_buff[dynamics::OneDimIntegratorDim::NUM_MEAS::VEL_IND] =
    this->velocity_measurement->updateNextMeasurementTime(time, meas_update_buff[dynamics::OneDimIntegratorDim::NUM_MEAS::VEL_IND]);

  return output_meas_update_buff;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::NUM_MEAS_DIM,OPTIONS> OneDimIntegratorMeasurementController<SCALAR,OPTIONS>::
  applyMeasurementsErrorBudget(const SCALAR                                                                                                                                         time,
                               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                                             dynamics::OneDimIntegratorDim::NUM_MEAS_DIM,   OPTIONS>>& meas_update_buff,
                               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                                             dynamics::OneDimIntegratorDim::TRUTH_DIM,      OPTIONS>>& ref_truth_state,
                               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                                             dynamics::OneDimIntegratorDim::NAV_DIM,        OPTIONS>>& ref_nav_state,
                               const Eigen::Ref<const Eigen::Matrix<SCALAR,dynamics::OneDimIntegratorDim::ERROR_DIM,      dynamics::OneDimIntegratorDim::ERROR_DIM,      OPTIONS>>& error_covariance,
                                     Eigen::Ref<      Eigen::Matrix<SCALAR,dynamics::OneDimIntegratorDim::LINCOV::AUG_DIM,dynamics::OneDimIntegratorDim::LINCOV::AUG_DIM,OPTIONS>>  aug_covariance)
{
  Eigen::Matrix<SCALAR,1,dynamics::OneDimIntegratorDim::NUM_MEAS_DIM,OPTIONS> output_meas_update_buff;

  if(this->position_measurement->measurementReady(time, meas_update_buff[dynamics::OneDimIntegratorDim::NUM_MEAS::POS_IND], ref_truth_state))
  {
    applyMeasurementErrorBudget<1,dynamics::OneDimIntegratorDim,false,SCALAR,OPTIONS>(
      this->position_measurement,
      this->position_measurement_noise,
      time,
      ref_truth_state,
      ref_nav_state,
      error_covariance,
      aug_covariance);
  }
  output_meas_update_buff[dynamics::OneDimIntegratorDim::NUM_MEAS::POS_IND] =
    this->position_measurement->updateNextMeasurementTime(time, meas_update_buff[dynamics::OneDimIntegratorDim::NUM_MEAS::POS_IND]);

  if(this->velocity_measurement->measurementReady(time, meas_update_buff[dynamics::OneDimIntegratorDim::NUM_MEAS::VEL_IND], ref_truth_state))
  {
    applyMeasurementErrorBudget<1,dynamics::OneDimIntegratorDim,false,SCALAR,OPTIONS>(
      this->velocity_measurement,
      this->velocity_measurement_noise,
      time,
      ref_truth_state,
      ref_nav_state,
      error_covariance,
      aug_covariance);
  }
  output_meas_update_buff[dynamics::OneDimIntegratorDim::NUM_MEAS::VEL_IND] =
    this->velocity_measurement->updateNextMeasurementTime(time, meas_update_buff[dynamics::OneDimIntegratorDim::NUM_MEAS::VEL_IND]);

  return output_meas_update_buff;
}
} // namespace sensors
} // namespace kf

#endif
/* one_dim_integrator_measurement_controller.hpp */
