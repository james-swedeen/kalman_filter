/**
 * @File: no_sensors_controller.hpp
 * @Date: March 2023
 * @Author: James Swedeen
 *
 * @brief
 * A class for The case when there are no measurements to process.
 **/

#ifndef KALMAN_FILTER_SENSORS_MEASUREMENTS_CONTROLLERS_NO_SENEORS_CONTROLLER_HPP
#define KALMAN_FILTER_SENSORS_MEASUREMENTS_CONTROLLERS_NO_SENEORS_CONTROLLER_HPP

/* C++ Headers */
#include<cstdint>
#include<utility>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<kalman_filter/sensors/measurements/controllers/measurement_controller_base.hpp>

namespace kf
{
namespace sensors
{
template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
class NoSensorsController;

template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using NoSensorsControllerPtr = std::shared_ptr<NoSensorsController<DIM_S,SCALAR,OPTIONS>>;

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
class NoSensorsController
 : public MeasurementControllerBase<DIM_S,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  NoSensorsController() = default;
  /**
   * @Copy Constructor
   **/
  NoSensorsController(const NoSensorsController&) = default;
  /**
   * @Move Constructor
   **/
  NoSensorsController(NoSensorsController&&) = default;
  /**
   * @Deconstructor
   **/
  ~NoSensorsController() override = default;
  /**
   * @Assignment Operators
   **/
  NoSensorsController& operator=(const NoSensorsController&)  = default;
  NoSensorsController& operator=(      NoSensorsController&&) = default;
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
};

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS> NoSensorsController<DIM_S,SCALAR,OPTIONS>::
  applyMeasurements(const map::MappingsBasePtr<DIM_S,SCALAR,OPTIONS>&                                           /* mappings */,
                    const SCALAR                                                                                /* time */,
                    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,               DIM_S::NUM_MEAS_DIM,OPTIONS>>& /* meas_update_buff */,
                    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,               DIM_S::TRUTH_DIM,   OPTIONS>>& /* truth_state */,
                          Eigen::Ref<      Eigen::Matrix<SCALAR,1,               DIM_S::NAV_DIM,     OPTIONS>>  /* nav_state */,
                          Eigen::Ref<      Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,   OPTIONS>>  /* error_covariance */)
{
  return Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS>::Constant(std::numeric_limits<SCALAR>::quiet_NaN());
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS> NoSensorsController<DIM_S,SCALAR,OPTIONS>::
  applyMeasurementsSSErrorCov(const map::MappingsBasePtr<DIM_S,SCALAR,OPTIONS>&                                           /* mappings */,
                              const SCALAR                                                                                /* time */,
                              const Eigen::Ref<const Eigen::Matrix<SCALAR,1,               DIM_S::NUM_MEAS_DIM,OPTIONS>>& /* meas_update_buff */,
                              const Eigen::Ref<const Eigen::Matrix<SCALAR,1,               DIM_S::TRUTH_DIM,   OPTIONS>>& /* truth_state */,
                              const Eigen::Ref<const Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,   OPTIONS>>& /* ss_error_covariance */,
                                    Eigen::Ref<      Eigen::Matrix<SCALAR,1,               DIM_S::NAV_DIM,     OPTIONS>>  /* nav_state */)
{
  return Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS>::Constant(std::numeric_limits<SCALAR>::quiet_NaN());
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS> NoSensorsController<DIM_S,SCALAR,OPTIONS>::
  applyMeasurementsLinCov(const SCALAR                                                                                         /* time */,
                          const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::NUM_MEAS_DIM,   OPTIONS>>& /* meas_update_buff */,
                          const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::TRUTH_DIM,      OPTIONS>>& /* ref_truth_state */,
                          const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::NAV_DIM,        OPTIONS>>& /* ref_nav_state */,
                                Eigen::Ref<      Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,      DIM_S::ERROR_DIM,      OPTIONS>>  /* error_covariance */,
                                Eigen::Ref<      Eigen::Matrix<SCALAR,DIM_S::LINCOV::AUG_DIM,DIM_S::LINCOV::AUG_DIM,OPTIONS>>  /* aug_covariance */)
{
  return Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS>::Constant(std::numeric_limits<SCALAR>::quiet_NaN());
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS> NoSensorsController<DIM_S,SCALAR,OPTIONS>::
  applyMeasurementsErrorBudget(const SCALAR                                                                                         /* time */,
                               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::NUM_MEAS_DIM,   OPTIONS>>& /* meas_update_buff */,
                               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::TRUTH_DIM,      OPTIONS>>& /* ref_truth_state */,
                               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::NAV_DIM,        OPTIONS>>& /* ref_nav_state */,
                               const Eigen::Ref<const Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,      DIM_S::ERROR_DIM,      OPTIONS>>& /* error_covariance */,
                                     Eigen::Ref<      Eigen::Matrix<SCALAR,DIM_S::LINCOV::AUG_DIM,DIM_S::LINCOV::AUG_DIM,OPTIONS>>  /* aug_covariance */)
{
  return Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS>::Constant(std::numeric_limits<SCALAR>::quiet_NaN());
}
} // namespace sensors
} // namespace kf

#endif
/* no_sensors_controller.hpp */
