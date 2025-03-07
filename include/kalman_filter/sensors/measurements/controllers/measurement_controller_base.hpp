/**
 * @File: measurement_controller_base.hpp
 * @Date: April 2022
 * @Author: James Swedeen
 *
 * @brief
 * A base class for controlling when and how non-inertial measurements are applied.
 **/

#ifndef KALMAN_FILTER_SENSORS_MEASUREMENTS_CONTROLLERS_MEASUREMENT_CONTROLLER_BASE_HPP
#define KALMAN_FILTER_SENSORS_MEASUREMENTS_CONTROLLERS_MEASUREMENT_CONTROLLER_BASE_HPP

/* C++ Headers */
#include<cstdint>
#include<utility>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<kalman_filter/mappings/mappings_base.hpp>
#include<kalman_filter/sensors/measurements/measurement_base.hpp>

namespace kf
{
namespace sensors
{
template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
class MeasurementControllerBase;

template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using MeasurementControllerBasePtr = std::shared_ptr<MeasurementControllerBase<DIM_S,SCALAR,OPTIONS>>;

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
class MeasurementControllerBase
{
public:
  /**
   * @Default Constructor
   **/
  MeasurementControllerBase() noexcept = default;
  /**
   * @Copy Constructor
   **/
  MeasurementControllerBase(const MeasurementControllerBase&) noexcept = default;
  /**
   * @Move Constructor
   **/
  MeasurementControllerBase(MeasurementControllerBase&&) noexcept = default;
  /**
   * @Deconstructor
   **/
  virtual ~MeasurementControllerBase() noexcept = default;
  /**
   * @Assignment Operators
   **/
  MeasurementControllerBase& operator=(const MeasurementControllerBase&)  noexcept = default;
  MeasurementControllerBase& operator=(      MeasurementControllerBase&&) noexcept = default;
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
  inline virtual Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS>
    applyMeasurements(const map::MappingsBasePtr<DIM_S,SCALAR,OPTIONS>&                                           mappings,
                      const SCALAR                                                                                time,
                      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,               DIM_S::NUM_MEAS_DIM,OPTIONS>>& meas_update_buff,
                      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,               DIM_S::TRUTH_DIM,   OPTIONS>>& truth_state,
                            Eigen::Ref<      Eigen::Matrix<SCALAR,1,               DIM_S::NAV_DIM,     OPTIONS>>  nav_state,
                            Eigen::Ref<      Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,   OPTIONS>>  error_covariance) = 0;
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
  inline virtual Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS>
    applyMeasurementsSSErrorCov(const map::MappingsBasePtr<DIM_S,SCALAR,OPTIONS>&                                           mappings,
                                const SCALAR                                                                                time,
                                const Eigen::Ref<const Eigen::Matrix<SCALAR,1,               DIM_S::NUM_MEAS_DIM,OPTIONS>>& meas_update_buff,
                                const Eigen::Ref<const Eigen::Matrix<SCALAR,1,               DIM_S::TRUTH_DIM,   OPTIONS>>& truth_state,
                                const Eigen::Ref<const Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,   OPTIONS>>& ss_error_covariance,
                                      Eigen::Ref<      Eigen::Matrix<SCALAR,1,               DIM_S::NAV_DIM,     OPTIONS>>  nav_state) = 0;
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
  inline virtual Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS>
    applyMeasurementsLinCov(const SCALAR                                                                                         time,
                            const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::NUM_MEAS_DIM,   OPTIONS>>& meas_update_buff,
                            const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::TRUTH_DIM,      OPTIONS>>& ref_truth_state,
                            const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::NAV_DIM,        OPTIONS>>& ref_nav_state,
                                  Eigen::Ref<      Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,      DIM_S::ERROR_DIM,      OPTIONS>>  error_covariance,
                                  Eigen::Ref<      Eigen::Matrix<SCALAR,DIM_S::LINCOV::AUG_DIM,DIM_S::LINCOV::AUG_DIM,OPTIONS>>  aug_covariance) = 0;
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
  inline virtual Eigen::Matrix<SCALAR,1,DIM_S::NUM_MEAS_DIM,OPTIONS>
    applyMeasurementsErrorBudget(const SCALAR                                                                                         time,
                                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::NUM_MEAS_DIM,   OPTIONS>>& meas_update_buff,
                                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::TRUTH_DIM,      OPTIONS>>& ref_truth_state,
                                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,                     DIM_S::NAV_DIM,        OPTIONS>>& ref_nav_state,
                                 const Eigen::Ref<const Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,      DIM_S::ERROR_DIM,      OPTIONS>>& error_covariance,
                                       Eigen::Ref<      Eigen::Matrix<SCALAR,DIM_S::LINCOV::AUG_DIM,DIM_S::LINCOV::AUG_DIM,OPTIONS>>  aug_covariance) = 0;
};
} // namespace sensors
} // namespace kf

#endif
/* measurement_controller_base.hpp */
