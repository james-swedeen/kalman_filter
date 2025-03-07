/**
 * @File: measurement_base.hpp
 * @Date: April 2022
 * @Author: James Swedeen
 *
 * @brief
 * A base class for non-inertial measurements.
 **/

#ifndef KALMAN_FILTER_SENSORS_MEASUREMENTS_MEASUREMENT_BASE_HPP
#define KALMAN_FILTER_SENSORS_MEASUREMENTS_MEASUREMENT_BASE_HPP

/* C++ Headers */
#include<cstdint>
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */

namespace kf
{
namespace sensors
{
template<Eigen::Index DIM, typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
class MeasurementBase;

template<Eigen::Index DIM, typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using MeasurementBasePtr = std::shared_ptr<MeasurementBase<DIM,DIM_S,SCALAR,OPTIONS>>;

/**
 * @DIM
 * The size of the actual measurement values.
 *
 * @DIM_S
 * The type of a Dimensions object or an inheriting object that has information about the size of the state vectors.
 *
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options.
 **/
template<Eigen::Index DIM, typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class MeasurementBase
{
public:
  /**
   * @Default Constructor
   **/
  MeasurementBase() = delete;
  /**
   * @Copy Constructor
   **/
  MeasurementBase(const MeasurementBase&) noexcept = default;
  /**
   * @Move Constructor
   **/
  MeasurementBase(MeasurementBase&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * Initializes the object for use.
   *
   * @parameters
   * ss_kalman_gain: The steady state kalman gain, note this is only used if the USE_SS_KALMAN_GAIN flag is
   *                 enabled in the measurement controller
   **/
  template<typename KALMAN_GAIN_DERIVED>
  explicit MeasurementBase(const Eigen::MatrixBase<KALMAN_GAIN_DERIVED>& ss_kalman_gain);
  /**
   * @Deconstructor
   **/
  virtual ~MeasurementBase() noexcept = default;
  /**
   * @Assignment Operators
   **/
  MeasurementBase& operator=(const MeasurementBase&)  noexcept = default;
  MeasurementBase& operator=(      MeasurementBase&&) noexcept = default;
  /**
   * @getMeasurement
   *
   * @brief
   * Used to synthesize a measurement.
   *
   * @parameters
   * time: The current simulation time
   * truth_state: The current truth state vector
   * measurement_noise: The vector of additive measurement noise to be added to the measurement
   *
   * @return
   * The vector of measurements.
   **/
  inline virtual Eigen::Matrix<SCALAR,1,DIM,OPTIONS>
    getMeasurement(const SCALAR                                                              time,
                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& truth_state,
                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,             OPTIONS>>& measurement_noise) = 0;
  /**
   * @estimateMeasurement
   *
   * @brief
   * Used to estimate a measurement given the navigation state.
   *
   * @parameters
   * time: The current simulation time
   * nav_state: The current navigation state vector
   *
   * @return
   * The vector of measurements.
   **/
  inline virtual Eigen::Matrix<SCALAR,1,DIM,OPTIONS>
    estimateMeasurement(const SCALAR                                                            time,
                        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& nav_state) = 0;
  /**
   * @getMeasurementEstimatePDWRErrorState
   *
   * @brief
   * Finds the derivative of "estimateMeasurement" with respect to the error state.
   *
   * @parameters
   * time: The current simulation time
   * nav_state: The current navigation state vector
   *
   * @return
   * The measurement matrix.
   **/
  inline virtual Eigen::Matrix<SCALAR,DIM,DIM_S::ERROR_DIM,OPTIONS>
    getMeasurementEstimatePDWRErrorState(
      const SCALAR                                                            time,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& nav_state) = 0;
  /**
   * @getMeasurementPDWRDispersionState
   *
   * @brief
   * Finds the derivative of "getMeasurement" with respect to the truth dispersion state.
   *
   * @parameters
   * time: The current simulation time
   * truth_state: The current truth state vector
   *
   * @return
   * The measurement matrix.
   **/
  inline virtual Eigen::Matrix<SCALAR,DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS>
    getMeasurementPDWRDispersionState(
      const SCALAR                                                              time,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& truth_state) = 0;
  /**
   * @measurementReady
   *
   * @brief
   * Returns true if and only if it is time to use a measurement.
   *
   * @parameters
   * time: The current simulation time
   * next_measurement_time: The time that the next measurement will be ready to use
   * truth_state: The current truth state vector
   *
   * @return
   * True if and only if it is time to use a measurement.
   **/
  inline virtual bool measurementReady(const SCALAR                                                              time,
                                       const SCALAR                                                              next_measurement_time,
                                       const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& truth_state) = 0;
  /**
   * @updateNextMeasurementTime
   *
   * @brief
   * Finds the next time that a measurement could be ready.
   *
   * @parameters
   * time: The current simulation time
   * next_measurement_time: The old time that the next measurement will be ready to use
   *
   * @return
   * The new time that the next measurement will be ready to use
   **/
  inline virtual SCALAR updateNextMeasurementTime(const SCALAR time, const SCALAR next_measurement_time) = 0;
  /**
   * @getSteadyStateKalmanGain
   *
   * @brief
   * Used to retrieve the internally held steady state kalman gain.
   *
   * @return
   * The steady state kalman gain.
   **/
  inline Eigen::Ref<const Eigen::Matrix<SCALAR,DIM,DIM_S::ERROR_DIM,OPTIONS>> getSteadyStateKalmanGain() const noexcept;
  /**
   * @setSteadyStateKalmanGain
   *
   * @brief
   * Used to set the internally held steady state kalman gain.
   *
   * @parameters
   * ss_kalman_gain: The new steady state kalman gain
   *
   * @return
   * The updated steady state kalman gain.
   **/
  inline Eigen::Ref<const Eigen::Matrix<SCALAR,DIM,DIM_S::ERROR_DIM,OPTIONS>>
    setSteadyStateKalmanGain(const Eigen::Ref<const Eigen::Matrix<SCALAR,DIM,DIM_S::ERROR_DIM,OPTIONS>>& ss_kalman_gain) noexcept;
private:
  Eigen::Matrix<SCALAR,DIM,DIM_S::ERROR_DIM,OPTIONS> ss_kalman_gain;
};

template<Eigen::Index DIM, typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename KALMAN_GAIN_DERIVED>
  MeasurementBase<DIM,DIM_S,SCALAR,OPTIONS>::MeasurementBase(const Eigen::MatrixBase<KALMAN_GAIN_DERIVED>& ss_kalman_gain)
 : ss_kalman_gain(ss_kalman_gain)
{
  static_assert((int(KALMAN_GAIN_DERIVED::RowsAtCompileTime) == DIM)              or (int(KALMAN_GAIN_DERIVED::RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(KALMAN_GAIN_DERIVED::ColsAtCompileTime) == DIM_S::ERROR_DIM) or (int(KALMAN_GAIN_DERIVED::ColsAtCompileTime) == Eigen::Dynamic));
  assert(ss_kalman_gain.rows() == DIM);
  assert(ss_kalman_gain.cols() == DIM_S::ERROR_DIM);
}

template<Eigen::Index DIM, typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Ref<const Eigen::Matrix<SCALAR,DIM,DIM_S::ERROR_DIM,OPTIONS>>
  MeasurementBase<DIM,DIM_S,SCALAR,OPTIONS>::getSteadyStateKalmanGain() const noexcept
{
  return this->ss_kalman_gain;
}

template<Eigen::Index DIM, typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Ref<const Eigen::Matrix<SCALAR,DIM,DIM_S::ERROR_DIM,OPTIONS>> MeasurementBase<DIM,DIM_S,SCALAR,OPTIONS>::
  setSteadyStateKalmanGain(const Eigen::Ref<const Eigen::Matrix<SCALAR,DIM,DIM_S::ERROR_DIM,OPTIONS>>& ss_kalman_gain) noexcept
{
  return (this->ss_kalman_gain = ss_kalman_gain);
}
} // namespace sensors
} // namespace kf

#endif
/* measurement_base.hpp */
