/**
 * @File: dimension_struct.hpp
 * @Date: April 2022
 * @Author: James Swedeen
 *
 * @brief
 * Defines a struct that holds information about how big the navigation state vector is, how big the truth state vector
 * is, ext.
 **/

#ifndef KALMAN_FILTER_HELPERS_DIMENSION_STRUCT_HPP
#define KALMAN_FILTER_HELPERS_DIMENSION_STRUCT_HPP

/* Eigen Headers */
#include<Eigen/Dense>

namespace kf
{
/**
 * @REF_DIM_T
 * The number of dimensions in each point along the reference trajectory
 *
 * @TRUTH_DIM_T
 * The number of dimensions each point of the truth state has.
 *
 * @NAV_DIM_T
 * The number of dimensions each point of the navigation state has.
 *
 * @NUM_MEAS_DIM_T
 * The number of discreet measurements there are.
 *
 * @TRUTH_DISP_DIM_T
 * The number of dimensions each point of the truth dispersion state has.
 *
 * @ERROR_DIM_T
 * The number of dimensions each point of the error state vector has.
 *
 * @INER_MEAS_DIM_T
 * The number of inertial measurements being used for model replacement there are.
 *
 * @CONTROL_DIM_T
 * The number of control input there are.
 *
 * @TRUTH_NOISE_DIM_T
 * The number of states in the additive process noise on the time derivative of the truth state.
 *
 * @INER_MEAS_NOISE_DIM_T
 * The number of states in the additive process noise on the inertial measurements.
 *
 * @STEADY_STATE_ERROR_COV_T
 * Set to true if you wish to skip the calculation of the error state covariance and use a steady state value instead.
 **/
template<Eigen::Index REF_DIM_T,
         Eigen::Index TRUTH_DIM_T,
         Eigen::Index NAV_DIM_T,
         Eigen::Index NUM_MEAS_DIM_T,
         Eigen::Index TRUTH_DISP_DIM_T,
         Eigen::Index ERROR_DIM_T,
         Eigen::Index INER_MEAS_DIM_T,
         Eigen::Index CONTROL_DIM_T,
         Eigen::Index TRUTH_NOISE_DIM_T,
         Eigen::Index INER_MEAS_NOISE_DIM_T,
         bool         STEADY_STATE_ERROR_COV_T>
struct Dimensions
{
public:
  // If true skip the calculation of the error state covariance and use a steady state value instead
  inline static constexpr const bool USE_STEADY_STATE_ERROR_COV = STEADY_STATE_ERROR_COV_T;
  // The size of the reference/nominal state vector
  inline static constexpr const Eigen::Index REF_DIM = REF_DIM_T;
  // The size of the true state vector
  inline static constexpr const Eigen::Index TRUTH_DIM = TRUTH_DIM_T;
  // The size of the navigation state vector
  inline static constexpr const Eigen::Index NAV_DIM = NAV_DIM_T;
  // The number of discreet measurements there are
  inline static constexpr const Eigen::Index NUM_MEAS_DIM = NUM_MEAS_DIM_T;
  // The size of the truth dispersion state vector
  inline static constexpr const Eigen::Index TRUTH_DISP_DIM = TRUTH_DISP_DIM_T;
  // The size of the error state vector
  inline static constexpr const Eigen::Index ERROR_DIM = ERROR_DIM_T;
  // The size of the inertial measurements vector
  inline static constexpr const Eigen::Index INER_MEAS_DIM = INER_MEAS_DIM_T;
  // The number of control input there are
  inline static constexpr const Eigen::Index CONTROL_DIM = CONTROL_DIM_T;
  // The number of states in the additive process noise on the time derivative of the truth state
  inline static constexpr const Eigen::Index TRUTH_NOISE_DIM = TRUTH_NOISE_DIM_T;
  // The number of states in the inertial measurements
  inline static constexpr const Eigen::Index INER_MEAS_NOISE_DIM = INER_MEAS_NOISE_DIM_T;

  // Number of elements in the error state covariance matrix
  inline static constexpr const Eigen::Index ERROR_COV_LEN = (STEADY_STATE_ERROR_COV_T) ? 0 : ERROR_DIM * ERROR_DIM;

  // Location of the time
  inline static constexpr const Eigen::Index TIME_IND = 0;
  // Location of the first element in the discreet measurement timer vector
  inline static constexpr const Eigen::Index NUM_MEAS_START_IND = 1;
  // Location of the last element in the discreet measurement timer vector
  inline static constexpr const Eigen::Index NUM_MEAS_END_IND = NUM_MEAS_START_IND + NUM_MEAS_DIM - 1;
  // Location of the first element in the desired state
  inline static constexpr const Eigen::Index REF_START_IND = NUM_MEAS_END_IND + 1;
  // Location of the last element in the desired state
  inline static constexpr const Eigen::Index REF_END_IND = REF_START_IND + REF_DIM - 1;
  // Location of the first element in the error state covariance as a vector
  inline static constexpr const Eigen::Index ERROR_COV_START_IND = REF_END_IND + ((STEADY_STATE_ERROR_COV_T) ? 0 : 1);
  // Location of the last element in the error state covariance as a vector
  inline static constexpr const Eigen::Index ERROR_COV_END_IND = ERROR_COV_START_IND + ERROR_COV_LEN - ((STEADY_STATE_ERROR_COV_T) ? 0 : 1);

  // Indexes for the Monte Carlo vector
  struct MC
  {
  public:
    // Location of the first element in the truth state
    inline static constexpr const Eigen::Index TRUTH_START_IND = ERROR_COV_END_IND + 1;
    // Location of the last element in the truth state
    inline static constexpr const Eigen::Index TRUTH_END_IND = TRUTH_START_IND + TRUTH_DIM - 1;
    // Location of the first element in the navigation state
    inline static constexpr const Eigen::Index NAV_START_IND = TRUTH_END_IND + 1;
    // Location of the last element in the navigation state
    inline static constexpr const Eigen::Index NAV_END_IND = NAV_START_IND + NAV_DIM - 1;

    // The length of the full state vector
    inline static constexpr const Eigen::Index FULL_STATE_LEN = NAV_END_IND + 1;
  };
  // Indexes for the Linear Covariance vector
  struct LINCOV
  {
  public:
    // The number of states in the augmented state vector
    inline static constexpr const Eigen::Index AUG_DIM = TRUTH_DISP_DIM + ERROR_DIM;
    // Number of elements in the augmented state covariance matrix
    inline static constexpr const Eigen::Index AUG_COV_LEN = AUG_DIM * AUG_DIM;

    // Location of the first element in the augmented state covariance
    inline static constexpr const Eigen::Index AUG_COV_START_IND = ERROR_COV_END_IND + 1;
    // Location of the last element in the augmented state covariance
    inline static constexpr const Eigen::Index AUG_COV_END_IND = AUG_COV_START_IND + AUG_COV_LEN - 1;

    // The length of the full state vector
    inline static constexpr const Eigen::Index FULL_STATE_LEN = AUG_COV_END_IND + 1;

    // Location in the augmented vector were the first element of the truth state dispersion is
    inline static constexpr const Eigen::Index TRUTH_DISP_START_IND = 0;
    // Location in the augmented vector were the last element of the truth state dispersion is
    inline static constexpr const Eigen::Index TRUTH_DISP_END_IND = TRUTH_DISP_START_IND + TRUTH_DISP_DIM - 1;
    // Location in the augmented vector were the first element of the navigation state dispersion is
    inline static constexpr const Eigen::Index NAV_DISP_START_IND = TRUTH_DISP_END_IND + 1;
    // Location in the augmented vector were the last element of the navigation state dispersion is
    inline static constexpr const Eigen::Index NAV_DISP_END_IND = NAV_DISP_START_IND + ERROR_DIM - 1;
  };
};
} // namespace kf

#endif
/* dimension_struct.hpp */
