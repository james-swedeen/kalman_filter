/**
 * @File: dubins_airplane_mapping.hpp
 * @Date: July 2023
 * @Author: James Swedeen
 *
 * @brief
 * A helper class that defines the mappings between the state vectors of the Dubin's Airplane model.
 **/

#ifndef KALMAN_FILTER_MAPPINGS_DUBINS_AIRPLANE_MAPPING_HPP
#define KALMAN_FILTER_MAPPINGS_DUBINS_AIRPLANE_MAPPING_HPP

/* C++ Headers */
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<kalman_filter/mappings/mappings_base.hpp>
#include<kalman_filter/math/helpers.hpp>
#include<kalman_filter/math/quaternion.hpp>
#include<kalman_filter/dynamics/dubins_airplane_model.hpp>

namespace kf
{
namespace map
{
template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
class DubinsAirplaneMapping;

template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using DubinsAirplaneMappingPtr = std::shared_ptr<DubinsAirplaneMapping<DIM_S,SCALAR,OPTIONS>>;

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
class DubinsAirplaneMapping
 : public MappingsBase<DIM_S,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  DubinsAirplaneMapping() noexcept = default;
  /**
   * @Copy Constructor
   **/
  DubinsAirplaneMapping(const DubinsAirplaneMapping&) noexcept = default;
  /**
   * @Move Constructor
   **/
  DubinsAirplaneMapping(DubinsAirplaneMapping&&) noexcept = default;
  /**
   * @Deconstructor
   **/
  ~DubinsAirplaneMapping() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  DubinsAirplaneMapping& operator=(const DubinsAirplaneMapping&)  noexcept = default;
  DubinsAirplaneMapping& operator=(      DubinsAirplaneMapping&&) noexcept = default;
  /**
   * @correctErrors
   *
   * @brief
   * Given the current estimate of the navigation state and an estimate of the error state, update the
   * navigation state.
   *
   * @parameters
   * nav_state: The navigation state vector before the measurement is applied
   * error_state: The error state vector
   *
   * @return
   * The updated navigation state vector.
   **/
  inline Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>
    correctErrors(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,  OPTIONS>>& nav_state,
                  const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS>>& error_state) override;
  /**
   * @injectErrors
   *
   * @brief
   * Adds the given error state into the given truth state to inject errors into the truth state.
   *
   * @parameters
   * truth_state: The truth state
   * truth_disp_state: The truth dispersion state vector to inject into the truth state
   *
   * @return
   * The updated truth state vector.
   **/
  inline Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>
    injectErrors(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,     OPTIONS>>& truth_state,
                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DISP_DIM,OPTIONS>>& truth_disp_state) override;
  /**
   * @calculateAverageTruthState
   *
   * @brief
   * Finds the average of a set of truth state vectors.
   *
   * @parameters
   * truth_state_vectors: The truth state vectors
   *
   * @return
   * The average truth state.
   **/
  inline Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>
    calculateAverageTruthState(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::TRUTH_DIM,OPTIONS>& truth_state_vectors) override;
  /**
   * @calculateAverageNavState
   *
   * @brief
   * Finds the average of a set of navigation state vectors.
   *
   * @parameters
   * nav_state_vectors: The navigation state vectors
   *
   * @return
   * The average navigation state.
   **/
  inline Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>
    calculateAverageNavState(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::NAV_DIM,OPTIONS>& nav_state_vectors) override;
  /**
   * @calculateErrorState
   *
   * @brief
   * Finds the difference between the truth state vector and the navigation state as x_t - x_n.
   *
   * @parameters
   * truth_state: The truth state vector
   * nav_state: The navigation state
   *
   * @return
   * The updated navigation state vector.
   **/
  inline Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS>
    calculateErrorState(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& truth_state,
                        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,  OPTIONS>>& nav_state) override;
  /**
   * @calculateTruthStateDisp
   *
   * @brief
   * Finds the difference between the truth state vector and the averaged truth state as x_t - bar{x}_t.
   *
   * @parameters
   * truth_state: The truth state vector
   * avg_truth_state: The averaged truth state
   *
   * @return
   * The truth state dispersion.
   **/
  inline Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DISP_DIM,OPTIONS>
    calculateTruthStateDisp(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& truth_state,
                            const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& avg_truth_state) override;
  /**
   * @calculateNavStateDisp
   *
   * @brief
   * Finds the difference between the navigation state vector and the averaged navigation state as x_n - bar{x}_n.
   *
   * @parameters
   * nav_state: The navigation state vector
   * avg_nav_state: The averaged navigation state
   *
   * @return
   * The navigation state dispersion.
   **/
  inline Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS>
    calculateNavStateDisp(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& nav_state,
                          const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& avg_nav_state) override;
  /**
   * @mapRefTruth
   *
   * @brief
   * Maps the given vector from the reference trajectory to a truth state vector.
   *
   * @parameters
   * ref_state: A state from the reference trajectory
   *
   * @return
   * The truth state mapping of the given vector from the reference trajectory.
   **/
  inline Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>
    mapRefTruth(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,OPTIONS>>& ref_state) override;
  /**
   * @mapRefNav
   *
   * @brief
   * Maps the given vector from the reference trajectory to a navigation state vector.
   *
   * @parameters
   * ref_state: A state from the reference trajectory
   *
   * @return
   * The navigation state mapping of the given vector from the reference trajectory.
   **/
  inline Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>
    mapRefNav(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,OPTIONS>>& ref_state) override;
  /**
   * @mapNavTruth
   *
   * @brief
   * Maps the given vector from the navigation state to the truth state.
   *
   * @parameters
   * truth_state: The current truth state vector
   *
   * @return
   * The truth state mapping of the given vector.
   **/
  inline Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>
    mapTruthNav(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& truth_state) override;
  /**
   * @mapTruthNav
   *
   * @brief
   * Maps the given vector from the truth state to the navigation state.
   *
   * @parameters
   * nav_state: The navigation state vector
   *
   * @return
   * The navigation state mapping of the given vector.
   **/
  inline Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>
    mapNavTruth(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& nav_state) override;
  /**
   * @getTruthNavMapPDWRDispersionState
   *
   * @brief
   * Finds the linearization of the mapping from the truth state to the
   * navigation state with respect to the error state.
   *
   * @parameters
   * truth_state: The current truth state vector
   *
   * @return
   * The linearization of the mapping from the truth state to the navigation state with respect to the truth state.
   **/
  inline Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS>
    getTruthNavMapPDWRDispersionState(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& truth_state) override;
  /**
   * @getNavTruthMapPDWRErrorState
   *
   * @brief
   * Finds the linearization of the mapping from the truth state to the
   * navigation state with respect to the error state.
   *
   * @parameters
   * nav_state: The navigation state vector
   *
   * @return
   * The linearization of the mapping from the truth state to the navigation state with respect to the truth state.
   **/
  inline Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::ERROR_DIM,OPTIONS>
    getNavTruthMapPDWRErrorState(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& nav_state) override;
};


template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS> DubinsAirplaneMapping<DIM_S,SCALAR,OPTIONS>::
  correctErrors(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,  OPTIONS>>& nav_state,
                const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS>>& error_state)
{
  Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS> output;

  // Deal with all of the non quaternion stuff
  output.template leftCols<DIM_S::NAV::QUAT_START_IND>() =
    nav_state.  template leftCols<DIM_S::NAV::QUAT_START_IND>() +
    error_state.template leftCols<DIM_S::ERROR::EULER_START_IND>();
  output.template rightCols<DIM_S::NAV_DIM - DIM_S::NAV::QUAT_END_IND - 1>() =
    nav_state.  template rightCols<DIM_S::NAV_DIM   - DIM_S::NAV::QUAT_END_IND - 1>() +
    error_state.template rightCols<DIM_S::ERROR_DIM - DIM_S::ERROR::EULER_END_IND - 1>();
  // Deal with the quaternion
  Eigen::Matrix<SCALAR,1,4,OPTIONS> error_quat;
  error_quat[0] = 1;
  error_quat.template rightCols<3>() = error_state.template middleCols<3>(DIM_S::ERROR::EULER_START_IND).array() / SCALAR(2);

  output.template middleCols<4>(DIM_S::NAV::QUAT_START_IND) =
    math::quat::normalize(math::quat::product(nav_state.template middleCols<4>(DIM_S::NAV::QUAT_START_IND), error_quat));

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS> DubinsAirplaneMapping<DIM_S,SCALAR,OPTIONS>::
  injectErrors(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,     OPTIONS>>& truth_state,
               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DISP_DIM,OPTIONS>>& truth_disp_state)
{
  return truth_state + truth_disp_state;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS> DubinsAirplaneMapping<DIM_S,SCALAR,OPTIONS>::
  calculateAverageTruthState(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::TRUTH_DIM,OPTIONS>& truth_state_vectors)
{
  Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS> output;
  const SCALAR num_vectors = truth_state_vectors.rows();

  // Position
  output.template leftCols<3>() = truth_state_vectors.template leftCols<3>().colwise().sum().array() / num_vectors;
  // Angle states
  output[DIM_S::TRUTH::ROLL_IND]  = std::atan2(truth_state_vectors.template middleCols<1>(DIM_S::TRUTH::ROLL_IND). array().sin().sum() / num_vectors,
                                               truth_state_vectors.template middleCols<1>(DIM_S::TRUTH::ROLL_IND). array().cos().sum() / num_vectors);
  output[DIM_S::TRUTH::PITCH_IND] = std::atan2(truth_state_vectors.template middleCols<1>(DIM_S::TRUTH::PITCH_IND).array().sin().sum() / num_vectors,
                                               truth_state_vectors.template middleCols<1>(DIM_S::TRUTH::PITCH_IND).array().cos().sum() / num_vectors);
  output[DIM_S::TRUTH::YAW_IND]   = std::atan2(truth_state_vectors.template middleCols<1>(DIM_S::TRUTH::YAW_IND).  array().sin().sum() / num_vectors,
                                               truth_state_vectors.template middleCols<1>(DIM_S::TRUTH::YAW_IND).  array().cos().sum() / num_vectors);
  // Air speed
  output[DIM_S::TRUTH::AIR_SPEED_IND] = (truth_state_vectors.col(DIM_S::TRUTH::AIR_SPEED_IND).colwise().sum().array() / num_vectors)[0];
  if constexpr(DIM_S::USE_IMU_BIASES)
  {
    // Bias
    output.template rightCols<6>() = truth_state_vectors.template rightCols<6>().colwise().sum().array() / num_vectors;
  }

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS> DubinsAirplaneMapping<DIM_S,SCALAR,OPTIONS>::
  calculateAverageNavState(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::NAV_DIM,OPTIONS>& nav_state_vectors)
{
  Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS> output;
  const SCALAR num_vectors = nav_state_vectors.rows();

  // Deal with all of the non quaternion stuff
  output.template leftCols<DIM_S::NAV::QUAT_START_IND>() =
    nav_state_vectors.template leftCols<DIM_S::NAV::QUAT_START_IND>().colwise().sum().array() / num_vectors;

  output.template rightCols<DIM_S::NAV_DIM - DIM_S::NAV::QUAT_END_IND - 1>() =
    nav_state_vectors.template rightCols<DIM_S::NAV_DIM - DIM_S::NAV::QUAT_END_IND - 1>().colwise().sum().array() / num_vectors;
  // Deal with the quaternion
  output.template middleCols<4>(DIM_S::NAV::QUAT_START_IND) =
    math::quat::averageQuaternions(nav_state_vectors.template middleCols<4>(DIM_S::NAV::QUAT_START_IND));

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS> DubinsAirplaneMapping<DIM_S,SCALAR,OPTIONS>::
  calculateErrorState(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& truth_state,
                      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,  OPTIONS>>& nav_state)
{
        Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS> output;
  const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,  OPTIONS> truth_nav = this->mapTruthNav(truth_state);

  // Position
  output.template leftCols<3>() = truth_state.template leftCols<3>() - nav_state.template leftCols<3>();
  // Euler
  output.template middleCols<3>(DIM_S::ERROR::EULER_START_IND) =
    SCALAR(2) * math::quat::product(math::quat::conjugate(nav_state.template middleCols<4>(DIM_S::NAV::QUAT_START_IND)),
                                    truth_nav.template middleCols<4>(DIM_S::NAV::QUAT_START_IND)).template rightCols<3>().array();
  // Velocity
  output.template middleCols<3>(DIM_S::ERROR::VEL_START_IND) = truth_nav.template middleCols<3>(DIM_S::NAV::VEL_START_IND) -
                                                               nav_state.template middleCols<3>(DIM_S::NAV::VEL_START_IND);
  if constexpr(DIM_S::USE_IMU_BIASES)
  {
    // Bias
    output.template middleCols<3>(DIM_S::ERROR::GYRO_BIAS_START_IND) = truth_nav.template middleCols<3>(DIM_S::NAV::GYRO_BIAS_START_IND) -
                                                                       nav_state.template middleCols<3>(DIM_S::NAV::GYRO_BIAS_START_IND);
    output.template middleCols<3>(DIM_S::ERROR::ACCEL_BIAS_START_IND) = truth_nav.template middleCols<3>(DIM_S::NAV::ACCEL_BIAS_START_IND) -
                                                                        nav_state.template middleCols<3>(DIM_S::NAV::ACCEL_BIAS_START_IND);
  }

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DISP_DIM,OPTIONS> DubinsAirplaneMapping<DIM_S,SCALAR,OPTIONS>::
  calculateTruthStateDisp(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& truth_state,
                          const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& avg_truth_state)
{
  Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DISP_DIM,OPTIONS> output;

  // Position
  output.template leftCols<3>() = truth_state.template leftCols<3>() - avg_truth_state.template leftCols<3>();
  // Euler Angles
  output.template middleCols<3>(DIM_S::TRUTH_DISP::EULER_START_IND) =
    SCALAR(2) * math::quat::product(math::quat::conjugate(kf::math::quat::rollPitchYawToQuaternion(avg_truth_state.template middleCols<3>(DIM_S::TRUTH::EULER_START_IND))),
                                    kf::math::quat::rollPitchYawToQuaternion(truth_state.template middleCols<3>(DIM_S::TRUTH::EULER_START_IND))).template rightCols<3>().array();
  // Air Speed
  output[DIM_S::TRUTH_DISP::AIR_SPEED_IND] = truth_state[DIM_S::TRUTH::AIR_SPEED_IND] - avg_truth_state[DIM_S::TRUTH::AIR_SPEED_IND];
  if constexpr(DIM_S::USE_IMU_BIASES)
  {
    // Bias
    output.template middleCols<3>(DIM_S::TRUTH_DISP::GYRO_BIAS_START_IND) = truth_state.    template middleCols<3>(DIM_S::TRUTH::GYRO_BIAS_START_IND) -
                                                                            avg_truth_state.template middleCols<3>(DIM_S::TRUTH::GYRO_BIAS_START_IND);
    output.template middleCols<3>(DIM_S::TRUTH_DISP::ACCEL_BIAS_START_IND) = truth_state.    template middleCols<3>(DIM_S::TRUTH::ACCEL_BIAS_START_IND) -
                                                                             avg_truth_state.template middleCols<3>(DIM_S::TRUTH::ACCEL_BIAS_START_IND);
  }

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS> DubinsAirplaneMapping<DIM_S,SCALAR,OPTIONS>::
  calculateNavStateDisp(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& nav_state,
                        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& avg_nav_state)
{
  Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS> output;

  // Deal with all of the non quaternion stuff
  output.template leftCols<DIM_S::ERROR::EULER_START_IND>() =
    nav_state.    template leftCols<DIM_S::NAV::QUAT_START_IND>() -
    avg_nav_state.template leftCols<DIM_S::NAV::QUAT_START_IND>();
  output.template rightCols<DIM_S::ERROR_DIM - DIM_S::ERROR::EULER_END_IND - 1>() =
    nav_state.    template rightCols<DIM_S::NAV_DIM - DIM_S::NAV::QUAT_END_IND - 1>() -
    avg_nav_state.template rightCols<DIM_S::NAV_DIM - DIM_S::NAV::QUAT_END_IND - 1>();
  // Deal with the quaternion
  output.template middleCols<3>(DIM_S::ERROR::EULER_START_IND) =
    SCALAR(2) * math::quat::product(math::quat::conjugate(avg_nav_state.template middleCols<4>(DIM_S::NAV::QUAT_START_IND)),
                                    nav_state.template middleCols<4>(DIM_S::NAV::QUAT_START_IND)).template rightCols<3>().array();

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS> DubinsAirplaneMapping<DIM_S,SCALAR,OPTIONS>::
  mapRefTruth(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,OPTIONS>>& ref_state)
{
  Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS> output;

  // Pose
  output.template leftCols<6>() = ref_state.template leftCols<6>();
  // Air Speed
  output[DIM_S::TRUTH::AIR_SPEED_IND] = ref_state.template middleCols<3>(DIM_S::REF::VEL_START_IND).norm();
  if constexpr(DIM_S::USE_IMU_BIASES)
  {
    // Bias
    output.template rightCols<6>().setZero();
  }

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS> DubinsAirplaneMapping<DIM_S,SCALAR,OPTIONS>::
  mapRefNav(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,OPTIONS>>& ref_state)
{
  Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS> output;

  // Position
  output.template middleCols<3>(DIM_S::NAV::POS_START_IND) = ref_state.template middleCols<3>(DIM_S::REF::POS_START_IND);
  // Quaternion
  output.template middleCols<4>(DIM_S::NAV::QUAT_START_IND) =
    kf::math::quat::rollPitchYawToQuaternion(ref_state.template middleCols<3>(DIM_S::REF::EULER_START_IND));
  // Velocity
  output.template middleCols<3>(DIM_S::NAV::VEL_START_IND) = ref_state.template middleCols<3>(DIM_S::REF::VEL_START_IND);
  if constexpr(DIM_S::USE_IMU_BIASES)
  {
    // Bias
    output.template rightCols<6>().setZero();
  }

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS> DubinsAirplaneMapping<DIM_S,SCALAR,OPTIONS>::
  mapTruthNav(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& truth_state)
{
  Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS> output;

  // Position
  output.template leftCols<3>() = truth_state.template leftCols<3>();
  // Quaternion
  output.template middleCols<4>(DIM_S::NAV::QUAT_START_IND) =
    math::quat::rollPitchYawToQuaternion(truth_state.template middleCols<3>(DIM_S::TRUTH::EULER_START_IND));
  // Velocity
  output.template middleCols<3>(DIM_S::NAV::VEL_START_IND) =
    dynamics::DubinsAirplane<DIM_S,SCALAR,OPTIONS>::calculateTruthVelocity(truth_state);
  if constexpr(DIM_S::USE_IMU_BIASES)
  {
    // Bias
    output.template rightCols<6>() = truth_state.template rightCols<6>();
  }

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS> DubinsAirplaneMapping<DIM_S,SCALAR,OPTIONS>::
  mapNavTruth(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& nav_state)
{
  Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS> output;

  // Position
  output.template middleCols<3>(DIM_S::TRUTH::POS_START_IND) = nav_state.template middleCols<3>(DIM_S::NAV::POS_START_IND);
  // Euler Angles
  output.template middleCols<3>(DIM_S::TRUTH::EULER_START_IND) =
    kf::math::quat::quaternionToRollPitchYaw(nav_state.template middleCols<4>(DIM_S::NAV::QUAT_START_IND));
  // Air Speed
  const Eigen::Matrix<SCALAR,3,3,OPTIONS> ned_to_body_rotation = math::quat::quaternionToDirectionCosineMatrix(nav_state.template middleCols<4>(DIM_S::NAV::QUAT_START_IND)).transpose();
  const Eigen::Matrix<SCALAR,1,3,OPTIONS> vel_body             = ned_to_body_rotation * nav_state.template middleCols<3>(DIM_S::NAV::VEL_START_IND).transpose();

  output[DIM_S::TRUTH::AIR_SPEED_IND] = vel_body[0];
  //output[DIM_S::TRUTH::AIR_SPEED_IND] = nav_state.template middleCols<3>(DIM_S::NAV::VEL_START_IND).norm();
  if constexpr(DIM_S::USE_IMU_BIASES)
  {
    // Bias
    output.template rightCols<6>() = nav_state.template rightCols<6>();
  }

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS> DubinsAirplaneMapping<DIM_S,SCALAR,OPTIONS>::
  getTruthNavMapPDWRDispersionState(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& truth_state)
{
  Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS> output;
  output.setZero();

  const SCALAR c_pitch = std::cos(truth_state[DIM_S::TRUTH::PITCH_IND]);
  const SCALAR s_pitch = std::sin(truth_state[DIM_S::TRUTH::PITCH_IND]);
  const SCALAR c_yaw   = std::cos(truth_state[DIM_S::TRUTH::YAW_IND]);
  const SCALAR s_yaw   = std::sin(truth_state[DIM_S::TRUTH::YAW_IND]);

  // Position
  output.template block<3,3>(DIM_S::ERROR::POS_START_IND, DIM_S::TRUTH_DISP::POS_START_IND).setIdentity();
  // Euler Angles
  output.template block<3,3>(DIM_S::ERROR::EULER_START_IND, DIM_S::TRUTH_DISP::EULER_START_IND).setIdentity();
  // Velocity
  output(DIM_S::ERROR::NORTH_VEL_IND, DIM_S::TRUTH_DISP::PITCH_IND)     = -truth_state[DIM_S::TRUTH::AIR_SPEED_IND] * c_yaw * s_pitch;
  output(DIM_S::ERROR::NORTH_VEL_IND, DIM_S::TRUTH_DISP::YAW_IND)       = -truth_state[DIM_S::TRUTH::AIR_SPEED_IND] * s_yaw * c_pitch;
  output(DIM_S::ERROR::NORTH_VEL_IND, DIM_S::TRUTH_DISP::AIR_SPEED_IND) = c_yaw * c_pitch;

  output(DIM_S::ERROR::EAST_VEL_IND, DIM_S::TRUTH_DISP::PITCH_IND)     = -truth_state[DIM_S::TRUTH::AIR_SPEED_IND] * s_yaw * s_pitch;
  output(DIM_S::ERROR::EAST_VEL_IND, DIM_S::TRUTH_DISP::YAW_IND)       =  truth_state[DIM_S::TRUTH::AIR_SPEED_IND] * c_yaw * c_pitch;
  output(DIM_S::ERROR::EAST_VEL_IND, DIM_S::TRUTH_DISP::AIR_SPEED_IND) = s_yaw * c_pitch;

  output(DIM_S::ERROR::DOWN_VEL_IND, DIM_S::TRUTH_DISP::PITCH_IND)     = -truth_state[DIM_S::TRUTH::AIR_SPEED_IND] * c_pitch;
  output(DIM_S::ERROR::DOWN_VEL_IND, DIM_S::TRUTH_DISP::AIR_SPEED_IND) = -s_pitch;
  if constexpr(DIM_S::USE_IMU_BIASES)
  {
    // Bias
    output.template block<3,3>(DIM_S::ERROR::GYRO_BIAS_START_IND,  DIM_S::TRUTH_DISP::GYRO_BIAS_START_IND). setIdentity();
    output.template block<3,3>(DIM_S::ERROR::ACCEL_BIAS_START_IND, DIM_S::TRUTH_DISP::ACCEL_BIAS_START_IND).setIdentity();
  }

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::ERROR_DIM,OPTIONS> DubinsAirplaneMapping<DIM_S,SCALAR,OPTIONS>::
  getNavTruthMapPDWRErrorState(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& nav_state)
{
  Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::ERROR_DIM,OPTIONS> output;
  output.setZero();

  // Position
  output.template block<3,3>(DIM_S::TRUTH_DISP::POS_START_IND, DIM_S::ERROR::POS_START_IND).setIdentity();
  // Euler Angles
  output.template block<3,3>(DIM_S::TRUTH_DISP::EULER_START_IND, DIM_S::ERROR::EULER_START_IND).setIdentity();
  // Air Speed
  const Eigen::Matrix<SCALAR,3,3,OPTIONS> ned_to_body_rotation = math::quat::quaternionToDirectionCosineMatrix(nav_state.template middleCols<4>(DIM_S::NAV::QUAT_START_IND)).transpose();
  const Eigen::Matrix<SCALAR,1,3,OPTIONS> rpy                  = math::quat::quaternionToRollPitchYaw(nav_state.template middleCols<4>(DIM_S::NAV::QUAT_START_IND));

  const SCALAR c_pitch = std::cos(rpy[1]);
  const SCALAR s_pitch = std::sin(rpy[1]);
  const SCALAR c_yaw   = std::cos(rpy[2]);
  const SCALAR s_yaw   = std::sin(rpy[2]);

  output.template block<1,3>(DIM_S::TRUTH_DISP::AIR_SPEED_IND, DIM_S::ERROR::VEL_START_IND) = ned_to_body_rotation.template topRows<1>();
  output(DIM_S::TRUTH_DISP::AIR_SPEED_IND, DIM_S::ERROR::PITCH_IND) = (-nav_state[DIM_S::NAV::NORTH_VEL_IND] * c_yaw * s_pitch) - (nav_state[DIM_S::NAV::EAST_VEL_IND] * s_yaw * s_pitch) - (nav_state[DIM_S::NAV::DOWN_VEL_IND] * c_pitch);
  output(DIM_S::TRUTH_DISP::AIR_SPEED_IND, DIM_S::ERROR::YAW_IND)   = (-nav_state[DIM_S::NAV::NORTH_VEL_IND] * s_yaw * c_pitch) + (nav_state[DIM_S::NAV::EAST_VEL_IND] * c_yaw * c_pitch);
//  output.template block<1,3>(DIM_S::TRUTH_DISP::AIR_SPEED_IND, DIM_S::ERROR::VEL_START_IND) =
//    nav_state.template middleCols<3>(DIM_S::NAV::VEL_START_IND).normalized();
  if constexpr(DIM_S::USE_IMU_BIASES)
  {
    // Bias
    output.template block<3,3>(DIM_S::TRUTH_DISP::GYRO_BIAS_START_IND,  DIM_S::ERROR::GYRO_BIAS_START_IND). setIdentity();
    output.template block<3,3>(DIM_S::TRUTH_DISP::ACCEL_BIAS_START_IND, DIM_S::ERROR::ACCEL_BIAS_START_IND).setIdentity();
  }

  return output;
}
} // namespace map
} // namespace kf

#endif
/* dubins_airplane_mapping.hpp */

