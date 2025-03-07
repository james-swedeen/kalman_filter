/**
 * @File: simple_mapping.hpp
 * @Date: May 2022
 * @Author: James Swedeen
 *
 * @brief
 * A class that defines the mappings between states when the mapping is trivial, with
 * every state in the truth and navigation models being the same.
 **/

#ifndef KALMAN_FILTER_TRUTH_NAV_MAPPINGS_SIMPLE_MAPPING_HPP
#define KALMAN_FILTER_TRUTH_NAV_MAPPINGS_SIMPLE_MAPPING_HPP

/* C++ Headers */
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<kalman_filter/mappings/mappings_base.hpp>
#include<kalman_filter/math/quaternion.hpp>

namespace kf
{
namespace map
{
template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
class SimpleMapping;

template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using SimpleMappingPtr = std::shared_ptr<SimpleMapping<DIM_S,SCALAR,OPTIONS>>;

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
class SimpleMapping
 : public MappingsBase<DIM_S,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  SimpleMapping() noexcept = default;
  /**
   * @Copy Constructor
   **/
  SimpleMapping(const SimpleMapping&) noexcept = default;
  /**
   * @Move Constructor
   **/
  SimpleMapping(SimpleMapping&&) noexcept = default;
  /**
   * @Deconstructor
   **/
  ~SimpleMapping() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  SimpleMapping& operator=(const SimpleMapping&)  noexcept = default;
  SimpleMapping& operator=(      SimpleMapping&&) noexcept = default;
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
   * @mapRefNAV
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
  inline Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS>
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
inline Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS> SimpleMapping<DIM_S,SCALAR,OPTIONS>::
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
    math::quat::normalize(math::quat::product(error_quat, nav_state.template middleCols<4>(DIM_S::NAV::QUAT_START_IND)));

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS> SimpleMapping<DIM_S,SCALAR,OPTIONS>::
  injectErrors(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,     OPTIONS>>& truth_state,
               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DISP_DIM,OPTIONS>>& truth_disp_state)
{
  Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS> output;

  // Deal with all of the non quaternion stuff
  output.template leftCols<DIM_S::TRUTH::QUAT_START_IND>() =
    truth_state.     template leftCols<DIM_S::TRUTH::QUAT_START_IND>() +
    truth_disp_state.template leftCols<DIM_S::TRUTH_DISP::EULER_START_IND>();
  output.template rightCols<DIM_S::TRUTH_DIM - DIM_S::TRUTH::QUAT_END_IND - 1>() =
    truth_state.     template rightCols<DIM_S::TRUTH_DIM      - DIM_S::TRUTH::QUAT_END_IND - 1>() +
    truth_disp_state.template rightCols<DIM_S::TRUTH_DISP_DIM - DIM_S::TRUTH_DISP::EULER_END_IND - 1>();
  // Deal with the quaternion
  Eigen::Matrix<SCALAR,1,4,OPTIONS> error_quat;
  error_quat[0] = 1;
  error_quat.template rightCols<3>() = truth_disp_state.template middleCols<3>(DIM_S::TRUTH_DISP::EULER_START_IND).array() / SCALAR(2);

  output.template middleCols<4>(DIM_S::TRUTH::QUAT_START_IND) =
    math::quat::normalize(math::quat::product(error_quat, truth_state.template middleCols<4>(DIM_S::TRUTH::QUAT_START_IND)));

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS> SimpleMapping<DIM_S,SCALAR,OPTIONS>::
  calculateAverageTruthState(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::TRUTH_DIM,OPTIONS>& truth_state_vectors)
{
  Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS> output;
  const SCALAR num_vectors = truth_state_vectors.rows();

  // Deal with all of the non quaternion stuff
  output.template leftCols<DIM_S::TRUTH::QUAT_START_IND>() =
    truth_state_vectors.template leftCols<DIM_S::TRUTH::QUAT_START_IND>().colwise().sum().array() / num_vectors;

  output.template rightCols<DIM_S::TRUTH_DIM - DIM_S::TRUTH::QUAT_END_IND - 1>() =
    truth_state_vectors.template rightCols<DIM_S::TRUTH_DIM - DIM_S::TRUTH::QUAT_END_IND - 1>().colwise().sum().array() / num_vectors;
  // Deal with the quaternion
  output.template middleCols<4>(DIM_S::TRUTH::QUAT_START_IND) =
    math::quat::averageQuaternions(truth_state_vectors.template middleCols<4>(DIM_S::TRUTH::QUAT_START_IND));

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS> SimpleMapping<DIM_S,SCALAR,OPTIONS>::
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
inline Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS> SimpleMapping<DIM_S,SCALAR,OPTIONS>::
  calculateErrorState(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& truth_state,
                      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,  OPTIONS>>& nav_state)
{
  Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS> output;

  // Deal with all of the non quaternion stuff
  output.template leftCols<DIM_S::ERROR::EULER_START_IND>() =
    truth_state.template leftCols<DIM_S::TRUTH::QUAT_START_IND>() -
    nav_state.  template leftCols<DIM_S::NAV::QUAT_START_IND>();
  output.template rightCols<DIM_S::ERROR_DIM - DIM_S::ERROR::EULER_END_IND - 1>() =
    truth_state.template rightCols<DIM_S::TRUTH_DIM - DIM_S::TRUTH::QUAT_END_IND - 1>() -
    nav_state.  template rightCols<DIM_S::NAV_DIM   - DIM_S::NAV::QUAT_END_IND - 1>();
  // Deal with the quaternion
  output.template middleCols<3>(DIM_S::ERROR::EULER_START_IND) =
    SCALAR(2) * math::quat::product(truth_state.template middleCols<4>(DIM_S::TRUTH::QUAT_START_IND),
                                    math::quat::conjugate(nav_state.template middleCols<4>(DIM_S::NAV::QUAT_START_IND))).template rightCols<3>().array();

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DISP_DIM,OPTIONS> SimpleMapping<DIM_S,SCALAR,OPTIONS>::
  calculateTruthStateDisp(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& truth_state,
                          const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& avg_truth_state)
{
  Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DISP_DIM,OPTIONS> output;

  // Deal with all of the non quaternion stuff
  output.template leftCols<DIM_S::TRUTH_DISP::EULER_START_IND>() =
    truth_state.    template leftCols<DIM_S::TRUTH::QUAT_START_IND>() -
    avg_truth_state.template leftCols<DIM_S::TRUTH::QUAT_START_IND>();
  output.template rightCols<DIM_S::TRUTH_DISP_DIM - DIM_S::TRUTH_DISP::EULER_END_IND - 1>() =
    truth_state.    template rightCols<DIM_S::TRUTH_DIM - DIM_S::TRUTH::QUAT_END_IND - 1>() -
    avg_truth_state.template rightCols<DIM_S::TRUTH_DIM - DIM_S::TRUTH::QUAT_END_IND - 1>();
  // Deal with the quaternion
  output.template middleCols<3>(DIM_S::TRUTH_DISP::EULER_START_IND) =
    SCALAR(2) * math::quat::product(truth_state.template middleCols<4>(DIM_S::TRUTH::QUAT_START_IND),
                                    math::quat::conjugate(avg_truth_state.template middleCols<4>(DIM_S::TRUTH::QUAT_START_IND))).template rightCols<3>().array();

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS> SimpleMapping<DIM_S,SCALAR,OPTIONS>::
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
    SCALAR(2) * math::quat::product(nav_state.template middleCols<4>(DIM_S::NAV::QUAT_START_IND),
                                    math::quat::conjugate(avg_nav_state.template middleCols<4>(DIM_S::NAV::QUAT_START_IND))).template rightCols<3>().array();

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS> SimpleMapping<DIM_S,SCALAR,OPTIONS>::
  mapRefTruth(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,OPTIONS>>& ref_state)
{
  Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS> output;

  output.setZero();
  // Deal with all of the non quaternion stuff
  output.template leftCols<DIM_S::TRUTH::QUAT_START_IND>() = ref_state.template leftCols<DIM_S::REF::EULER_START_IND>();
  output.template middleCols<DIM_S::REF_DIM - DIM_S::REF::EULER_END_IND - 1>(DIM_S::TRUTH::QUAT_END_IND + 1) =
    ref_state.template rightCols<DIM_S::REF_DIM - DIM_S::REF::EULER_END_IND - 1>();
  // Deal with the quaternion
  output.template middleCols<4>(DIM_S::TRUTH::QUAT_START_IND) =
    kf::math::quat::rollPitchYawToQuaternion(ref_state.template middleCols<3>(DIM_S::REF::EULER_START_IND));

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS> SimpleMapping<DIM_S,SCALAR,OPTIONS>::
  mapRefNav(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,OPTIONS>>& ref_state)
{
  Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS> output;

  output.setZero();
  // Deal with all of the non quaternion stuff
  output.template leftCols<DIM_S::NAV::QUAT_START_IND>() = ref_state.template leftCols<DIM_S::REF::EULER_START_IND>();
  output.template middleCols<DIM_S::REF_DIM - DIM_S::REF::EULER_END_IND - 1>(DIM_S::NAV::QUAT_END_IND + 1) =
    ref_state.template rightCols<DIM_S::REF_DIM - DIM_S::REF::EULER_END_IND - 1>();
  // Deal with the quaternion
  output.template middleCols<4>(DIM_S::NAV::QUAT_START_IND) =
    kf::math::quat::rollPitchYawToQuaternion(ref_state.template middleCols<3>(DIM_S::REF::EULER_START_IND));

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS> SimpleMapping<DIM_S,SCALAR,OPTIONS>::
  mapTruthNav(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& truth_state)
{
  return truth_state.template leftCols<DIM_S::NAV_DIM>();
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS> SimpleMapping<DIM_S,SCALAR,OPTIONS>::
  mapNavTruth(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& nav_state)
{
  Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS> output;

  output.template leftCols<DIM_S::NAV_DIM>() = nav_state;
  output.template rightCols<DIM_S::TRUTH_DIM-DIM_S::NAV_DIM>().setZero();

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS> SimpleMapping<DIM_S,SCALAR,OPTIONS>::
  getTruthNavMapPDWRDispersionState(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& /* truth_state */)
{
  return Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS>::Identity();
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::ERROR_DIM,OPTIONS> SimpleMapping<DIM_S,SCALAR,OPTIONS>::
  getNavTruthMapPDWRErrorState(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& /* nav_state */)
{
  return Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::ERROR_DIM,OPTIONS>::Identity();
}
} // namespace map
} // namespace kf

#endif
/* simple_mapping.hpp */

