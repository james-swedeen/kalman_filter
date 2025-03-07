/**
 * @File: linear_mapping.hpp
 * @Date: March 2023
 * @Author: James Swedeen
 *
 * @brief
 * A class that defines the mappings between states when the mapping is trivial, with
 * every state in the truth and navigation models being the same.
 **/

#ifndef KALMAN_FILTER_MAPPINGS_LINEAR_MAPPING_HPP
#define KALMAN_FILTER_MAPPINGS_LINEAR_MAPPING_HPP

/* C++ Headers */
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<kalman_filter/mappings/mappings_base.hpp>

namespace kf
{
namespace map
{
template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
class LinearMapping;

template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using LinearMappingPtr = std::shared_ptr<LinearMapping<DIM_S,SCALAR,OPTIONS>>;

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
class LinearMapping
 : public MappingsBase<DIM_S,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  LinearMapping() noexcept = default;
  /**
   * @Copy Constructor
   **/
  LinearMapping(const LinearMapping&) noexcept = default;
  /**
   * @Move Constructor
   **/
  LinearMapping(LinearMapping&&) noexcept = default;
  /**
   * @Deconstructor
   **/
  ~LinearMapping() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  LinearMapping& operator=(const LinearMapping&)  noexcept = default;
  LinearMapping& operator=(      LinearMapping&&) noexcept = default;
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
inline Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS> LinearMapping<DIM_S,SCALAR,OPTIONS>::
  correctErrors(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,  OPTIONS>>& nav_state,
                const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS>>& error_state)
{
  return nav_state + error_state;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS> LinearMapping<DIM_S,SCALAR,OPTIONS>::
  injectErrors(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,     OPTIONS>>& truth_state,
               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DISP_DIM,OPTIONS>>& truth_disp_state)
{
  return truth_state + truth_disp_state;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS> LinearMapping<DIM_S,SCALAR,OPTIONS>::
  calculateAverageTruthState(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::TRUTH_DIM,OPTIONS>& truth_state_vectors)
{
  return truth_state_vectors.colwise().sum().array() / truth_state_vectors.rows();
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS> LinearMapping<DIM_S,SCALAR,OPTIONS>::
  calculateAverageNavState(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::NAV_DIM,OPTIONS>& nav_state_vectors)
{
  return nav_state_vectors.colwise().sum().array() / nav_state_vectors.rows();
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS> LinearMapping<DIM_S,SCALAR,OPTIONS>::
  calculateErrorState(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& truth_state,
                      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,  OPTIONS>>& nav_state)
{
  return this->mapTruthNav(truth_state) - nav_state;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DISP_DIM,OPTIONS> LinearMapping<DIM_S,SCALAR,OPTIONS>::
  calculateTruthStateDisp(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& truth_state,
                          const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& avg_truth_state)
{
  return truth_state - avg_truth_state;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS> LinearMapping<DIM_S,SCALAR,OPTIONS>::
  calculateNavStateDisp(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& nav_state,
                        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& avg_nav_state)
{
  return nav_state - avg_nav_state;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS> LinearMapping<DIM_S,SCALAR,OPTIONS>::
  mapRefTruth(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,OPTIONS>>& ref_state)
{
  Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS> output;

  if constexpr(DIM_S::TRUTH_DIM >= DIM_S::REF_DIM)
  {
    output.template leftCols< DIM_S::REF_DIM>() = ref_state;
    output.template rightCols<DIM_S::TRUTH_DIM-DIM_S::REF_DIM>().setZero();
  }
  else
  {
    output = ref_state.template leftCols<DIM_S::TRUTH_DIM>();
  }

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS> LinearMapping<DIM_S,SCALAR,OPTIONS>::
  mapRefNav(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,OPTIONS>>& ref_state)
{
  Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS> output;

  if constexpr(DIM_S::NAV_DIM >= DIM_S::REF_DIM)
  {
    output.template leftCols< DIM_S::REF_DIM>() = ref_state;
    output.template rightCols<DIM_S::NAV_DIM-DIM_S::REF_DIM>().setZero();
  }
  else
  {
    output = ref_state.template leftCols<DIM_S::NAV_DIM>();
  }

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS> LinearMapping<DIM_S,SCALAR,OPTIONS>::
  mapTruthNav(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& truth_state)
{
  Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS> output;

  if constexpr(DIM_S::NAV_DIM >= DIM_S::TRUTH_DIM)
  {
    output.template leftCols< DIM_S::TRUTH_DIM>() = truth_state;
    output.template rightCols<DIM_S::NAV_DIM-DIM_S::TRUTH_DIM>().setZero();
  }
  else
  {
    output = truth_state.template leftCols<DIM_S::NAV_DIM>();
  }

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS> LinearMapping<DIM_S,SCALAR,OPTIONS>::
  mapNavTruth(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& nav_state)
{
  Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS> output;

  if constexpr(DIM_S::TRUTH_DIM >= DIM_S::NAV_DIM)
  {
    output.template leftCols< DIM_S::NAV_DIM>() = nav_state;
    output.template rightCols<DIM_S::TRUTH_DIM-DIM_S::NAV_DIM>().setZero();
  }
  else
  {
    output = nav_state.template leftCols<DIM_S::TRUTH_DIM>();
  }

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS> LinearMapping<DIM_S,SCALAR,OPTIONS>::
  getTruthNavMapPDWRDispersionState(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& /* truth_state */)
{
  return Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS>::Identity();
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::ERROR_DIM,OPTIONS> LinearMapping<DIM_S,SCALAR,OPTIONS>::
  getNavTruthMapPDWRErrorState(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& /* nav_state */)
{
  return Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::ERROR_DIM,OPTIONS>::Identity();
}
} // namespace map
} // namespace kf

#endif
/* linear_mapping.hpp */

