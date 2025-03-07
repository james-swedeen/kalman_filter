/**
 * @File: mappings_base.hpp
 * @Date: April 2022
 * @Author: James Swedeen
 *
 * @brief
 * A base class for the mapping between various state vectors.
 **/

#ifndef KALMAN_FILTER_MAPPINGS_MAPPINGS_BASE_HPP
#define KALMAN_FILTER_MAPPINGS_MAPPINGS_BASE_HPP

/* C++ Headers */
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */

namespace kf
{
namespace map
{
template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
class MappingsBase;

template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using MappingsBasePtr = std::shared_ptr<MappingsBase<DIM_S,SCALAR,OPTIONS>>;

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
class MappingsBase
{
public:
  /**
   * @Default Constructor
   **/
  MappingsBase() noexcept = default;
  /**
   * @Copy Constructor
   **/
  MappingsBase(const MappingsBase&) noexcept = default;
  /**
   * @Move Constructor
   **/
  MappingsBase(MappingsBase&&) noexcept = default;
  /**
   * @Deconstructor
   **/
  virtual ~MappingsBase() noexcept = default;
  /**
   * @Assignment Operators
   **/
  MappingsBase& operator=(const MappingsBase&)  noexcept = default;
  MappingsBase& operator=(      MappingsBase&&) noexcept = default;
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
  inline virtual Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>
    correctErrors(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,  OPTIONS>>& nav_state,
                  const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS>>& error_state) = 0;
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
  inline virtual Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>
    injectErrors(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,     OPTIONS>>& truth_state,
                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DISP_DIM,OPTIONS>>& truth_disp_state) = 0;
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
  inline virtual Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>
    calculateAverageTruthState(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::TRUTH_DIM,OPTIONS>& truth_state_vectors) = 0;
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
  inline virtual Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>
    calculateAverageNavState(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::NAV_DIM,OPTIONS>& nav_state_vectors) = 0;
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
  inline virtual Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS>
    calculateErrorState(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& truth_state,
                        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,  OPTIONS>>& nav_state) = 0;
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
  inline virtual Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DISP_DIM,OPTIONS>
    calculateTruthStateDisp(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& truth_state,
                            const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& avg_truth_state) = 0;
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
  inline virtual Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,OPTIONS>
    calculateNavStateDisp(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& nav_state,
                          const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& avg_nav_state) = 0;
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
  inline virtual Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>
    mapRefTruth(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,OPTIONS>>& ref_state) = 0;
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
  inline virtual Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>
    mapRefNav(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,OPTIONS>>& ref_state) = 0;
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
  inline virtual Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>
    mapTruthNav(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& truth_state) = 0;
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
  inline virtual Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>
    mapNavTruth(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& nav_state) = 0;
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
  inline virtual Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS>
    getTruthNavMapPDWRDispersionState(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS>>& truth_state) = 0;
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
  inline virtual Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::ERROR_DIM,OPTIONS>
    getNavTruthMapPDWRErrorState(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& nav_state) = 0;
};
} // namespace map
} // namespace kf

#endif
/* mappings_base.hpp */

