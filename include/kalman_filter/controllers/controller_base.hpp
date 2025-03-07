/**
 * @File: controller_base.hpp
 * @Date: May 2022
 * @Author: James Swedeen
 *
 * @brief
 * A base class for controllers.
 **/

#ifndef KALMAN_FILTER_CONTROLLERS_CONTROLLER_BASE_HPP
#define KALMAN_FILTER_CONTROLLERS_CONTROLLER_BASE_HPP

/* C++ Headers */
#include<cstdint>
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */

namespace kf
{
namespace control
{
template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
class ControllerBase;

template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using ControllerBasePtr = std::shared_ptr<ControllerBase<DIM_S,SCALAR,OPTIONS>>;

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
class ControllerBase
{
public:
  /**
   * @Default Constructor
   **/
  ControllerBase() noexcept = default;
  /**
   * @Copy Constructor
   **/
  ControllerBase(const ControllerBase&) noexcept = default;
  /**
   * @Move Constructor
   **/
  ControllerBase(ControllerBase&&) noexcept = default;
  /**
   * @Deconstructor
   **/
  virtual ~ControllerBase() noexcept = default;
  /**
   * @Assignment Operators
   **/
  ControllerBase& operator=(const ControllerBase&)  noexcept = default;
  ControllerBase& operator=(      ControllerBase&&) noexcept = default;
  /**
   * @getControl
   *
   * @brief
   * Used to find the control input for this step in the simulation.
   *
   * @parameters
   * time: The current simulation time
   * nav_state: The current navigation state vector
   * ref_state: The current reference state vector
   * next_ref_state: The next reference state vector
   * sim_dt: The time delta between the current and next reference states
   *
   * @return
   * The control for this time step.
   **/
  inline virtual Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS>
    getControl(const SCALAR                                                            time,
               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& nav_state,
               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,OPTIONS>>& ref_state,
               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,OPTIONS>>& next_ref_state,
               const SCALAR                                                            sim_dt) = 0;
  /**
   * @getControlPDWRErrorState
   *
   * @brief
   * Used to find the partial derivative of the control with respect to the error state.
   *
   * @parameters
   * time: The current simulation time
   * nav_state: The current navigation state vector
   * ref_state: The current reference state vector
   *
   * @return
   * The control for this time step.
   **/
  inline virtual Eigen::Matrix<SCALAR,DIM_S::CONTROL_DIM,DIM_S::ERROR_DIM,OPTIONS>
    getControlPDWRErrorState(const SCALAR                                                            time,
                             const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& nav_state,
                             const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,OPTIONS>>& ref_state) = 0;
};
} // namespace control
} // namespace kf

#endif
/* controller_base.hpp */
