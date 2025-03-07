/**
 * @File: open_loop_controller.hpp
 * @Date: May 2022
 * @Author: James Swedeen
 *
 * @brief
 * A base class for controllers.
 **/

#ifndef KALMAN_FILTER_CONTROLLERS_OPEN_LOOP_CONTROLLER_HPP
#define KALMAN_FILTER_CONTROLLERS_OPEN_LOOP_CONTROLLER_HPP

/* C++ Headers */
#include<cstdint>
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<kalman_filter/controllers/controller_base.hpp>

namespace kf
{
namespace control
{
template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
class OpenLoopController;

template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using OpenLoopControllerPtr = std::shared_ptr<OpenLoopController<DIM_S,SCALAR,OPTIONS>>;

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
class OpenLoopController
 : public ControllerBase<DIM_S,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  OpenLoopController() noexcept = default;
  /**
   * @Copy Constructor
   **/
  OpenLoopController(const OpenLoopController&) noexcept = default;
  /**
   * @Move Constructor
   **/
  OpenLoopController(OpenLoopController&&) noexcept = default;
  /**
   * @Deconstructor
   **/
  ~OpenLoopController() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  OpenLoopController& operator=(const OpenLoopController&)  noexcept = default;
  OpenLoopController& operator=(      OpenLoopController&&) noexcept = default;
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
  inline Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS>
    getControl(const SCALAR                                                            time,
               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& nav_state,
               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,OPTIONS>>& ref_state,
               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,OPTIONS>>& next_ref_state,
               const SCALAR                                                            sim_dt) override;
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
  inline Eigen::Matrix<SCALAR,DIM_S::CONTROL_DIM,DIM_S::ERROR_DIM,OPTIONS>
    getControlPDWRErrorState(const SCALAR                                                            time,
                             const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& nav_state,
                             const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,OPTIONS>>& ref_state) override;
};

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS> OpenLoopController<DIM_S,SCALAR,OPTIONS>::
  getControl(const SCALAR                                                            /* time */,
             const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& /* nav_state */,
             const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,OPTIONS>>& ref_state,
             const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,OPTIONS>>& /* next_ref_state */,
             const SCALAR                                                            /* sim_dt */)
{
  Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS> output;

  output[DIM_S::CONTROL::ROLL_RATE_IND]  = ref_state[DIM_S::REF::ROLL_RATE_IND];
  output[DIM_S::CONTROL::PITCH_RATE_IND] = ref_state[DIM_S::REF::PITCH_RATE_IND];
  output[DIM_S::CONTROL::YAW_RATE_IND]   = ref_state[DIM_S::REF::YAW_RATE_IND];
  output[DIM_S::CONTROL::X_ACCEL_IND]    = ref_state[DIM_S::REF::X_ACCEL_IND];
  output[DIM_S::CONTROL::Y_ACCEL_IND]    = ref_state[DIM_S::REF::Y_ACCEL_IND];
  output[DIM_S::CONTROL::Z_ACCEL_IND]    = ref_state[DIM_S::REF::Z_ACCEL_IND];

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,DIM_S::CONTROL_DIM,DIM_S::ERROR_DIM,OPTIONS> OpenLoopController<DIM_S,SCALAR,OPTIONS>::
  getControlPDWRErrorState(const SCALAR                                                            /* time */,
                           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& /* nav_state */,
                           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,OPTIONS>>& /* ref_state */)
{
  Eigen::Matrix<SCALAR,DIM_S::CONTROL_DIM,DIM_S::ERROR_DIM,OPTIONS> output;
  output.setZero();
  return output;
}
} // namespace control
} // namespace kf

#endif
/* open_loop_controller.hpp */
