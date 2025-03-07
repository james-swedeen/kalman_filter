/**
 * @File: dubins_airplane_controller.hpp
 * @Date: July 2023
 * @Author: James Swedeen
 *
 * @brief
 * A controller class for the Dubin's Airplane.
 **/

#ifndef KALMAN_FILTER_CONTROLLERS_DUBINS_AIRPLANE_CONTROLLER_HPP
#define KALMAN_FILTER_CONTROLLERS_DUBINS_AIRPLANE_CONTROLLER_HPP

/* C++ Headers */
#include<cstdint>
#include<memory>
#include<limits>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<kalman_filter/controllers/controller_base.hpp>
#include<kalman_filter/math/quaternion.hpp>
#include<kalman_filter/dynamics/dubins_airplane_model.hpp>
#include<kalman_filter/mappings/dubins_airplane_mapping.hpp>

namespace kf
{
namespace control
{
template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
class DubinsAirplaneController;

template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using DubinsAirplaneControllerPtr = std::shared_ptr<DubinsAirplaneController<DIM_S,SCALAR,OPTIONS>>;

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
class DubinsAirplaneController
 : public ControllerBase<DIM_S,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  DubinsAirplaneController() = delete;
  /**
   * @Copy Constructor
   **/
  DubinsAirplaneController(const DubinsAirplaneController&) noexcept = default;
  /**
   * @Move Constructor
   **/
  DubinsAirplaneController(DubinsAirplaneController&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * Initializes the object for use.
   *
   * @parameters
   * gravity_accel: The magnitude of the acceleration from gravity
   * yaw_inf_error: Max yaw error along the reference
   * lat_error_gain: Gain that controls how fast lateral errors are corrected
   * forward_proportional_gain: Proportional gain in the forward direction
   * yaw_proportional_gain: Proportional gain in the yaw direction
   * down_proportional_gain: Proportional gain in the down direction from the bode fixed frame
   * yaw_derivative_gain: Derivative gain for the yaw controller
   * down_derivative_gain: Derivative gain for the down controller
   * forward_derivative_gain: Derivative gain for the forward controller
   **/
  DubinsAirplaneController(const dynamics::DubinsAirplanePtr<  DIM_S,SCALAR,OPTIONS>& dynamics,
                           const map::DubinsAirplaneMappingPtr<DIM_S,SCALAR,OPTIONS>& mappings,
                           const SCALAR                                               gravity_accel,
                           const SCALAR                                               yaw_inf_error,
                           const SCALAR                                               lat_error_gain,
                           const SCALAR                                               forward_proportional_gain,
                           const SCALAR                                               yaw_proportional_gain,
                           const SCALAR                                               down_proportional_gain,
                           const SCALAR                                               yaw_derivative_gain,
                           const SCALAR                                               down_derivative_gain,
                           const SCALAR                                               forward_derivative_gain);
  /**
   * @Deconstructor
   **/
  ~DubinsAirplaneController() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  DubinsAirplaneController& operator=(const DubinsAirplaneController&)  noexcept = default;
  DubinsAirplaneController& operator=(      DubinsAirplaneController&&) noexcept = default;
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
private:
  SCALAR                                              gravity_accel;
  SCALAR                                              yaw_inf_error;
  SCALAR                                              lat_error_gain;
  SCALAR                                              forward_proportional_gain;
  SCALAR                                              yaw_proportional_gain;
  SCALAR                                              down_proportional_gain;
  SCALAR                                              yaw_derivative_gain;
  SCALAR                                              down_derivative_gain;
  SCALAR                                              forward_derivative_gain;
  dynamics::DubinsAirplanePtr<  DIM_S,SCALAR,OPTIONS> dynamics;
  map::DubinsAirplaneMappingPtr<DIM_S,SCALAR,OPTIONS> mappings;
};


template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
DubinsAirplaneController<DIM_S,SCALAR,OPTIONS>::
  DubinsAirplaneController(const dynamics::DubinsAirplanePtr<  DIM_S,SCALAR,OPTIONS>& dynamics,
                           const map::DubinsAirplaneMappingPtr<DIM_S,SCALAR,OPTIONS>& mappings,
                           const SCALAR                                               gravity_accel,
                           const SCALAR                                               yaw_inf_error,
                           const SCALAR                                               lat_error_gain,
                           const SCALAR                                               forward_proportional_gain,
                           const SCALAR                                               yaw_proportional_gain,
                           const SCALAR                                               down_proportional_gain,
                           const SCALAR                                               yaw_derivative_gain,
                           const SCALAR                                               down_derivative_gain,
                           const SCALAR                                               forward_derivative_gain)
 : gravity_accel(gravity_accel),
   yaw_inf_error(yaw_inf_error),
   lat_error_gain(lat_error_gain),
   forward_proportional_gain(forward_proportional_gain),
   yaw_proportional_gain(yaw_proportional_gain),
   down_proportional_gain(down_proportional_gain),
   yaw_derivative_gain(yaw_derivative_gain),
   down_derivative_gain(down_derivative_gain),
   forward_derivative_gain(forward_derivative_gain),
   dynamics(dynamics),
   mappings(mappings)
{}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS> DubinsAirplaneController<DIM_S,SCALAR,OPTIONS>::
  getControl(const SCALAR                                                            time,
             const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& nav_state,
             const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,OPTIONS>>& ref_state,
             const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,OPTIONS>>& next_ref_state,
             const SCALAR                                                            sim_dt)
{
  // Helper variables
  const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS> ref_state_truth      = this->mappings->mapRefTruth(ref_state);
  const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS> next_ref_state_truth = this->mappings->mapRefTruth(next_ref_state);

  const Eigen::Matrix<SCALAR,3,3,OPTIONS> ned_to_path_rotation = math::quat::rollPitchYawToDirectionCosineMatrix(ref_state.template middleCols<3>(DIM_S::REF::EULER_START_IND)).transpose();
  const Eigen::Matrix<SCALAR,1,3,OPTIONS> path_error           = ned_to_path_rotation * (ref_state.template middleCols<3>(DIM_S::REF::POS_START_IND) - nav_state.template middleCols<3>(DIM_S::NAV::POS_START_IND)).transpose();
  const Eigen::Matrix<SCALAR,1,3,OPTIONS> path_error_rate      = -ned_to_path_rotation * nav_state.template middleCols<3>(DIM_S::NAV::VEL_START_IND).transpose();
  const Eigen::Matrix<SCALAR,1,3,OPTIONS> nav_rpy              = math::quat::quaternionToRollPitchYaw(nav_state.template middleCols<4>(DIM_S::NAV::QUAT_START_IND));

  const SCALAR commanded_yaw = ref_state[DIM_S::REF::YAW_IND] +
                               (this->yaw_inf_error * (SCALAR(2)/math::pi<SCALAR>()) * std::atan(this->lat_error_gain * path_error[1]));
  const SCALAR yaw_error = commanded_yaw - nav_rpy[2];

  const Eigen::Matrix<SCALAR,3,3,OPTIONS> ned_to_body_rotation = math::quat::quaternionToDirectionCosineMatrix(nav_state.template middleCols<4>(DIM_S::NAV::QUAT_START_IND)).transpose();
  const Eigen::Matrix<SCALAR,1,3,OPTIONS> vel_body             = ned_to_body_rotation * nav_state.template middleCols<3>(DIM_S::NAV::VEL_START_IND).transpose();

  // Find feedback term
  Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS> feedback;

  feedback[DIM_S::CONTROL::ROLL_RATE_IND]      = (this->yaw_proportional_gain     * std::atan2(std::sin(yaw_error), std::cos(yaw_error)))
                                               - (this->yaw_derivative_gain       * ((this->gravity_accel * std::tan(nav_rpy[0])) / vel_body[0]));
  feedback[DIM_S::CONTROL::PITCH_RATE_IND]     = (this->down_proportional_gain    * -path_error[2])
                                               - (this->down_derivative_gain      * path_error_rate[2]);
  feedback[DIM_S::CONTROL::AIR_SPEED_RATE_IND] = (this->forward_proportional_gain * path_error[0])
                                               + (this->forward_derivative_gain   * path_error_rate[0]);

  // Find feed forward term
  const Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS> truth_dynamics_mat =
    this->dynamics->getTruthStateDynamicsPDWRDispersionState(time, ref_state_truth, Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS>::Constant(std::numeric_limits<SCALAR>::quiet_NaN()));
  const Eigen::Matrix<SCALAR,DIM_S::CONTROL_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS> truth_control_mat =
    this->dynamics->getTruthStateDynamicsPDWRControl(time, ref_state_truth, Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS>::Constant(std::numeric_limits<SCALAR>::quiet_NaN()));

  Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,OPTIONS> ff_error_vec =
    next_ref_state_truth.transpose() - ((Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS>::Identity() + (sim_dt * truth_dynamics_mat)) * ref_state_truth.transpose());
  ff_error_vec[DIM_S::TRUTH::ROLL_IND]  = std::atan2(std::sin(ff_error_vec[DIM_S::TRUTH::ROLL_IND]),  std::cos(ff_error_vec[DIM_S::TRUTH::ROLL_IND]));
  ff_error_vec[DIM_S::TRUTH::PITCH_IND] = std::atan2(std::sin(ff_error_vec[DIM_S::TRUTH::PITCH_IND]), std::cos(ff_error_vec[DIM_S::TRUTH::PITCH_IND]));
  ff_error_vec[DIM_S::TRUTH::YAW_IND]   = std::atan2(std::sin(ff_error_vec[DIM_S::TRUTH::YAW_IND]),   std::cos(ff_error_vec[DIM_S::TRUTH::YAW_IND]));

  const Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS> feed_forward =
    truth_control_mat.transpose().colPivHouseholderQr().solve(ff_error_vec.transpose() * (SCALAR(1)/sim_dt));

  return feedback + feed_forward;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,DIM_S::CONTROL_DIM,DIM_S::ERROR_DIM,OPTIONS> DubinsAirplaneController<DIM_S,SCALAR,OPTIONS>::
  getControlPDWRErrorState(const SCALAR                                                            time,
                           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,OPTIONS>>& nav_state,
                           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,OPTIONS>>& ref_state)
{
  const Eigen::Matrix<SCALAR,3,3,OPTIONS> ned_to_path_rotation = math::quat::rollPitchYawToDirectionCosineMatrix(ref_state.template middleCols<3>(DIM_S::REF::EULER_START_IND)).transpose();
  const Eigen::Matrix<SCALAR,1,3,OPTIONS> path_error           = ned_to_path_rotation * (ref_state.template middleCols<3>(DIM_S::REF::POS_START_IND) - nav_state.template middleCols<3>(DIM_S::NAV::POS_START_IND)).transpose();
  const Eigen::Matrix<SCALAR,1,3,OPTIONS> nav_rpy              = math::quat::quaternionToRollPitchYaw(nav_state.template middleCols<4>(DIM_S::NAV::QUAT_START_IND));

  const Eigen::Matrix<SCALAR,3,3,OPTIONS> ned_to_body_rotation = math::quat::quaternionToDirectionCosineMatrix(nav_state.template middleCols<4>(DIM_S::NAV::QUAT_START_IND)).transpose();
  const Eigen::Matrix<SCALAR,1,3,OPTIONS> vel_body             = ned_to_body_rotation * nav_state.template middleCols<3>(DIM_S::NAV::VEL_START_IND).transpose();

  const SCALAR c_pitch = std::cos(nav_rpy[1]);
  const SCALAR s_pitch = std::sin(nav_rpy[1]);
  const SCALAR c_yaw   = std::cos(nav_rpy[2]);
  const SCALAR s_yaw   = std::sin(nav_rpy[2]);

  Eigen::Matrix<SCALAR,DIM_S::CONTROL_DIM,DIM_S::ERROR_DIM,OPTIONS> output;
  output.setZero();

  const SCALAR roll_rate_pd_air_speed = -this->yaw_derivative_gain * (-this->gravity_accel / std::pow(vel_body[0], SCALAR(2))) * std::tan(nav_rpy[0]);
  output(DIM_S::CONTROL::ROLL_RATE_IND, DIM_S::ERROR::YAW_IND) =
    -this->yaw_proportional_gain
    + (roll_rate_pd_air_speed * ((-nav_state[DIM_S::NAV::NORTH_VEL_IND] * s_yaw * c_pitch) + (nav_state[DIM_S::NAV::EAST_VEL_IND] * c_yaw * c_pitch)));
  output.template block<1,3>(DIM_S::CONTROL::ROLL_RATE_IND, DIM_S::ERROR::POS_START_IND) =
    -(this->yaw_proportional_gain * this->yaw_inf_error * (SCALAR(2)/math::pi<SCALAR>()) * (this->lat_error_gain/(SCALAR(1) + std::pow(this->lat_error_gain * path_error[1], SCALAR(2))))) *
    ned_to_path_rotation.row(1);
  output(DIM_S::CONTROL::ROLL_RATE_IND, DIM_S::ERROR::ROLL_IND) = -this->yaw_derivative_gain * (this->gravity_accel / vel_body[0]) * (SCALAR(1) / std::pow(std::cos(nav_rpy[0]), SCALAR(2)));
  output.template block<1,3>(DIM_S::CONTROL::ROLL_RATE_IND, DIM_S::ERROR::VEL_START_IND) = roll_rate_pd_air_speed * ned_to_body_rotation.template topRows<1>();
  output(DIM_S::CONTROL::ROLL_RATE_IND, DIM_S::ERROR::PITCH_IND) = roll_rate_pd_air_speed * ((-nav_state[DIM_S::NAV::NORTH_VEL_IND] * c_yaw * s_pitch) - (nav_state[DIM_S::NAV::EAST_VEL_IND] * s_yaw * s_pitch) - (nav_state[DIM_S::NAV::DOWN_VEL_IND] * c_pitch));

  output.template block<1,3>(DIM_S::CONTROL::PITCH_RATE_IND, DIM_S::ERROR::POS_START_IND) = this->down_proportional_gain * ned_to_path_rotation.row(2);
  output.template block<1,3>(DIM_S::CONTROL::PITCH_RATE_IND, DIM_S::ERROR::VEL_START_IND) = this->down_derivative_gain   * ned_to_path_rotation.row(2);

  output.template block<1,3>(DIM_S::CONTROL::AIR_SPEED_RATE_IND, DIM_S::ERROR::POS_START_IND) = -this->forward_proportional_gain * ned_to_path_rotation.row(0);
  output.template block<1,3>(DIM_S::CONTROL::AIR_SPEED_RATE_IND, DIM_S::ERROR::VEL_START_IND) = -this->forward_derivative_gain   * ned_to_path_rotation.row(0);

  return output;
}
} // namespace control
} // namespace kf

#endif
/* dubins_airplane_controller.hpp */
