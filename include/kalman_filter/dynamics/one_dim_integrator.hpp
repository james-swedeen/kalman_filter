/**
 * @File: one_dim_integrator.hpp
 * @Date: March 2023
 * @Author: James Swedeen
 *
 * @brief
 * Dynamics class for a 1-dimensional second order integrator.
 **/

#ifndef KALMAN_FILTER_DYNAMICS_ONE_DIM_INTEGRATOR_HPP
#define KALMAN_FILTER_DYNAMICS_ONE_DIM_INTEGRATOR_HPP

/* C++ Headers */
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<kalman_filter/dynamics/dynamics_base.hpp>
#include<kalman_filter/helpers/dimension_struct.hpp>

namespace kf
{
namespace dynamics
{
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
class OneDimIntegrator;

template<typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using OneDimIntegratorPtr = std::shared_ptr<OneDimIntegrator<SCALAR,OPTIONS>>;

/**
 * @OneDimIntegratorDim
 **/
struct OneDimIntegratorDim
 : public Dimensions<2,6,6,2,6,6,1,1,5,1,false>
{
public:
  struct REF
  {
  public:
    inline static constexpr const Eigen::Index DISIRED_POS_IND = 0;
    inline static constexpr const Eigen::Index DISIRED_VEL_IND = 1;
  };
  struct TRUTH
  {
  public:
    inline static constexpr const Eigen::Index POS_IND                = 0;
    inline static constexpr const Eigen::Index VEL_IND                = 1;
    inline static constexpr const Eigen::Index DISTURBANCE_ACCEL_IND  = 2;
    inline static constexpr const Eigen::Index ACCEL_BIAS_IND         = 3;
    inline static constexpr const Eigen::Index ACCEL_SCALE_FACTOR_IND = 4;
    inline static constexpr const Eigen::Index ACTUATOR_BIAS_IND      = 5;
  };
  struct NAV
  {
  public:
    inline static constexpr const Eigen::Index POS_IND                = 0;
    inline static constexpr const Eigen::Index VEL_IND                = 1;
    inline static constexpr const Eigen::Index DISTURBANCE_ACCEL_IND  = 2;
    inline static constexpr const Eigen::Index ACCEL_BIAS_IND         = 3;
    inline static constexpr const Eigen::Index ACCEL_SCALE_FACTOR_IND = 4;
    inline static constexpr const Eigen::Index ACTUATOR_BIAS_IND      = 5;
  };
  struct NUM_MEAS
  {
  public:
    inline static constexpr const Eigen::Index POS_IND = 0;
    inline static constexpr const Eigen::Index VEL_IND = 1;
  };
  struct ERROR
  {
  public:
    inline static constexpr const Eigen::Index POS_IND                = 0;
    inline static constexpr const Eigen::Index VEL_IND                = 1;
    inline static constexpr const Eigen::Index DISTURBANCE_ACCEL_IND  = 2;
    inline static constexpr const Eigen::Index ACCEL_BIAS_IND         = 3;
    inline static constexpr const Eigen::Index ACCEL_SCALE_FACTOR_IND = 4;
    inline static constexpr const Eigen::Index ACTUATOR_BIAS_IND      = 5;
  };
  struct TRUTH_DISP
  {
  public:
    inline static constexpr const Eigen::Index POS_IND                = 0;
    inline static constexpr const Eigen::Index VEL_IND                = 1;
    inline static constexpr const Eigen::Index DISTURBANCE_ACCEL_IND  = 2;
    inline static constexpr const Eigen::Index ACCEL_BIAS_IND         = 3;
    inline static constexpr const Eigen::Index ACCEL_SCALE_FACTOR_IND = 4;
    inline static constexpr const Eigen::Index ACTUATOR_BIAS_IND      = 5;
  };
  struct INER_MEAS
  {
  public:
    inline static constexpr const Eigen::Index ACCEL_IND = 0;
  };
  struct CONTROL
  {
  public:
    inline static constexpr const Eigen::Index ACCEL_IND = 0;
  };
  struct TRUTH_NOISE
  {
  public:
    inline static constexpr const Eigen::Index WIND_IND               = 0;
    inline static constexpr const Eigen::Index DISTURBANCE_ACCEL_IND  = 1;
    inline static constexpr const Eigen::Index ACCEL_BIAS_IND         = 2;
    inline static constexpr const Eigen::Index ACCEL_SCALE_FACTOR_IND = 3;
    inline static constexpr const Eigen::Index ACTUATOR_BIAS_IND      = 4;
  };
  struct INER_MEAS_NOISE
  {
  public:
    inline static constexpr const Eigen::Index ACCEL_IND = 0;
  };
};


/**
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options.
 **/
template<typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class OneDimIntegrator
 : public DynamicsBase<OneDimIntegratorDim,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  OneDimIntegrator() = delete;
  /**
   * @Copy Constructor
   **/
  OneDimIntegrator(const OneDimIntegrator&) noexcept = default;
  /**
   * @Move Constructor
   **/
  OneDimIntegrator(OneDimIntegrator&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * Initializes the class for use.
   *
   * @parameters
   * sim_time_step: The time step that the simulation is running at
   * disturbance_accel_time_constant: The first order Gauss Markov time constant for the disturbance accelerations bias
   * accel_bias_time_constant: The first order Gauss Markov time constant for the accelerometer bias
   * accel_scale_factor_time_constant: The first order Gauss Markov time constant for the accelerometer scale factor
   * actuator_bias_time_constant: The first order Gauss Markov time constant for the actuator bias
   **/
  OneDimIntegrator(const SCALAR sim_time_step,
                   const SCALAR disturbance_accel_time_constant,
                   const SCALAR accel_bias_time_constant,
                   const SCALAR accel_scale_factor_time_constant,
                   const SCALAR actuator_bias_time_constant);
  /**
   * @Deconstructor
   **/
  ~OneDimIntegrator() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  OneDimIntegrator& operator=(const OneDimIntegrator&)  noexcept = default;
  OneDimIntegrator& operator=(      OneDimIntegrator&&) noexcept = default;
  /**
   * @getTruthStateDynamics
   *
   * @brief
   * Finds the time derivative of the truth state with additive process noise.
   *
   * @parameters
   * time: The current simulation time
   * truth_state: The current truth state vector
   * control_input: The current control vector
   * process_noise: The process noise on the truth state dynamics
   *
   * @return
   * The time derivative of the truth state vector.
   **/
  inline Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::TRUTH_DIM,OPTIONS>
    getTruthStateDynamics(const SCALAR                                                                                  time,
                          const Eigen::Ref<const Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::TRUTH_DIM,      OPTIONS>>& truth_state,
                          const Eigen::Ref<const Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::CONTROL_DIM,    OPTIONS>>& control_input,
                          const Eigen::Ref<const Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::TRUTH_NOISE_DIM,OPTIONS>>& process_noise) override;
  /**
   * @getNavStateDynamics
   *
   * @brief
   * Finds the time derivative of the navigation state.
   *
   * @parameters
   * time: The current simulation time
   * nav_state: The current navigation state vector
   * inertial_reading: The inertial measurements with biases and noise
   *
   * @return
   * The time derivative of the navigation state vector.
   **/
  inline Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::NAV_DIM,OPTIONS>
    getNavStateDynamics(const SCALAR                                                                                time,
                        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::NAV_DIM,      OPTIONS>>& nav_state,
                        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::REF_DIM,      OPTIONS>>& ref_state,
                        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::INER_MEAS_DIM,OPTIONS>>& inertial_reading) override;
  /**
   * @getTruthStateDynamicsPDWRDispersionState
   *
   * @brief
   * Finds the derivative of the truth state dynamics with respect to the truth dispersion state.
   *
   * @parameters
   * time: The current simulation time
   * truth_state: The current truth state vector
   * control_input: The current control vector
   *
   * @return
   * The derivative of the truth state dynamics with respect to the truth dispersion state.
   **/
  inline Eigen::Matrix<SCALAR,OneDimIntegratorDim::TRUTH_DISP_DIM,OneDimIntegratorDim::TRUTH_DISP_DIM,OPTIONS>
    getTruthStateDynamicsPDWRDispersionState(
      const SCALAR                                                                              time,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::TRUTH_DIM,  OPTIONS>>& truth_state,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::CONTROL_DIM,OPTIONS>>& control_input) override;
  /**
   * @getTruthStateDynamicsPDWRControl
   *
   * @brief
   * Finds the derivative of the truth state dynamics with respect to the control.
   *
   * @parameters
   * time: The current simulation time
   * truth_state: The current truth state vector
   * control_input: The current control vector
   *
   * @return
   * The derivative of the truth state dynamics with respect to the control.
   **/
  inline Eigen::Matrix<SCALAR,OneDimIntegratorDim::CONTROL_DIM,OneDimIntegratorDim::TRUTH_DISP_DIM,OPTIONS>
    getTruthStateDynamicsPDWRControl(
      const SCALAR                                                                              time,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::TRUTH_DIM,  OPTIONS>>& truth_state,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::CONTROL_DIM,OPTIONS>>& control_input) override;
  /**
   * @getTruthStateDynamicsPDWRNoise
   *
   * @brief
   * Finds the derivative of the truth state dynamics with respect to the process noise vector.
   *
   * @parameters
   * time: The current simulation time
   * truth_state: The current truth state vector
   * control_input: The current control vector
   *
   * @return
   * The derivative of the truth state dynamics with respect to the process noise vector.
   **/
  inline Eigen::Matrix<SCALAR,OneDimIntegratorDim::TRUTH_DISP_DIM,OneDimIntegratorDim::TRUTH_NOISE_DIM,OPTIONS>
    getTruthStateDynamicsPDWRNoise(
      const SCALAR                                                                              time,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::TRUTH_DIM,  OPTIONS>>& truth_state,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::CONTROL_DIM,OPTIONS>>& control_input) override;
  /**
   * @getNavStateDynamicsPDWRErrorState
   *
   * @brief
   * Finds the derivative of the navigation state dynamics with respect to the navigation error state.
   *
   * @parameters
   * time: The current simulation time
   * nav_state: The current navigation state vector
   * inertial_reading: The inertial measurements with biases and noise
   *
   * @return
   * The derivative of the navigation state dynamics with respect to the navigation error state.
   **/
  inline Eigen::Matrix<SCALAR,OneDimIntegratorDim::ERROR_DIM,OneDimIntegratorDim::ERROR_DIM,OPTIONS>
    getNavStateDynamicsPDWRErrorState(
      const SCALAR                                                                                time,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::NAV_DIM,      OPTIONS>>& nav_state,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::REF_DIM,      OPTIONS>>& ref_state,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::INER_MEAS_DIM,OPTIONS>>& inertial_reading) override;
  /**
   * @getNavStateDynamicsPDWRInertialMeasurement
   *
   * @brief
   * Finds the derivative of the navigation state dynamics with respect to the inertial measurement vector.
   *
   * @parameters
   * time: The current simulation time
   * nav_state: The current navigation state vector
   * inertial_reading: The inertial measurements with biases and noise
   *
   * @return
   * The derivative of the navigation state dynamics with respect to the navigation error state.
   **/
  inline Eigen::Matrix<SCALAR,OneDimIntegratorDim::INER_MEAS_DIM,OneDimIntegratorDim::ERROR_DIM,OPTIONS>
    getNavStateDynamicsPDWRInertialMeasurement(
      const SCALAR                                                                                time,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::NAV_DIM,      OPTIONS>>& nav_state,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::REF_DIM,      OPTIONS>>& ref_state,
      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::INER_MEAS_DIM,OPTIONS>>& inertial_reading) override;
  /**
   * @findTimeStep
   *
   * @brief
   * Used to calculate the time that is spanned between two state vectors along the simulation vector.
   *
   * @parameters
   * prev_ref_state: The reference state of the previous time step
   * cur_ref_state: The reference state of the current time step
   *
   * @return
   * The time that is spanned between prev_ref_state and cur_ref_state in seconds.
   **/
  inline SCALAR findTimeStep(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::REF_DIM,OPTIONS>>& prev_ref_state,
                             const Eigen::Ref<const Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::REF_DIM,OPTIONS>>& cur_ref_state) override;
private:
  const SCALAR sim_time_step;
  const SCALAR disturbance_accel_time_constant;
  const SCALAR accel_bias_time_constant;
  const SCALAR accel_scale_factor_time_constant;
  const SCALAR actuator_bias_time_constant;
};

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
OneDimIntegrator<SCALAR,OPTIONS>::OneDimIntegrator(const SCALAR sim_time_step,
                                                   const SCALAR disturbance_accel_time_constant,
                                                   const SCALAR accel_bias_time_constant,
                                                   const SCALAR accel_scale_factor_time_constant,
                                                   const SCALAR actuator_bias_time_constant)
 : sim_time_step(sim_time_step),
   disturbance_accel_time_constant(disturbance_accel_time_constant),
   accel_bias_time_constant(accel_bias_time_constant),
   accel_scale_factor_time_constant(accel_scale_factor_time_constant),
   actuator_bias_time_constant(actuator_bias_time_constant)
{}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::TRUTH_DIM,OPTIONS> OneDimIntegrator<SCALAR,OPTIONS>::
  getTruthStateDynamics(const SCALAR                                                                                  /* time */,
                        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::TRUTH_DIM,      OPTIONS>>& truth_state,
                        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::CONTROL_DIM,    OPTIONS>>& control_input,
                        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::TRUTH_NOISE_DIM,OPTIONS>>& process_noise)
{
  Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::TRUTH_DIM,OPTIONS> output;

  output[OneDimIntegratorDim::TRUTH::POS_IND] = truth_state[OneDimIntegratorDim::TRUTH::VEL_IND] +
                                                process_noise[OneDimIntegratorDim::TRUTH_NOISE::WIND_IND];
  output[OneDimIntegratorDim::TRUTH::VEL_IND] = truth_state[OneDimIntegratorDim::TRUTH::DISTURBANCE_ACCEL_IND] +
                                                control_input[OneDimIntegratorDim::CONTROL::ACCEL_IND] +
                                                truth_state[OneDimIntegratorDim::TRUTH::ACTUATOR_BIAS_IND];
  output[OneDimIntegratorDim::TRUTH::DISTURBANCE_ACCEL_IND] =
    ((SCALAR(-1)/this->disturbance_accel_time_constant)*truth_state[OneDimIntegratorDim::TRUTH::DISTURBANCE_ACCEL_IND]) +
    process_noise[OneDimIntegratorDim::TRUTH_NOISE::DISTURBANCE_ACCEL_IND];
  output[OneDimIntegratorDim::TRUTH::ACCEL_BIAS_IND] =
    ((SCALAR(-1)/this->accel_bias_time_constant)*truth_state[OneDimIntegratorDim::TRUTH::ACCEL_BIAS_IND]) +
    process_noise[OneDimIntegratorDim::TRUTH_NOISE::ACCEL_BIAS_IND];
  output[OneDimIntegratorDim::TRUTH::ACCEL_SCALE_FACTOR_IND] =
    ((SCALAR(-1)/this->accel_scale_factor_time_constant)*truth_state[OneDimIntegratorDim::TRUTH::ACCEL_SCALE_FACTOR_IND]) +
    process_noise[OneDimIntegratorDim::TRUTH_NOISE::ACCEL_SCALE_FACTOR_IND];
  output[OneDimIntegratorDim::TRUTH::ACTUATOR_BIAS_IND] =
    ((SCALAR(-1)/this->actuator_bias_time_constant)*truth_state[OneDimIntegratorDim::TRUTH::ACTUATOR_BIAS_IND]) +
    process_noise[OneDimIntegratorDim::TRUTH_NOISE::ACTUATOR_BIAS_IND];

  return output;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::NAV_DIM,OPTIONS> OneDimIntegrator<SCALAR,OPTIONS>::
  getNavStateDynamics(const SCALAR                                                                                /* time */,
                      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::NAV_DIM,      OPTIONS>>& nav_state,
                      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::REF_DIM,      OPTIONS>>& ref_state,
                      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::INER_MEAS_DIM,OPTIONS>>& inertial_reading)
{
  Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::NAV_DIM,OPTIONS> output;

  output[OneDimIntegratorDim::NAV::POS_IND] = nav_state[OneDimIntegratorDim::NAV::VEL_IND];
  output[OneDimIntegratorDim::NAV::VEL_IND] =
    (inertial_reading[OneDimIntegratorDim::INER_MEAS::ACCEL_IND]/(SCALAR(1) + nav_state[OneDimIntegratorDim::NAV::ACCEL_SCALE_FACTOR_IND])) -
    nav_state[OneDimIntegratorDim::NAV::ACCEL_BIAS_IND];
  output[OneDimIntegratorDim::NAV::DISTURBANCE_ACCEL_IND] =
    ((SCALAR(-1)/this->disturbance_accel_time_constant)*nav_state[OneDimIntegratorDim::NAV::DISTURBANCE_ACCEL_IND]);
  output[OneDimIntegratorDim::NAV::ACCEL_BIAS_IND] =
    ((SCALAR(-1)/this->accel_bias_time_constant)*nav_state[OneDimIntegratorDim::NAV::ACCEL_BIAS_IND]);
  output[OneDimIntegratorDim::NAV::ACCEL_SCALE_FACTOR_IND] =
    ((SCALAR(-1)/this->accel_scale_factor_time_constant)*nav_state[OneDimIntegratorDim::NAV::ACCEL_SCALE_FACTOR_IND]);
  output[OneDimIntegratorDim::NAV::ACTUATOR_BIAS_IND] =
    ((SCALAR(-1)/this->actuator_bias_time_constant)*nav_state[OneDimIntegratorDim::NAV::ACTUATOR_BIAS_IND]);

  return output;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,OneDimIntegratorDim::TRUTH_DISP_DIM,OneDimIntegratorDim::TRUTH_DISP_DIM,OPTIONS>
  OneDimIntegrator<SCALAR,OPTIONS>::getTruthStateDynamicsPDWRDispersionState(
    const SCALAR                                                                              /* time */,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::TRUTH_DIM,  OPTIONS>>& /* truth_state */,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::CONTROL_DIM,OPTIONS>>& /* control_input */)
{
  Eigen::Matrix<SCALAR,OneDimIntegratorDim::TRUTH_DISP_DIM,OneDimIntegratorDim::TRUTH_DISP_DIM,OPTIONS> output(
    Eigen::Matrix<SCALAR,OneDimIntegratorDim::TRUTH_DISP_DIM,OneDimIntegratorDim::TRUTH_DISP_DIM,OPTIONS>::Zero());

  output(OneDimIntegratorDim::TRUTH_DISP::POS_IND,OneDimIntegratorDim::TRUTH_DISP::VEL_IND) = 1;

  output(OneDimIntegratorDim::TRUTH_DISP::VEL_IND,OneDimIntegratorDim::TRUTH_DISP::DISTURBANCE_ACCEL_IND) = 1;
  output(OneDimIntegratorDim::TRUTH_DISP::VEL_IND,OneDimIntegratorDim::TRUTH_DISP::ACTUATOR_BIAS_IND) = 1;

  output(OneDimIntegratorDim::TRUTH_DISP::DISTURBANCE_ACCEL_IND,OneDimIntegratorDim::TRUTH_DISP::DISTURBANCE_ACCEL_IND) =
    SCALAR(-1)/this->disturbance_accel_time_constant;
  output(OneDimIntegratorDim::TRUTH_DISP::ACCEL_BIAS_IND,OneDimIntegratorDim::TRUTH_DISP::ACCEL_BIAS_IND) =
    SCALAR(-1)/this->accel_bias_time_constant;
  output(OneDimIntegratorDim::TRUTH_DISP::ACCEL_SCALE_FACTOR_IND,OneDimIntegratorDim::TRUTH_DISP::ACCEL_SCALE_FACTOR_IND) =
    SCALAR(-1)/this->accel_scale_factor_time_constant;
  output(OneDimIntegratorDim::TRUTH_DISP::ACTUATOR_BIAS_IND,OneDimIntegratorDim::TRUTH_DISP::ACTUATOR_BIAS_IND) =
    SCALAR(-1)/this->actuator_bias_time_constant;

  return output;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,OneDimIntegratorDim::CONTROL_DIM,OneDimIntegratorDim::TRUTH_DISP_DIM,OPTIONS>
  OneDimIntegrator<SCALAR,OPTIONS>::getTruthStateDynamicsPDWRControl(
    const SCALAR                                                                              /* time */,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::TRUTH_DIM,  OPTIONS>>& /* truth_state */,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::CONTROL_DIM,OPTIONS>>& /* control_input */)
{
  Eigen::Matrix<SCALAR,OneDimIntegratorDim::CONTROL_DIM,OneDimIntegratorDim::TRUTH_DISP_DIM,OPTIONS> output(
    Eigen::Matrix<SCALAR,OneDimIntegratorDim::TRUTH_DISP_DIM,OneDimIntegratorDim::CONTROL_DIM,OPTIONS>::Zero());

  output[OneDimIntegratorDim::TRUTH_DISP::VEL_IND] = 1;

  return output;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,OneDimIntegratorDim::TRUTH_DISP_DIM,OneDimIntegratorDim::TRUTH_NOISE_DIM,OPTIONS>
  OneDimIntegrator<SCALAR,OPTIONS>::getTruthStateDynamicsPDWRNoise(
    const SCALAR                                                                              /* time */,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::TRUTH_DIM,  OPTIONS>>& /* truth_state */,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::CONTROL_DIM,OPTIONS>>& /* control_input */)
{
  Eigen::Matrix<SCALAR,OneDimIntegratorDim::TRUTH_DISP_DIM,OneDimIntegratorDim::TRUTH_NOISE_DIM,OPTIONS> output(
    Eigen::Matrix<SCALAR,OneDimIntegratorDim::TRUTH_DISP_DIM,OneDimIntegratorDim::TRUTH_NOISE_DIM,OPTIONS>::Zero());

  output(OneDimIntegratorDim::TRUTH_DISP::POS_IND,               OneDimIntegratorDim::TRUTH_NOISE::WIND_IND)               = 1;
  output(OneDimIntegratorDim::TRUTH_DISP::DISTURBANCE_ACCEL_IND, OneDimIntegratorDim::TRUTH_NOISE::DISTURBANCE_ACCEL_IND)  = 1;
  output(OneDimIntegratorDim::TRUTH_DISP::ACCEL_BIAS_IND,        OneDimIntegratorDim::TRUTH_NOISE::ACCEL_BIAS_IND)         = 1;
  output(OneDimIntegratorDim::TRUTH_DISP::ACCEL_SCALE_FACTOR_IND,OneDimIntegratorDim::TRUTH_NOISE::ACCEL_SCALE_FACTOR_IND) = 1;
  output(OneDimIntegratorDim::TRUTH_DISP::ACTUATOR_BIAS_IND,     OneDimIntegratorDim::TRUTH_NOISE::ACTUATOR_BIAS_IND)      = 1;

  return output;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,OneDimIntegratorDim::ERROR_DIM,OneDimIntegratorDim::ERROR_DIM,OPTIONS>
  OneDimIntegrator<SCALAR,OPTIONS>::getNavStateDynamicsPDWRErrorState(
    const SCALAR                                                                                /* time */,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::NAV_DIM,      OPTIONS>>& nav_state,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::REF_DIM,      OPTIONS>>& ref_state,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::INER_MEAS_DIM,OPTIONS>>& inertial_reading)
{
  Eigen::Matrix<SCALAR,OneDimIntegratorDim::ERROR_DIM,OneDimIntegratorDim::ERROR_DIM,OPTIONS> output(
    Eigen::Matrix<SCALAR,OneDimIntegratorDim::ERROR_DIM,OneDimIntegratorDim::ERROR_DIM,OPTIONS>::Zero());

  output(OneDimIntegratorDim::ERROR::POS_IND,OneDimIntegratorDim::ERROR::VEL_IND) = 1;

  output(OneDimIntegratorDim::ERROR::VEL_IND,OneDimIntegratorDim::ERROR::ACCEL_BIAS_IND) = -1;
  output(OneDimIntegratorDim::ERROR::VEL_IND,OneDimIntegratorDim::ERROR::DISTURBANCE_ACCEL_IND) =
    -inertial_reading[OneDimIntegratorDim::INER_MEAS::ACCEL_IND]/(std::pow(SCALAR(1) + nav_state[OneDimIntegratorDim::NAV::ACCEL_SCALE_FACTOR_IND], 2));

  output(OneDimIntegratorDim::ERROR::DISTURBANCE_ACCEL_IND,OneDimIntegratorDim::ERROR::DISTURBANCE_ACCEL_IND) =
    SCALAR(-1)/this->disturbance_accel_time_constant;
  output(OneDimIntegratorDim::ERROR::ACCEL_BIAS_IND,OneDimIntegratorDim::ERROR::ACCEL_BIAS_IND) =
    SCALAR(-1)/this->accel_bias_time_constant;
  output(OneDimIntegratorDim::ERROR::ACCEL_SCALE_FACTOR_IND,OneDimIntegratorDim::ERROR::ACCEL_SCALE_FACTOR_IND) =
    SCALAR(-1)/this->accel_scale_factor_time_constant;
  output(OneDimIntegratorDim::ERROR::ACTUATOR_BIAS_IND,OneDimIntegratorDim::ERROR::ACTUATOR_BIAS_IND) =
    SCALAR(-1)/this->actuator_bias_time_constant;

  return output;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,OneDimIntegratorDim::INER_MEAS_DIM,OneDimIntegratorDim::ERROR_DIM,OPTIONS>
  OneDimIntegrator<SCALAR,OPTIONS>::getNavStateDynamicsPDWRInertialMeasurement(
    const SCALAR                                                                                /* time */,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::NAV_DIM,      OPTIONS>>& nav_state,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::REF_DIM,      OPTIONS>>& ref_state,
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::INER_MEAS_DIM,OPTIONS>>& /* inertial_reading */)
{
  Eigen::Matrix<SCALAR,OneDimIntegratorDim::INER_MEAS_DIM,OneDimIntegratorDim::ERROR_DIM,OPTIONS> output(
    Eigen::Matrix<SCALAR,OneDimIntegratorDim::INER_MEAS_DIM,OneDimIntegratorDim::ERROR_DIM,OPTIONS>::Zero());

  output[OneDimIntegratorDim::ERROR::VEL_IND] =
    SCALAR(1)/(SCALAR(1) + nav_state[OneDimIntegratorDim::ERROR::ACCEL_SCALE_FACTOR_IND]);

  return output;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR OneDimIntegrator<SCALAR,OPTIONS>::
  findTimeStep(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::REF_DIM,OPTIONS>>& /* prev_ref_state */,
               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,OneDimIntegratorDim::REF_DIM,OPTIONS>>& /* cur_ref_state */)
{
  return this->sim_time_step;
}
} // namespace dynamics
} // namespace kf

#endif
/* one_dim_integrator.hpp */
