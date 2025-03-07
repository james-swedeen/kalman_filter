/**
 * @File: versions.hpp
 * @Date: April 2022
 * @Author: James Swedeen
 *
 * @brief
 * Used to control which version of a Kalman filter will be run.
 **/

#ifndef KALMAN_FILTER_HELPERS_VERSIONS_HPP
#define KALMAN_FILTER_HELPERS_VERSIONS_HPP

/* C++ Headers */
#include<cstdint>

namespace kf
{
enum Versions : uint64_t
{
  /**
   * @NULL_VERSION
   *
   * This enumeration explicitly represents nothing.
   * The result of using it will be the baseline Kalman filter.
   **/
  NULL_VERSION = 0x0000'0000'0000'0000,
  /**
   * @OPEN_LOOP
   *
   * If enabled the Kalman filter will assume that the control commands are zeroed out.
   **/
  OPEN_LOOP = 0x0000'0000'0000'0001,
  /**
   * @MODEL_REPLACEMENT
   *
   * If enabled the Kalman filter will assume that the navigation state is using inertial measurement to propagate.
   **/
  MODEL_REPLACEMENT = 0x0000'0000'0000'0002,
  /**
   * @APPLY_STATE_BOUNDS
   *
   * If true, the kalman filter will apply bounds on the states specified in the tools object.
   **/
  APPLY_STATE_BOUNDS = 0x0000'0000'0000'0004,
  /**
   * @RUNNING_ERROR_BUDGET
   *
   * To be used internally whenever the code is performing an error budget analysis.
   * When enabled lincov will use the pre-existing error covariance values.
   **/
  RUNNING_ERROR_BUDGET = 0x0000'8000'0000'0000,
  /**
   * @INTEGRATOR_BIT_FLAG
   *
   * These bits are set aside for determining what type of integration to use.
   **/
  INTEGRATOR_BIT_FLAG = 0xFFFF'0000'0000'0000,
  /**
   * @EULER_INTEGRATION
   *
   * Use Euler integration.
   **/
  EULER_INTEGRATION = 0x0001'0000'0000'0000,
  /**
   * @RK4_INTEGRATION
   *
   * Use RK4 integration.
   **/
  RK4_INTEGRATION = 0x0002'0000'0000'0000,
  /**
   * @ODE45_INTEGRATION
   *
   * Use Boost's ODE45 integration.
   **/
  ODE45_INTEGRATION = 0x0003'0000'0000'0000,
};

/**
 * @valid
 *
 * @brief
 * Tests to see if the given configuration is valid.
 *
 * @parameters
 * config: The configuration to test
 **/
template<Versions VERSION>
constexpr void monteCarloValid() noexcept;
template<Versions VERSION>
constexpr void linCovValid()     noexcept;

/**
 * @test
 *
 * @brief
 * Each is used to test if a given attribute is held in the
 * given configuration.
 *
 * @parameters
 * config: The configuration to test
 *
 * @return
 * True if the attribute asked about is true in the configuration given.
 **/
constexpr bool nullVersion(       const Versions config) noexcept;
constexpr bool openLoop(          const Versions config) noexcept;
constexpr bool modelReplacement(  const Versions config) noexcept;
constexpr bool runningErrorBudget(const Versions config) noexcept;
constexpr bool applyStateBounds(  const Versions config) noexcept;
constexpr bool eulerIntegration(  const Versions config) noexcept;
constexpr bool rk4Integration(    const Versions config) noexcept;
constexpr bool ode45Integration(  const Versions config) noexcept;
} // namespace kf


template<kf::Versions VERSION>
constexpr void kf::monteCarloValid() noexcept
{
  static_assert(not runningErrorBudget(VERSION));
  if constexpr(eulerIntegration(VERSION))
  {
    static_assert(not rk4Integration(VERSION));
    static_assert(not ode45Integration(VERSION));
  }
  if constexpr(rk4Integration(VERSION))
  {
    static_assert(not eulerIntegration(VERSION));
    static_assert(not ode45Integration(VERSION));
  }
  if constexpr(ode45Integration(VERSION))
  {
    static_assert(not rk4Integration(VERSION));
    static_assert(not eulerIntegration(VERSION));
  }
  static_assert(eulerIntegration(VERSION) or rk4Integration(VERSION) or ode45Integration(VERSION));
}

template<kf::Versions VERSION>
constexpr void kf::linCovValid() noexcept
{
  static_assert(not applyStateBounds(VERSION), "This isn't possible in LinCov");
  if constexpr(eulerIntegration(VERSION))
  {
    static_assert(not rk4Integration(VERSION));
    static_assert(not ode45Integration(VERSION));
  }
  if constexpr(rk4Integration(VERSION))
  {
    static_assert(not eulerIntegration(VERSION));
    static_assert(not ode45Integration(VERSION));
  }
  if constexpr(ode45Integration(VERSION))
  {
    static_assert(not rk4Integration(VERSION));
    static_assert(not eulerIntegration(VERSION));
  }
  static_assert(eulerIntegration(VERSION) or rk4Integration(VERSION) or ode45Integration(VERSION));
}

constexpr bool kf::nullVersion(const Versions config) noexcept
{
  return Versions::NULL_VERSION == config;
}

constexpr bool kf::openLoop(const Versions config) noexcept
{
  return Versions::OPEN_LOOP == (config bitand Versions::OPEN_LOOP);
}

constexpr bool kf::modelReplacement(const Versions config) noexcept
{
  return Versions::MODEL_REPLACEMENT == (config bitand Versions::MODEL_REPLACEMENT);
}

constexpr bool kf::runningErrorBudget(const Versions config) noexcept
{
  return Versions::RUNNING_ERROR_BUDGET == (config bitand Versions::RUNNING_ERROR_BUDGET);
}

constexpr bool kf::applyStateBounds(const Versions config) noexcept
{
  return Versions::APPLY_STATE_BOUNDS == (config bitand Versions::APPLY_STATE_BOUNDS);
}

constexpr bool kf::eulerIntegration(const Versions config) noexcept
{
  return Versions::EULER_INTEGRATION == (config bitand Versions::INTEGRATOR_BIT_FLAG);
}

constexpr bool kf::rk4Integration(const Versions config) noexcept
{
  return Versions::RK4_INTEGRATION == (config bitand Versions::INTEGRATOR_BIT_FLAG);
}

constexpr bool kf::ode45Integration(const Versions config) noexcept
{
  return Versions::ODE45_INTEGRATION == (config bitand Versions::INTEGRATOR_BIT_FLAG);
}

#endif
/* versions.hpp */
