/**
 * @File: integrator_step.hpp
 * @Date: March 2023
 * @Author: James Swedeen
 *
 * @brief
 * Defines a function that implements one integration step.
 **/

#ifndef KALMAN_FILTER_HELPERS_INTEGRATOR_STEP_HPP
#define KALMAN_FILTER_HELPERS_INTEGRATOR_STEP_HPP

/* C++ Headers */
#include<functional>

/* Boost Headers */
#include<boost/numeric/odeint.hpp>
#include<boost/numeric/odeint/external/eigen/eigen_algebra.hpp>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<kalman_filter/helpers/versions.hpp>

namespace kf
{
/**
 * @integratorStep
 *
 * @brief
 * Used to perform one step of integration.
 *
 * @templates
 * VERSION: Controls what type of simulation will be ran
 * SCALAR: The object type that each dimension will be represented with
 * FUNC_DERIVED: The matrix type that the derivative function uses
 * INPUT_DERIVED: The matrix type that this function works with
 *
 * @parameters
 * derivative_func: The derivative of the value we are integrating
 * prev_x: The old value
 * dt: The step over which we are integrating
 *
 * @return
 * The integrated state vector.
 **/
template<Versions VERSION, typename SCALAR, typename FUNC_DERIVED, typename INPUT_DERIVED>
inline typename INPUT_DERIVED::PlainMatrix
  integratorStep(const std::function<typename FUNC_DERIVED::PlainMatrix(const Eigen::Ref<const FUNC_DERIVED>&)>& derivative_func,
                 const Eigen::MatrixBase<INPUT_DERIVED>&                                                         prev_x,
                 const SCALAR                                                                                    dt);
} // namespace kf

template<kf::Versions VERSION, typename SCALAR, typename FUNC_DERIVED, typename INPUT_DERIVED>
inline typename INPUT_DERIVED::PlainMatrix
  kf::integratorStep(const std::function<typename FUNC_DERIVED::PlainMatrix(const Eigen::Ref<const FUNC_DERIVED>&)>& derivative_func,
                     const Eigen::MatrixBase<INPUT_DERIVED>&                                                         prev_x,
                     const SCALAR                                                                                    dt)
{
  if constexpr(kf::eulerIntegration(VERSION))
  {
    return prev_x + (dt * derivative_func(prev_x).array()).matrix();
  }
  else if constexpr(kf::rk4Integration(VERSION))
  {
    const SCALAR half_dt = dt / SCALAR(2);

    const typename INPUT_DERIVED::PlainMatrix k1 = derivative_func(prev_x);
    const typename INPUT_DERIVED::PlainMatrix k2 = derivative_func(prev_x + (half_dt * k1.array()).matrix());
    const typename INPUT_DERIVED::PlainMatrix k3 = derivative_func(prev_x + (half_dt * k2.array()).matrix());
    const typename INPUT_DERIVED::PlainMatrix k4 = derivative_func(prev_x + (dt      * k3.array()).matrix());

    return prev_x + ((dt/SCALAR(6)) * (k1 + (SCALAR(2)*k2.array()).matrix() + (SCALAR(2)*k3.array()).matrix() + k4).array()).matrix();
  }
  else if constexpr(kf::ode45Integration(VERSION))
  {
    typedef Eigen::Matrix<SCALAR,1,INPUT_DERIVED::SizeAtCompileTime,Eigen::RowMajor> VEC_STATE_TYPE;
    typedef boost::numeric::odeint::runge_kutta_dopri5<VEC_STATE_TYPE,SCALAR,VEC_STATE_TYPE,SCALAR,boost::numeric::odeint::vector_space_algebra> STEPPER_TYPE;

    typename INPUT_DERIVED::PlainMatrix output = prev_x;
    VEC_STATE_TYPE                      vec_output = Eigen::Reshaped<typename INPUT_DERIVED::PlainMatrix,1,INPUT_DERIVED::SizeAtCompileTime,Eigen::RowMajor>(output);

    boost::numeric::odeint::integrate_adaptive(
      boost::numeric::odeint::make_dense_output<STEPPER_TYPE>(1.0e-12, 1.0e-8),
      //boost::numeric::odeint::make_controlled<STEPPER_TYPE>(1.0e-12, 1.0e-8),
      //boost::numeric::odeint::make_controlled<STEPPER_TYPE>(1.0e-6, 1.0e-3),
      //STEPPER_TYPE(),
      [&derivative_func] (const VEC_STATE_TYPE& x, VEC_STATE_TYPE& dxdt, const SCALAR /* t */) -> void
        {
          typename INPUT_DERIVED::PlainMatrix nvec_dxdt =
            derivative_func(Eigen::Reshaped<const VEC_STATE_TYPE,INPUT_DERIVED::RowsAtCompileTime,INPUT_DERIVED::ColsAtCompileTime,Eigen::RowMajor>(x));

          dxdt = Eigen::Reshaped<typename INPUT_DERIVED::PlainMatrix,1,INPUT_DERIVED::SizeAtCompileTime,Eigen::RowMajor>(nvec_dxdt);
        },
      vec_output,
      SCALAR(0),
      dt,
      dt);

    return Eigen::Reshaped<const VEC_STATE_TYPE,INPUT_DERIVED::RowsAtCompileTime,INPUT_DERIVED::ColsAtCompileTime,Eigen::RowMajor>(vec_output);
  }
  else
  {
    assert(false);
    return INPUT_DERIVED::PlainMatrix::Zero();
  }
}

#endif
/* rk4_step.hpp */
