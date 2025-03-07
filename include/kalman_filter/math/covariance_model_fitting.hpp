/**
 * @File: covariance_model_fitting.hpp
 * @Date: May 2023
 * @Author: James Swedeen
 *
 * @brief
 * Provides functions that are used to fit particular models to covariance propagation data.
 **/

#ifndef KALMAN_FILTER_MATH_COVARIANCE_MODEL_FITTING_HPP
#define KALMAN_FILTER_MATH_COVARIANCE_MODEL_FITTING_HPP

/* C++ Headers */
#include<execution>

/* Boost Headers */
#include<boost/range/irange.hpp>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */

namespace kf
{
namespace math
{
namespace fit
{
/**
 * @firstOrderGaussMarkovLeastSquares
 *
 * @brief
 * Uses the non-linear least squares algorithm to find first order Gauss Markov parameters that fit the provided data.
 *
 * @templates
 * TIME_DERIVED: The matrix type of the time vector
 * VAR_DERIVED: The matrix type of the variance vector
 *
 * @parameters
 * time_data: The vector of time points that correlates to var_data
 * var_data: The variance of the state that this function is fitting the model to over time
 * init_guess: An initial guess at what the time constant and variance of the FOGM process is
 * max_error_decay_rate: The squared summed prediction residual decay rate that must be achieved before the solution is
 *                       considered actuate enough
 * max_iteration: The maximum number of iterations to perform (usually won't use all of them)
 *
 * @return
 * A pair where the first element is the time constant of the FOGM process and the second parameter is the variance of
 * the driving white process noise of the FOGM process.
 **/
template<typename TIME_DERIVED, typename VAR_DERIVED>
inline std::pair<typename TIME_DERIVED::Scalar,typename TIME_DERIVED::Scalar>
  firstOrderGaussMarkovLeastSquares(const Eigen::MatrixBase<TIME_DERIVED>&                                        time_data,
                                    const Eigen::MatrixBase<VAR_DERIVED>&                                         var_data,
                                    const std::pair<typename TIME_DERIVED::Scalar,typename TIME_DERIVED::Scalar>& init_guess,
                                    const typename TIME_DERIVED::Scalar                                           max_error_decay_rate = 1e-8,
                                    const size_t                                                                  max_iteration        = 100);
/**
 * @firstOrderGaussMarkovPropagation
 *
 * @brief
 * Uses the provided first order Gauss Markov parameters and initial variance to propagate the covariance using
 * exact integration.
 *
 * @templates
 * TIME_DERIVED: The matrix type of the time vector
 * VAR_DERIVED: The matrix type of the variance vector
 *
 * @parameters
 * time_data: The vector of time points that the variances will be calculated at
 * var_data: The variance of the state that this function is fitting the model to over time,
 *           the initial variance must be filled initially but all other values will be calculated by this function.
 *           Note: This variable is declared const so that the compiler will allow it, but the constness is casted away
 *           internally.
 * fogm_params: A pair where the first element is the time constant of the FOGM process and the second parameter
 *              is the variance of the driving white process noise of the FOGM process
 **/
template<typename TIME_DERIVED, typename VAR_DERIVED>
inline void firstOrderGaussMarkovPropagation(
              const Eigen::MatrixBase<TIME_DERIVED>&                                        time_data,
              const Eigen::MatrixBase<VAR_DERIVED>&                                         var_data,
              const std::pair<typename TIME_DERIVED::Scalar,typename TIME_DERIVED::Scalar>& fogm_params);
} // namespace fit

template<typename TIME_DERIVED, typename VAR_DERIVED>
inline std::pair<typename TIME_DERIVED::Scalar,typename TIME_DERIVED::Scalar> fit::
  firstOrderGaussMarkovLeastSquares(const Eigen::MatrixBase<TIME_DERIVED>&                                        time_data,
                                    const Eigen::MatrixBase<VAR_DERIVED>&                                         var_data,
                                    const std::pair<typename TIME_DERIVED::Scalar,typename TIME_DERIVED::Scalar>& init_guess,
                                    const typename TIME_DERIVED::Scalar                                           max_error_decay_rate,
                                    const size_t                                                                  max_iteration)
{
  assert((1 == time_data.rows()) or (1 == time_data.cols()));
  assert(((time_data.cols() == var_data.cols()) and (time_data.rows() == var_data.rows())) or
         ((time_data.cols() == var_data.rows()) and (time_data.cols() == var_data.rows())));

  typedef typename TIME_DERIVED::Scalar SCALAR;

  const Eigen::Index                                     length = std::max<Eigen::Index>(time_data.cols(), time_data.rows());
  Eigen::Matrix<SCALAR,1,2,Eigen::RowMajor>              cur_guess({init_guess.first, init_guess.second});
  Eigen::Matrix<SCALAR,1,Eigen::Dynamic,Eigen::RowMajor> residual(1, length-1);
  Eigen::Matrix<SCALAR,2,Eigen::Dynamic,Eigen::RowMajor> H(       2, length-1);
  SCALAR                                                 prev_sum_squared_residual = std::numeric_limits<SCALAR>::infinity();

  for(size_t it_count = 0; it_count < max_iteration; ++it_count)
  {
    const SCALAR time_const_over_two = cur_guess[0]/SCALAR(2);
    const SCALAR inv_time_const      = SCALAR(1)/cur_guess[0];

    for(Eigen::Index time_it = 1; time_it < length; ++time_it)
    {
      const SCALAR time_diff                              = time_data[time_it] - time_data[0];
      const SCALAR exp_term                               = std::exp(-SCALAR(2)*inv_time_const*time_diff);
      const SCALAR one_minus_exp_term                     = SCALAR(1) - exp_term;
      const SCALAR time_const_over_two_one_minus_exp_term = time_const_over_two*one_minus_exp_term;
      const SCALAR time_diff_over_time_const_exp_term     = (time_diff/cur_guess[0])*exp_term;

      // Calculate measurement residual
      residual[time_it-1] = var_data[time_it] -
                            ((exp_term*var_data[0]) + (time_const_over_two_one_minus_exp_term*cur_guess[1]));
      // Calculate H
      H(0, time_it-1) = ((SCALAR(2)/cur_guess[0])*time_diff_over_time_const_exp_term*var_data[0]) +
                        ((-time_diff_over_time_const_exp_term + (one_minus_exp_term/SCALAR(2)))*cur_guess[1]);
      H(1, time_it-1) = time_const_over_two_one_minus_exp_term;
    }
    // Preform update
    cur_guess += (H.transpose().colPivHouseholderQr().solve(residual.transpose())).transpose();
    // Check if finished
    const SCALAR cur_sum_squared_residual = residual.array().square().sum();
    if(std::abs(cur_sum_squared_residual-prev_sum_squared_residual) < (max_error_decay_rate*cur_sum_squared_residual))
    {
      break;
    }
    prev_sum_squared_residual = cur_sum_squared_residual;
  }

  return std::make_pair(cur_guess[0], cur_guess[1]);
}

template<typename TIME_DERIVED, typename VAR_DERIVED>
inline void fit::firstOrderGaussMarkovPropagation(
              const Eigen::MatrixBase<TIME_DERIVED>&                                        time_data,
              const Eigen::MatrixBase<VAR_DERIVED>&                                         var_data,
              const std::pair<typename TIME_DERIVED::Scalar,typename TIME_DERIVED::Scalar>& fogm_params)
{
  assert((1 == time_data.rows()) or (1 == time_data.cols()));
  assert(((time_data.cols() == var_data.cols()) and (time_data.rows() == var_data.rows())) or
         ((time_data.cols() == var_data.rows()) and (time_data.cols() == var_data.rows())));

  typedef typename TIME_DERIVED::Scalar SCALAR;

  const Eigen::Index                           length = std::max<Eigen::Index>(time_data.cols(), time_data.rows());
  const SCALAR       time_const_over_two_var = (fogm_params.first/SCALAR(2))*fogm_params.second;
  const SCALAR       inv_time_const          = SCALAR(1)/fogm_params.first;

  const auto exact_prop = [&time_data, &var_data, &fogm_params, time_const_over_two_var, inv_time_const]
                          (const Eigen::Index index) -> void
  {
    const SCALAR time_diff = time_data[index] - time_data[0];
    const SCALAR exp_term  = std::exp(-SCALAR(2)*inv_time_const*time_diff);

    const_cast<Eigen::MatrixBase<VAR_DERIVED>&>(var_data)[index] = (exp_term*var_data[0]) + (time_const_over_two_var*(SCALAR(1) - exp_term));
  };

  boost::integer_range<Eigen::Index> data_inds(1, length);

  std::for_each(std::execution::par_unseq, data_inds.begin(), data_inds.end(), exact_prop);
}
} // namespace math
} // namespace kf

#endif
/* covariance_model_fitting.hpp */
