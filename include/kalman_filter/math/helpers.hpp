/**
 * @File: helpers.hpp
 * @Date: April 2022
 * @Author: James Swedeen
 *
 * @brief
 * Defines general helper functions for math.
 **/

#ifndef KALMAN_FILTER_MATH_HELPERS_HPP
#define KALMAN_FILTER_MATH_HELPERS_HPP

/* Eigen Headers */
#include<Eigen/Dense>

namespace kf
{
namespace math
{
/**
 * @Constants
 *
 * @brief
 * Some constants that are used frequently.
 *
 * @templates
 * SCALAR: The floating point type
 **/
template<typename SCALAR = double>
constexpr SCALAR pi() noexcept;

template<typename SCALAR = double>
constexpr SCALAR twoPi() noexcept;

template<typename SCALAR = double>
constexpr SCALAR oneHalfPi() noexcept;
/**
 * @dotProduct
 *
 * @brief
 * Vector wise dot product.
 *
 * @templates
 * DERIVED1: The matrix type of the first argument
 * DERIVED2: The matrix type of the second argument
 *
 * @parameters
 * vec_1: The first vector to dot, dynamic by N columns
 * vec_2: The second vector to dot, dynamic by N columns
 *
 * @return
 * The dot product.
 **/
template<typename DERIVED1, typename DERIVED2>
inline Eigen::Matrix<typename DERIVED1::Scalar,1,int(DERIVED1::RowsAtCompileTime),Eigen::RowMajor>
  dotProduct(const Eigen::MatrixBase<DERIVED1>& vec_1, const Eigen::MatrixBase<DERIVED2>& vec_2) noexcept;
/**
 * @crossProduct
 *
 * @brief
 * Vector wise cross product.
 *
 * @templates
 * DERIVED1: The matrix type of the first argument
 * DERIVED2: The matrix type of the second argument
 *
 * @parameters
 * vec_1: The first vector to dot, dynamic by 3 columns
 * vec_2: The second vector to dot, dynamic by 3 columns
 *
 * @return
 * The cross product.
 **/
template<typename DERIVED1, typename DERIVED2>
inline typename DERIVED1::PlainMatrix
  crossProduct(const Eigen::MatrixBase<DERIVED1>& vec_1, const Eigen::MatrixBase<DERIVED2>& vec_2) noexcept;
/**
 * @crossProductMatrix
 *
 * @brief
 * Makes the matrix that produces the cross product when doted with another vector.
 *
 * @templates
 * DERIVED: The matrix type of the argument
 *
 * @templates
 * SCALAR: Either a double or a float.
 * OPTIONS: Eigen matrix options.
 *
 * @parameters
 * vec: The vector to make the matrix from, 1 by 3 columns
 *
 * @return
 * The cross product matrix.
 **/
template<typename DERIVED>
inline Eigen::Matrix<typename DERIVED::Scalar,3,3,Eigen::RowMajor>
  crossProductMatrix(const Eigen::MatrixBase<DERIVED>& vec) noexcept;
/**
 * @angleDiff
 *
 * @brief
 * Finds the angular displacement from the first angle to the second.
 *
 * @templates
 * SCALAR: The floating point type
 *
 * @parameters
 * angle_one: The starting angle
 * angle_two: The target angle
 *
 * @return
 * The displacement between the two angles.
 **/
template<typename SCALAR>
inline SCALAR angleDiff(const SCALAR angle_one, const SCALAR angle_two) noexcept;
} // namespace math

template<typename SCALAR>
constexpr SCALAR math::pi() noexcept
{
  return std::atan(SCALAR(1))*SCALAR(4);
}

template<typename SCALAR>
constexpr SCALAR math::twoPi() noexcept
{
  return SCALAR(2.0) * math::pi<SCALAR>();
}

template<typename SCALAR>
constexpr SCALAR math::oneHalfPi() noexcept
{
  return math::pi<SCALAR>() / SCALAR(2.0);
}

template<typename DERIVED1, typename DERIVED2>
inline Eigen::Matrix<typename DERIVED1::Scalar,1,int(DERIVED1::RowsAtCompileTime),Eigen::RowMajor>
  math::dotProduct(const Eigen::MatrixBase<DERIVED1>& vec_1, const Eigen::MatrixBase<DERIVED2>& vec_2) noexcept
{
  static_assert(int(DERIVED1::ColsAtCompileTime) == int(DERIVED2::ColsAtCompileTime));
  static_assert(int(DERIVED1::RowsAtCompileTime) == int(DERIVED2::RowsAtCompileTime));

  return (vec_1.array() * vec_2.array()).rowwise().sum();
}

template<typename DERIVED1, typename DERIVED2>
inline typename DERIVED1::PlainMatrix
  math::crossProduct(const Eigen::MatrixBase<DERIVED1>& vec_1, const Eigen::MatrixBase<DERIVED2>& vec_2) noexcept
{
  typename DERIVED1::PlainMatrix output(vec_1.rows(), 3);

  output.template leftCols<1>()    = (vec_1.template middleCols<1>(1).array() * vec_2.template rightCols<1>().  array()) -
                                     (vec_1.template rightCols<1>().  array() * vec_2.template middleCols<1>(1).array());
  output.template middleCols<1>(1) = (vec_1.template rightCols<1>().  array() * vec_2.template leftCols<1>().   array()) -
                                     (vec_1.template leftCols<1>().   array() * vec_2.template rightCols<1>().  array());
  output.template rightCols<1>()   = (vec_1.template leftCols<1>().   array() * vec_2.template middleCols<1>(1).array()) -
                                     (vec_1.template middleCols<1>(1).array() * vec_2.template leftCols<1>().   array());

  return output;
}

template<typename DERIVED>
inline Eigen::Matrix<typename DERIVED::Scalar,3,3,Eigen::RowMajor>
  math::crossProductMatrix(const Eigen::MatrixBase<DERIVED>& vec) noexcept
{
  Eigen::Matrix<typename DERIVED::Scalar,3,3,Eigen::RowMajor> output;
  output.setZero();

  output(0, 1) = -vec[2];
  output(1, 0) =  vec[2];
  output(0, 2) =  vec[1];
  output(2, 0) = -vec[1];
  output(1, 2) = -vec[0];
  output(2, 1) =  vec[0];

  return output;
}

template<typename SCALAR>
inline SCALAR math::angleDiff(const SCALAR angle_one, const SCALAR angle_two) noexcept
{
  return pi<SCALAR>() - std::fabs(std::fmod(std::fabs(angle_one - angle_two), twoPi<SCALAR>()) - pi<SCALAR>());
}
} // namespace kf

#endif
/* helpers.hpp */
