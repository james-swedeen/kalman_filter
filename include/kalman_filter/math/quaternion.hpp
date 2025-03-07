/**
 * @File: quaternion.hpp
 * @Date: April 2022
 * @Author: James Swedeen
 *
 * @brief
 * Some helper functions for working with Hamiltonian quaternions.
 **/

#ifndef KALMAN_FILTER_MATH_QUATERNION_HPP
#define KALMAN_FILTER_MATH_QUATERNION_HPP

/* C++ Headers */
#include<cstdint>
#include<cmath>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<kalman_filter/math/helpers.hpp>

namespace kf
{
namespace math
{
namespace quat
{
/**
 * @IND
 *
 * @brief
 * Provides the index of each quaternion element.
 **/
namespace IND
{
  inline constexpr static const Eigen::Index W = 0; // The scalier component of the quaternion
  inline constexpr static const Eigen::Index X = 1; // The first element of the vector component of the quaternion
  inline constexpr static const Eigen::Index Y = 2; // The second element of the vector component of the quaternion
  inline constexpr static const Eigen::Index Z = 3; // The third element of the vector component of the quaternion
}
/**
 * @conjugate
 *
 * @brief
 * Calculates the conjugate of the quaternion.
 *
 * @templates
 * DERIVED: The matrix type of the quaternion
 *
 * @parameters
 * quat: The quaternion to conjugate
 *
 * @return
 * The conjugate of the quaternion.
 **/
template<typename DERIVED>
inline typename DERIVED::PlainMatrix conjugate(const Eigen::MatrixBase<DERIVED>& quat) noexcept;
/**
 * @conjugateInPlace
 *
 * @brief
 * Calculates the conjugate of the quaternion.
 *
 * @templates
 * DERIVED: The matrix type of the quaternion
 *
 * @parameters
 * quat: The quaternion to conjugate and the output variable
 **/
template<typename DERIVED>
inline void conjugateInPlace(Eigen::MatrixBase<DERIVED>& quat) noexcept;
/**
 * @normalize
 *
 * @brief
 * Normalizes the quaternion.
 *
 * @templates
 * DERIVED: The matrix type of the quaternion
 *
 * @parameters
 * quat: The quaternion to normalize
 *
 * @return
 * The normalized quaternion.
 **/
template<typename DERIVED>
inline typename DERIVED::PlainMatrix normalize(const Eigen::MatrixBase<DERIVED>& quat) noexcept;
/**
 * @normalizeInPlace
 *
 * @brief
 * Normalizes the quaternion in place.
 *
 * @templates
 * DERIVED: The matrix type of the quaternion
 *
 * @parameters
 * quat: The quaternion to normalize
 **/
template<typename DERIVED>
inline void normalizeInPlace(Eigen::MatrixBase<DERIVED>& quat) noexcept;
/**
 * @product
 *
 * @brief
 * Performs the quaternion product on the two provided quaternions.
 *
 * @templates
 * DERIVED1: The matrix type of the first quaternion
 * DERIVED2: The matrix type of the second quaternion
 *
 * @parameters
 * quat_1: The first quaternion to multiply
 * quat_2: The second quaternion to multiply
 *
 * @return
 * The product of the two quaternions.
 **/
template<typename DERIVED1, typename DERIVED2>
inline typename DERIVED1::PlainMatrix product(const Eigen::MatrixBase<DERIVED1>& quat_1,
                                              const Eigen::MatrixBase<DERIVED2>& quat_2) noexcept;
/**
 * @rollPitchYawToQuaternion
 *
 * @brief
 * Calculates a quaternion from a given roll, pitch, and yaw.
 *
 * @templates
 * DERIVED1: The matrix type of the first argument
 * DERIVED2: The matrix type of the second argument
 * DERIVED3: The matrix type of the third argument
 *
 * @parameters
 * roll: The roll angle
 * pitch: The pitch angle
 * yaw: The yaw angle
 *
 * @return
 * The corresponding quaternion.
 **/
template<typename DERIVED1, typename DERIVED2, typename DERIVED3>
inline Eigen::Matrix<typename DERIVED1::Scalar,int(DERIVED1::ColsAtCompileTime),4,Eigen::RowMajor>
  rollPitchYawToQuaternion(const Eigen::MatrixBase<DERIVED1>& roll,
                           const Eigen::MatrixBase<DERIVED2>& pitch,
                           const Eigen::MatrixBase<DERIVED3>& yaw) noexcept;

template<typename DERIVED>
inline Eigen::Matrix<typename DERIVED::Scalar,int(DERIVED::RowsAtCompileTime),4,Eigen::RowMajor>
  rollPitchYawToQuaternion(const Eigen::MatrixBase<DERIVED>& rollPitchYaw) noexcept;
/**
 * @quaternionToRollPitchYaw
 *
 * @brief
 * Calculates the roll, pitch, and yaw associated with a particular quaternion.
 *
 * @templates
 * DERIVED: The matrix type of the quaternion
 *
 * @parameters
 * roll: The roll angle
 * pitch: The pitch angle
 * yaw: The yaw angle
 *
 * @return
 * The roll, pitch, and yaw associated with a particular quaternion in that order.
 **/
template<typename DERIVED>
inline Eigen::Matrix<typename DERIVED::Scalar,int(DERIVED::RowsAtCompileTime),3,Eigen::RowMajor>
  quaternionToRollPitchYaw(const Eigen::MatrixBase<DERIVED>& quat) noexcept;
/**
 * @quaternionToDirectionCosineMatrix
 *
 * @brief
 * Calculates the direction cosine matrix associated with a particular quaternion.
 *
 * @templates
 * DERIVED: The matrix type of the quaternion
 *
 * @parameters
 * quat: The quaternion
 *
 * @return
 * The direction cosine matrix associated with a particular quaternion in that order.
 **/
template<typename DERIVED>
inline Eigen::Matrix<typename DERIVED::Scalar,3,3,Eigen::RowMajor>
  quaternionToDirectionCosineMatrix(const Eigen::MatrixBase<DERIVED>& quat) noexcept;
/**
 * @rollPitchYawToDirectionCosineMatrix
 *
 * @brief
 * Calculates the direction cosine matrix associated with a particular roll, pitch, yaw sequence.
 *
 * @templates
 * SCALAR: The floating point type
 * OPTIONS: Eigen Matrix options
 *
 * @parameters
 * roll: The roll angle
 * pitch: The pitch angle
 * yaw: The yaw angle
 *
 * @return
 * The direction cosine matrix associated with a particular roll, pitch, yaw sequence in that order.
 **/
template<typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
inline Eigen::Matrix<SCALAR,3,3,OPTIONS>
  rollPitchYawToDirectionCosineMatrix(const SCALAR roll, const SCALAR pitch, const SCALAR yaw) noexcept;

template<typename DERIVED>
inline Eigen::Matrix<typename DERIVED::Scalar,3,3,Eigen::RowMajor>
  rollPitchYawToDirectionCosineMatrix(const Eigen::MatrixBase<DERIVED>& rollPitchYaw) noexcept;
/**
 * @averageQuaternions
 *
 * @brief
 * Averages the given quaternions.
 *
 * @templates
 * DERIVED: The matrix type of the quaternion
 *
 * @parameters
 * quats: The quaternions to average
 *
 * @return
 * The averaged quaternion.
 **/
template<typename DERIVED>
inline Eigen::Matrix<typename DERIVED::Scalar,1,4,Eigen::RowMajor>
  averageQuaternions(const Eigen::MatrixBase<DERIVED>& quats) noexcept;
} // quat


template<typename DERIVED>
inline typename DERIVED::PlainMatrix quat::conjugate(const Eigen::MatrixBase<DERIVED>& quat) noexcept
{
  static_assert((int(DERIVED::ColsAtCompileTime) == 4) or (int(DERIVED::ColsAtCompileTime) == Eigen::Dynamic));
  assert(quat.cols() == 4);

  typename DERIVED::PlainMatrix output = quat;
  output.template rightCols<3>().noalias() = -output.template rightCols<3>();
  return output;
}

template<typename DERIVED>
inline void quat::conjugateInPlace(Eigen::MatrixBase<DERIVED>& quat) noexcept
{
  static_assert((int(DERIVED::ColsAtCompileTime) == 4) or (int(DERIVED::ColsAtCompileTime) == Eigen::Dynamic));
  assert(quat.cols() == 4);

  quat.template rightCols<3>().noalias() = -quat.template rightCols<3>();
}

template<typename DERIVED>
inline typename DERIVED::PlainMatrix quat::normalize(const Eigen::MatrixBase<DERIVED>& quat) noexcept
{
  static_assert((int(DERIVED::ColsAtCompileTime) == 4) or (int(DERIVED::ColsAtCompileTime) == Eigen::Dynamic));
  assert(quat.cols() == 4);

  return quat.array() * quat.rowwise().norm().array().inverse().template replicate<1,4>();
}

template<typename DERIVED>
inline void quat::normalizeInPlace(Eigen::MatrixBase<DERIVED>& quat) noexcept
{
  static_assert((int(DERIVED::ColsAtCompileTime) == 4) or (int(DERIVED::ColsAtCompileTime) == Eigen::Dynamic));
  assert(quat.cols() == 4);

  quat.noalias() = (quat.array() * quat.rowwise().norm().array().inverse().template replicate<1,4>()).matrix();
}

template<typename DERIVED1, typename DERIVED2>
inline typename DERIVED1::PlainMatrix quat::product(const Eigen::MatrixBase<DERIVED1>& quat_1,
                                                    const Eigen::MatrixBase<DERIVED2>& quat_2) noexcept
{
  static_assert((int(DERIVED1::ColsAtCompileTime) == 4) or (int(DERIVED1::ColsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED2::ColsAtCompileTime) == 4) or (int(DERIVED2::ColsAtCompileTime) == Eigen::Dynamic));
  assert(quat_1.cols() == 4);
  assert(quat_2.cols() == 4);

  typename DERIVED1::PlainMatrix output(quat_1.rows(), 4);

  output.template leftCols<1>()  = (quat_1.template leftCols<1>().array() * quat_2.template leftCols<1>().array()) -
                                   dotProduct(quat_2.template rightCols<3>(),
                                              quat_1.template rightCols<3>()).transpose().array();
  output.template rightCols<3>() =
    (quat_1.template leftCols<1>().template replicate<1,3>().array() * quat_2.template rightCols<3>().array()) +
    (quat_2.template leftCols<1>().template replicate<1,3>().array() * quat_1.template rightCols<3>().array()) +
    crossProduct(quat_1.template rightCols<3>(), quat_2.template rightCols<3>()).array();

  return output;
}

template<typename DERIVED1, typename DERIVED2, typename DERIVED3>
inline Eigen::Matrix<typename DERIVED1::Scalar,int(DERIVED1::ColsAtCompileTime),4,Eigen::RowMajor>
  quat::rollPitchYawToQuaternion(const Eigen::MatrixBase<DERIVED1>& roll,
                                 const Eigen::MatrixBase<DERIVED2>& pitch,
                                 const Eigen::MatrixBase<DERIVED3>& yaw) noexcept
{
  static_assert((int(DERIVED1::RowsAtCompileTime) == 1) or (int(DERIVED1::RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED2::RowsAtCompileTime) == 1) or (int(DERIVED3::RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED3::RowsAtCompileTime) == 1) or (int(DERIVED3::RowsAtCompileTime) == Eigen::Dynamic));
  assert(roll. rows() == 1);
  assert(pitch.rows() == 1);
  assert(yaw.  rows() == 1);

  typedef typename DERIVED1::Scalar SCALAR;

  Eigen::Matrix<typename DERIVED1::Scalar,int(DERIVED1::ColsAtCompileTime),4,Eigen::RowMajor> output(roll.cols(), 4);

  const typename DERIVED1::PlainArray half_roll  = roll. array() * (SCALAR(1)/SCALAR(2));
  const typename DERIVED2::PlainArray half_pitch = pitch.array() * (SCALAR(1)/SCALAR(2));
  const typename DERIVED3::PlainArray half_yaw   = yaw.  array() * (SCALAR(1)/SCALAR(2));

  const typename DERIVED1::PlainArray c_roll  = half_roll.cos();
  const typename DERIVED1::PlainArray s_roll  = half_roll.sin();
  const typename DERIVED2::PlainArray c_pitch = half_pitch.cos();
  const typename DERIVED2::PlainArray s_pitch = half_pitch.sin();
  const typename DERIVED3::PlainArray c_yaw   = half_yaw.cos();
  const typename DERIVED3::PlainArray s_yaw   = half_yaw.sin();

  output.col(IND::W) = (c_roll * c_pitch * c_yaw) + (s_roll * s_pitch * s_yaw);
  output.col(IND::X) = (s_roll * c_pitch * c_yaw) - (c_roll * s_pitch * s_yaw);
  output.col(IND::Y) = (c_roll * s_pitch * c_yaw) + (s_roll * c_pitch * s_yaw);
  output.col(IND::Z) = (c_roll * c_pitch * s_yaw) - (s_roll * s_pitch * c_yaw);

  return output;
}

template<typename DERIVED>
inline Eigen::Matrix<typename DERIVED::Scalar,int(DERIVED::RowsAtCompileTime),4,Eigen::RowMajor>
  quat::rollPitchYawToQuaternion(const Eigen::MatrixBase<DERIVED>& rollPitchYaw) noexcept
{
  static_assert((int(DERIVED::ColsAtCompileTime) == 3) or (int(DERIVED::ColsAtCompileTime) == Eigen::Dynamic));
  assert(rollPitchYaw.cols() == 3);

  return rollPitchYawToQuaternion(rollPitchYaw.template leftCols<1>(),
                                  rollPitchYaw.template middleCols<1>(1),
                                  rollPitchYaw.template rightCols<1>());
}

template<typename DERIVED>
inline Eigen::Matrix<typename DERIVED::Scalar,int(DERIVED::RowsAtCompileTime),3,Eigen::RowMajor>
  quat::quaternionToRollPitchYaw(const Eigen::MatrixBase<DERIVED>& quat) noexcept
{
  static_assert((int(DERIVED::ColsAtCompileTime) == 4) or (int(DERIVED::ColsAtCompileTime) == Eigen::Dynamic));
  assert(quat.cols() == 4);

  typedef Eigen::Array<typename DERIVED::Scalar,1,int(DERIVED::RowsAtCompileTime),Eigen::RowMajor> ARRAY_TYPE;
  typedef typename DERIVED::Scalar                                                                 SCALAR;

  const Eigen::Index                                                                        output_len = quat.rows();
  Eigen::Matrix<typename DERIVED::Scalar,int(DERIVED::RowsAtCompileTime),3,Eigen::RowMajor> output(output_len, 3);

  const ARRAY_TYPE w_p2 = quat.col(IND::W).array().square();
  const ARRAY_TYPE x_p2 = quat.col(IND::X).array().square();
  const ARRAY_TYPE y_p2 = quat.col(IND::Y).array().square();
  const ARRAY_TYPE z_p2 = quat.col(IND::Z).array().square();

  const ARRAY_TYPE roll_num
    = SCALAR(2)*((quat.col(IND::W).array()*quat.col(IND::X).array()) + (quat.col(IND::Y).array()*quat.col(IND::Z).array()));
  const ARRAY_TYPE roll_den = w_p2 + z_p2 - x_p2 - y_p2;

  const ARRAY_TYPE pitch_term
    = SCALAR(2)*((quat.col(IND::W).array()*quat.col(IND::Y).array()) - (quat.col(IND::Z).array()*quat.col(IND::X).array()));

  const ARRAY_TYPE yaw_num
    = SCALAR(2)*((quat.col(IND::W).array()*quat.col(IND::Z).array()) + (quat.col(IND::X).array()*quat.col(IND::Y).array()));
  const ARRAY_TYPE yaw_den = w_p2 + x_p2 - y_p2 - z_p2;

  for(Eigen::Index row_it = 0; row_it < output_len; ++row_it)
  {
    output(row_it, 0) = std::atan2(roll_num[row_it], roll_den[row_it]);

    if(std::abs(pitch_term[row_it]) >= 1)
    {
      output(row_it, 1) = std::copysign(oneHalfPi<SCALAR>(), pitch_term[row_it]);
    }
    else
    {
      output(row_it, 1) = std::asin(pitch_term[row_it]);
    }

    output(row_it, 2) = std::atan2(yaw_num[row_it], yaw_den[row_it]);
  }

  return output;
}

template<typename DERIVED>
inline Eigen::Matrix<typename DERIVED::Scalar,3,3,Eigen::RowMajor>
  quat::quaternionToDirectionCosineMatrix(const Eigen::MatrixBase<DERIVED>& quat) noexcept
{
  static_assert((int(DERIVED::RowsAtCompileTime) == 1) or (int(DERIVED::RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED::ColsAtCompileTime) == 4) or (int(DERIVED::ColsAtCompileTime) == Eigen::Dynamic));
  assert(quat.rows() == 1);
  assert(quat.cols() == 4);

  typedef typename DERIVED::Scalar SCALAR;

  Eigen::Matrix<typename DERIVED::Scalar,3,3,Eigen::RowMajor> output;

  const SCALAR w_p2 = std::pow(quat[IND::W], 2);
  const SCALAR x_p2 = std::pow(quat[IND::X], 2);
  const SCALAR y_p2 = std::pow(quat[IND::Y], 2);
  const SCALAR z_p2 = std::pow(quat[IND::Z], 2);

  output(0,0) = w_p2 + x_p2 - y_p2 - z_p2;
  output(0,1) = SCALAR(2)*((quat[IND::X]*quat[IND::Y]) - (quat[IND::W]*quat[IND::Z]));
  output(0,2) = SCALAR(2)*((quat[IND::X]*quat[IND::Z]) + (quat[IND::W]*quat[IND::Y]));
  output(1,0) = SCALAR(2)*((quat[IND::X]*quat[IND::Y]) + (quat[IND::W]*quat[IND::Z]));
  output(1,1) = w_p2 - x_p2 + y_p2 - z_p2;
  output(1,2) = SCALAR(2)*((quat[IND::Y]*quat[IND::Z]) - (quat[IND::W]*quat[IND::X]));
  output(2,0) = SCALAR(2)*((quat[IND::X]*quat[IND::Z]) - (quat[IND::W]*quat[IND::Y]));
  output(2,1) = SCALAR(2)*((quat[IND::Y]*quat[IND::Z]) + (quat[IND::W]*quat[IND::X]));
  output(2,2) = w_p2 - x_p2 - y_p2 + z_p2;

  return output;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,3,3,OPTIONS>
  quat::rollPitchYawToDirectionCosineMatrix(const SCALAR roll, const SCALAR pitch, const SCALAR yaw) noexcept
{
  Eigen::Matrix<SCALAR,3,3,OPTIONS> output;

  const SCALAR s_roll  = std::sin(roll);
  const SCALAR c_roll  = std::cos(roll);
  const SCALAR s_pitch = std::sin(pitch);
  const SCALAR c_pitch = std::cos(pitch);
  const SCALAR s_yaw   = std::sin(yaw);
  const SCALAR c_yaw   = std::cos(yaw);

  output(0, 0) = c_pitch*c_yaw;
  output(1, 0) = c_pitch*s_yaw;
  output(2, 0) = -s_pitch;
  output(0, 1) = (s_roll*s_pitch*c_yaw) - (c_roll*s_yaw);
  output(1, 1) = (s_roll*s_pitch*s_yaw) + (c_roll*c_yaw);
  output(2, 1) = s_roll*c_pitch;
  output(0, 2) = (c_roll*s_pitch*c_yaw) + (s_roll*s_yaw);
  output(1, 2) = (c_roll*s_pitch*s_yaw) - (s_roll*c_yaw);
  output(2, 2) = c_roll*c_pitch;

  return output;
/*  return quaternionToDirectionCosineMatrix(rollPitchYawToQuaternion(Eigen::Matrix<SCALAR,1,1,OPTIONS>(roll),
                                                                    Eigen::Matrix<SCALAR,1,1,OPTIONS>(pitch),
                                                                    Eigen::Matrix<SCALAR,1,1,OPTIONS>(yaw)));*/
}

template<typename DERIVED>
inline Eigen::Matrix<typename DERIVED::Scalar,3,3,Eigen::RowMajor>
  quat::rollPitchYawToDirectionCosineMatrix(const Eigen::MatrixBase<DERIVED>& rollPitchYaw) noexcept
{
  return rollPitchYawToDirectionCosineMatrix(rollPitchYaw[0], rollPitchYaw[1], rollPitchYaw[2]);
}

template<typename DERIVED>
inline Eigen::Matrix<typename DERIVED::Scalar,1,4,Eigen::RowMajor>
  quat::averageQuaternions(const Eigen::MatrixBase<DERIVED>& quats) noexcept
{
  static_assert((int(DERIVED::ColsAtCompileTime) == 4) or (int(DERIVED::ColsAtCompileTime) == Eigen::Dynamic));
  assert(quats.cols() == 4);

  const Eigen::Index num_quats = quats.rows();

  Eigen::Matrix<typename DERIVED::Scalar,4,4,Eigen::RowMajor> accumulator_mat;
  accumulator_mat.setZero();

  for(Eigen::Index quat_it = 0; quat_it < num_quats; ++quat_it)
  {
    accumulator_mat.noalias() += quats.row(quat_it).transpose() * quats.row(quat_it);
  }
  accumulator_mat.array() /= typename DERIVED::Scalar(num_quats);

  return Eigen::SelfAdjointEigenSolver<Eigen::Matrix<typename DERIVED::Scalar,4,4,Eigen::RowMajor>>(accumulator_mat).eigenvectors().template rightCols<1>().transpose();
}
} // namespace math
} // namespace kf

#endif
/* quaternion.hpp */
