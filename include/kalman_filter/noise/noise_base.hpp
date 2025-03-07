/**
 * @File: noise_base.hpp
 * @Date: April 2022
 * @Author: James Swedeen
 *
 * @brief
 * A base class for noise generators.
 **/

#ifndef KALMAN_FILTER_NOISE_NOISE_BASE_HPP
#define KALMAN_FILTER_NOISE_NOISE_BASE_HPP

/* C++ Headers */
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

namespace kf
{
namespace noise
{
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
class NoiseBase;

template<Eigen::Index DIM, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using NoiseBasePtr = std::shared_ptr<NoiseBase<DIM,SCALAR,OPTIONS>>;

/**
 * @DIM
 * The number of dimensions each point will have.
 *
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options.
 **/
template<Eigen::Index DIM, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class NoiseBase
{
public:
  /**
   * @Default Constructor
   **/
  NoiseBase() noexcept = default;
  /**
   * @Copy Constructor
   **/
  NoiseBase(const NoiseBase&) noexcept = default;
  /**
   * @Move Constructor
   **/
  NoiseBase(NoiseBase&&) noexcept = default;
  /**
   * @Deconstructor
   **/
  virtual ~NoiseBase() noexcept = default;
  /**
   * @Assignment Operators
   **/
  NoiseBase& operator=(const NoiseBase&)  noexcept = default;
  NoiseBase& operator=(      NoiseBase&&) noexcept = default;
  /**
   * @getNoise
   *
   * @brief
   * Used to get the vector of discreet noise.
   *
   * @return
   * The vector of noise values.
   **/
  inline virtual Eigen::Matrix<SCALAR,1,DIM,OPTIONS> getNoise() = 0;
  /**
   * @getContinuousNoise
   *
   * @brief
   * Used to get the vector of noise that approximates continuous noise.
   *
   * @parameters
   * time_step: The time step between the last state vector and the current one
   *
   * @return
   * The vector of noise values.
   **/
  inline virtual Eigen::Matrix<SCALAR,1,DIM,OPTIONS> getContinuousNoise(const SCALAR time_step) = 0;
  /**
   * @getMean
   *
   * @brief
   * Gets the mean of the random process.
   *
   * @return
   * The mean of the random process.
   **/
  inline virtual Eigen::Matrix<SCALAR,1,DIM,OPTIONS> getMean() = 0;
  /**
   * @getCovariance
   *
   * @brief
   * Gets the covariance matrix of the random process.
   *
   * @return
   * The covariance matrix of the random process.
   **/
  inline virtual Eigen::Matrix<SCALAR,DIM,DIM,OPTIONS> getCovariance() = 0;
};
} // namespace noise
} // namespace kf

#endif
/* noise_base.hpp */
