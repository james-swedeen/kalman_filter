/**
 * @File: multi_noise.hpp
 * @Date: April 2023
 * @Author: James Swedeen
 *
 * @brief
 * A helper class that wraps other noise objects and uses the sub-noise objects to define the output noise
 * characteristics.
 **/

#ifndef KALMAN_FILTER_NOISE_MULTI_NOISE_HPP
#define KALMAN_FILTER_NOISE_MULTI_NOISE_HPP

/* C++ Headers */
#include<memory>
#include<vector>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<kalman_filter/noise/noise_base.hpp>

namespace kf
{
namespace noise
{
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
class MultiNoise;

template<Eigen::Index DIM, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using MultiNoisePtr = std::shared_ptr<MultiNoise<DIM,SCALAR,OPTIONS>>;

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
class MultiNoise
 : public NoiseBase<DIM,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  MultiNoise() = delete;
  /**
   * @Copy Constructor
   **/
  MultiNoise(const MultiNoise&) = default;
  /**
   * @Move Constructor
   **/
  MultiNoise(MultiNoise&&) = default;
  /**
   * @Constructor
   *
   * @brief
   * Initializes the object for use.
   *
   * @parameters
   * one_dim_subnoise: A vector of pairs of one dimensional noise sources and the index in the output noise that the
   *                   sub-noise source correlates to.
   * two_dim_subnoise: A vector of pairs of two dimensional noise sources and the first index in the output noise
   *                   that the sub-noise source correlates to.
   * three_dim_subnoise: A vector of pairs of three dimensional noise sources and the first index in the output noise
   *                     that the sub-noise source correlates to.
   **/
  MultiNoise(const std::vector<std::pair<NoiseBasePtr<1,SCALAR,OPTIONS>,Eigen::Index>>& one_dim_subnoise,
             const std::vector<std::pair<NoiseBasePtr<2,SCALAR,OPTIONS>,Eigen::Index>>& two_dim_subnoise,
             const std::vector<std::pair<NoiseBasePtr<3,SCALAR,OPTIONS>,Eigen::Index>>& three_dim_subnoise);
  /**
   * @Deconstructor
   **/
  ~MultiNoise() override = default;
  /**
   * @Assignment Operators
   **/
  MultiNoise& operator=(const MultiNoise&)  = default;
  MultiNoise& operator=(      MultiNoise&&) = default;
  /**
   * @getNoise
   *
   * @brief
   * Used to get the vector of discreet noise.
   *
   * @return
   * The vector of noise values.
   **/
  inline Eigen::Matrix<SCALAR,1,DIM,OPTIONS> getNoise() override;
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
  inline Eigen::Matrix<SCALAR,1,DIM,OPTIONS> getContinuousNoise(const SCALAR time_step) override;
  /**
   * @getMean
   *
   * @brief
   * Gets the mean of the random process.
   *
   * @return
   * The mean of the random process.
   **/
  inline Eigen::Matrix<SCALAR,1,DIM,OPTIONS> getMean() override;
  /**
   * @getCovariance
   *
   * @brief
   * Gets the covariance matrix of the random process.
   *
   * @return
   * The covariance matrix of the random process.
   **/
  inline Eigen::Matrix<SCALAR,DIM,DIM,OPTIONS> getCovariance() override;
private:
  std::vector<std::pair<NoiseBasePtr<1,SCALAR,OPTIONS>,Eigen::Index>> one_dim_subnoise;
  std::vector<std::pair<NoiseBasePtr<2,SCALAR,OPTIONS>,Eigen::Index>> two_dim_subnoise;
  std::vector<std::pair<NoiseBasePtr<3,SCALAR,OPTIONS>,Eigen::Index>> three_dim_subnoise;
};

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
MultiNoise<DIM,SCALAR,OPTIONS>::
  MultiNoise(const std::vector<std::pair<NoiseBasePtr<1,SCALAR,OPTIONS>,Eigen::Index>>& one_dim_subnoise,
             const std::vector<std::pair<NoiseBasePtr<2,SCALAR,OPTIONS>,Eigen::Index>>& two_dim_subnoise,
             const std::vector<std::pair<NoiseBasePtr<3,SCALAR,OPTIONS>,Eigen::Index>>& three_dim_subnoise)
 : NoiseBase<DIM,SCALAR,OPTIONS>(),
   one_dim_subnoise(one_dim_subnoise),
   two_dim_subnoise(two_dim_subnoise),
   three_dim_subnoise(three_dim_subnoise)
{
  assert(DIM == this->one_dim_subnoise.size() + (2*this->two_dim_subnoise.size()) + (3*this->three_dim_subnoise.size()));
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM,OPTIONS> MultiNoise<DIM,SCALAR,OPTIONS>::getNoise()
{
  Eigen::Matrix<SCALAR,1,DIM,OPTIONS> output;

  const size_t num_one_dim_subnoise = this->one_dim_subnoise.size();
  for(size_t noise_it = 0; noise_it < num_one_dim_subnoise; ++noise_it)
  {
    output.template middleCols<1>(this->one_dim_subnoise[noise_it].second) =
      this->one_dim_subnoise[noise_it].first->getNoise();
  }
  const size_t num_two_dim_subnoise = this->two_dim_subnoise.size();
  for(size_t noise_it = 0; noise_it < num_two_dim_subnoise; ++noise_it)
  {
    output.template middleCols<2>(this->two_dim_subnoise[noise_it].second) =
      this->two_dim_subnoise[noise_it].first->getNoise();
  }
  const size_t num_three_dim_subnoise = this->three_dim_subnoise.size();
  for(size_t noise_it = 0; noise_it < num_three_dim_subnoise; ++noise_it)
  {
    output.template middleCols<3>(this->three_dim_subnoise[noise_it].second) =
      this->three_dim_subnoise[noise_it].first->getNoise();
  }

  return output;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM,OPTIONS> MultiNoise<DIM,SCALAR,OPTIONS>::getContinuousNoise(const SCALAR time_step)
{
  Eigen::Matrix<SCALAR,1,DIM,OPTIONS> output;

  const size_t num_one_dim_subnoise = this->one_dim_subnoise.size();
  for(size_t noise_it = 0; noise_it < num_one_dim_subnoise; ++noise_it)
  {
    output.template middleCols<1>(this->one_dim_subnoise[noise_it].second) =
      this->one_dim_subnoise[noise_it].first->getContinuousNoise(time_step);
  }
  const size_t num_two_dim_subnoise = this->two_dim_subnoise.size();
  for(size_t noise_it = 0; noise_it < num_two_dim_subnoise; ++noise_it)
  {
    output.template middleCols<2>(this->two_dim_subnoise[noise_it].second) =
      this->two_dim_subnoise[noise_it].first->getContinuousNoise(time_step);
  }
  const size_t num_three_dim_subnoise = this->three_dim_subnoise.size();
  for(size_t noise_it = 0; noise_it < num_three_dim_subnoise; ++noise_it)
  {
    output.template middleCols<3>(this->three_dim_subnoise[noise_it].second) =
      this->three_dim_subnoise[noise_it].first->getContinuousNoise(time_step);
  }

  return output;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM,OPTIONS> MultiNoise<DIM,SCALAR,OPTIONS>::getMean()
{
  Eigen::Matrix<SCALAR,1,DIM,OPTIONS> output;

  const size_t num_one_dim_subnoise = this->one_dim_subnoise.size();
  for(size_t noise_it = 0; noise_it < num_one_dim_subnoise; ++noise_it)
  {
    output.template middleCols<1>(this->one_dim_subnoise[noise_it].second) =
      this->one_dim_subnoise[noise_it].first->getMean();
  }
  const size_t num_two_dim_subnoise = this->two_dim_subnoise.size();
  for(size_t noise_it = 0; noise_it < num_two_dim_subnoise; ++noise_it)
  {
    output.template middleCols<2>(this->two_dim_subnoise[noise_it].second) =
      this->two_dim_subnoise[noise_it].first->getMean();
  }
  const size_t num_three_dim_subnoise = this->three_dim_subnoise.size();
  for(size_t noise_it = 0; noise_it < num_three_dim_subnoise; ++noise_it)
  {
    output.template middleCols<3>(this->three_dim_subnoise[noise_it].second) =
      this->three_dim_subnoise[noise_it].first->getMean();
  }

  return output;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,DIM,DIM,OPTIONS> MultiNoise<DIM,SCALAR,OPTIONS>::getCovariance()
{
  Eigen::Matrix<SCALAR,DIM,DIM,OPTIONS> output;
  output.setZero();

  const size_t num_one_dim_subnoise = this->one_dim_subnoise.size();
  for(size_t noise_it = 0; noise_it < num_one_dim_subnoise; ++noise_it)
  {
    output.template block<1,1>(this->one_dim_subnoise[noise_it].second, this->one_dim_subnoise[noise_it].second) =
      this->one_dim_subnoise[noise_it].first->getCovariance();
  }
  const size_t num_two_dim_subnoise = this->two_dim_subnoise.size();
  for(size_t noise_it = 0; noise_it < num_two_dim_subnoise; ++noise_it)
  {
    output.template block<2,2>(this->two_dim_subnoise[noise_it].second, this->two_dim_subnoise[noise_it].second) =
      this->two_dim_subnoise[noise_it].first->getCovariance();
  }
  const size_t num_three_dim_subnoise = this->three_dim_subnoise.size();
  for(size_t noise_it = 0; noise_it < num_three_dim_subnoise; ++noise_it)
  {
    output.template block<3,3>(this->three_dim_subnoise[noise_it].second, this->three_dim_subnoise[noise_it].second) =
      this->three_dim_subnoise[noise_it].first->getCovariance();
  }

  return output;
}
} // namespace noise
} // namespace kf

#endif
/* multi_noise.hpp */
