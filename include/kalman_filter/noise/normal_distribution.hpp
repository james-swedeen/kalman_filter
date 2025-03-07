/**
 * @File: normal_distribution.hpp
 * @Date: April 2022
 * @Author: James Swedeen
 *
 * @brief
 * A noise generator that pulls from the normal distribution.
 **/

#ifndef KALMAN_FILTER_NOISE_NORMAL_DISTRIBUTION_HPP
#define KALMAN_FILTER_NOISE_NORMAL_DISTRIBUTION_HPP

/* C++ Headers */
#include<memory>
#include<string>
#include<vector>
#include<chrono>
#include<cassert>
#include<random>

/* Eigen Headers */
#include<Eigen/Dense>

/* ROS Headers */
#include<rclcpp/rclcpp.hpp>

/* Local Headers */
#include<kalman_filter/noise/noise_base.hpp>

namespace kf
{
namespace noise
{
template<Eigen::Index DIM, bool ENABLED, bool ZERO_MEAN, bool UNIT_VAR, typename SCALAR, Eigen::StorageOptions OPTIONS>
class NormalDistribution;

template<Eigen::Index DIM, bool ENABLED, bool ZERO_MEAN, bool UNIT_VAR, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using NormalDistributionPtr = std::shared_ptr<NormalDistribution<DIM,ENABLED,ZERO_MEAN,UNIT_VAR,SCALAR,OPTIONS>>;

/**
 * @makeNormalDistribution
 *
 * @brief
 * Helper function that uses information from the ROS parameter server to construct the given object.
 *
 * @templates
 * DIM: The number of dimensions each point will have
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 *
 * @parameters
 * node: The node with the right namespacing to have access to the parameters needed
 * prefix: The prefix of the parameter names
 *
 * @return
 * A fully constructed NormalDistribution.
 **/
template<Eigen::Index DIM, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
NoiseBasePtr<DIM,SCALAR,OPTIONS> makeNormalDistribution(const rclcpp::Node::SharedPtr& node, const std::string& prefix);
/**
 * @makeFirstOrderGaussMarkovDrivingNoise
 *
 * @brief
 * Helper function that uses information from the ROS parameter server to construct the given object.
 *
 * @templates
 * DIM: The number of dimensions each point will have
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 *
 * @parameters
 * node: The node with the right namespacing to have access to the parameters needed
 * prefix: The prefix of the parameter names
 *
 * @return
 * A fully constructed NormalDistribution for a first order Gauss Markov process.
 **/
template<Eigen::Index DIM, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
NoiseBasePtr<DIM,SCALAR,OPTIONS> makeFirstOrderGaussMarkovDrivingNoise(const rclcpp::Node::SharedPtr& node, const std::string& prefix);

/**
 * @DIM
 * The number of dimensions each point will have.
 *
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @ENABLED
 * False if and only if you want this noise source to be turned off.
 *
 * @ZERO_MEAN
 * True if this variable will always have zero mean.
 *
 * @UNIT_VAR
 * True if this variable will always have an identity for it's covariance matrix.
 *
 * @OPTIONS
 * Eigen Matrix options.
 **/
template<Eigen::Index DIM, bool ENABLED, bool ZERO_MEAN, bool UNIT_VAR, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class NormalDistribution
 : public NoiseBase<DIM,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  NormalDistribution() noexcept;
  /**
   * @Copy Constructor
   **/
  NormalDistribution(const NormalDistribution&) noexcept = default;
  /**
   * @Move Constructor
   **/
  NormalDistribution(NormalDistribution&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * Used to set the random number seed.
   *
   * @parameters
   * mean: The mean vector of the random variable
   * covariance: The covariance matrix of the random variable
   * seed: The seed to use for random number generation
   **/
  NormalDistribution(const Eigen::Matrix<SCALAR,1,  DIM,OPTIONS>& mean,
                     const Eigen::Matrix<SCALAR,DIM,DIM,OPTIONS>& covariance,
                     const unsigned                               seed = std::chrono::system_clock::now().time_since_epoch().count());
  /**
   * @Deconstructor
   **/
  ~NormalDistribution() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  NormalDistribution& operator=(const NormalDistribution&)  noexcept = default;
  NormalDistribution& operator=(      NormalDistribution&&) noexcept = default;
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
  // For random numbers
  std::default_random_engine       rand_generator;
  std::normal_distribution<SCALAR> distribution;
  // Defines distribution
  Eigen::Matrix<SCALAR,1,  DIM,OPTIONS> m_mean;
  Eigen::Matrix<SCALAR,DIM,DIM,OPTIONS> m_covariance;
  // Helpers
  Eigen::EigenSolver<Eigen::Matrix<SCALAR,DIM,DIM,OPTIONS>> m_cov_decomp;
  Eigen::Matrix<SCALAR,DIM,DIM,OPTIONS>                     m_cov_eigenvectors;
  Eigen::Matrix<SCALAR,1,  DIM,OPTIONS>                     m_cov_eigenvalues;
  Eigen::Matrix<SCALAR,DIM,DIM,OPTIONS>                     m_stddev;
};

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
NoiseBasePtr<DIM,SCALAR,OPTIONS> makeNormalDistribution(const rclcpp::Node::SharedPtr& node, const std::string& prefix)
{
  node->declare_parameter(prefix + ".enabled",            rclcpp::PARAMETER_BOOL);
  node->declare_parameter(prefix + ".standard_deviation", rclcpp::PARAMETER_DOUBLE_ARRAY);

  const bool enabled = node->get_parameter(prefix + ".enabled").as_bool();

  const std::vector<double> standard_deviation_vec = node->get_parameter(prefix + ".standard_deviation").as_double_array();
  assert(DIM == standard_deviation_vec.size());

  Eigen::Matrix<SCALAR,DIM,DIM,OPTIONS> covariance = Eigen::Matrix<SCALAR,DIM,DIM,OPTIONS>::Zero();
  for(size_t dim_it = 0; dim_it < DIM; ++dim_it)
  {
    covariance(dim_it,dim_it) = std::pow(standard_deviation_vec[dim_it], 2);
  }

  if(enabled)
  {
    return std::make_shared<NormalDistribution<DIM,true,true,false,SCALAR,OPTIONS>>(
             Eigen::Matrix<SCALAR,1,DIM,OPTIONS>::Zero(),
             covariance);
  }
  else // Not enabled
  {
    return std::make_shared<NormalDistribution<DIM,false,true,false,SCALAR,OPTIONS>>(
             Eigen::Matrix<SCALAR,1,DIM,OPTIONS>::Zero(),
             covariance);
  }
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
NoiseBasePtr<DIM,SCALAR,OPTIONS> makeFirstOrderGaussMarkovDrivingNoise(const rclcpp::Node::SharedPtr& node, const std::string& prefix)
{
  node->declare_parameter(prefix + ".enabled",            rclcpp::PARAMETER_BOOL);
  node->declare_parameter(prefix + ".standard_deviation", rclcpp::PARAMETER_DOUBLE_ARRAY);
  node->declare_parameter(prefix + ".time_constant",      rclcpp::PARAMETER_DOUBLE_ARRAY);

  const bool enabled = node->get_parameter(prefix + ".enabled").as_bool();

  const std::vector<double> standard_deviation_vec = node->get_parameter(prefix + ".standard_deviation").as_double_array();
  assert(DIM == standard_deviation_vec.size());

  const std::vector<double> time_constant_vec = node->get_parameter(prefix + ".time_constant").as_double_array();
  assert(DIM == time_constant_vec.size());

  Eigen::Matrix<SCALAR,DIM,DIM,OPTIONS> covariance = Eigen::Matrix<SCALAR,DIM,DIM,OPTIONS>::Zero();
  for(size_t dim_it = 0; dim_it < DIM; ++dim_it)
  {
    covariance(dim_it,dim_it) = (SCALAR(2) * std::pow(standard_deviation_vec[dim_it], 2)) / time_constant_vec[dim_it];
  }

  if(enabled)
  {
    return std::make_shared<NormalDistribution<DIM,true,true,false,SCALAR,OPTIONS>>(
             Eigen::Matrix<SCALAR,1,DIM,OPTIONS>::Zero(),
             covariance);
  }
  else // Not enabled
  {
    return std::make_shared<NormalDistribution<DIM,false,true,false,SCALAR,OPTIONS>>(
             Eigen::Matrix<SCALAR,1,DIM,OPTIONS>::Zero(),
             covariance);
  }
}

template<Eigen::Index DIM, bool ENABLED, bool ZERO_MEAN, bool UNIT_VAR, typename SCALAR, Eigen::StorageOptions OPTIONS>
NormalDistribution<DIM,ENABLED,ZERO_MEAN,UNIT_VAR,SCALAR,OPTIONS>::
  NormalDistribution(const Eigen::Matrix<SCALAR,1,  DIM,OPTIONS>& mean,
                     const Eigen::Matrix<SCALAR,DIM,DIM,OPTIONS>& covariance,
                     const unsigned                               seed)
 : NoiseBase<DIM,SCALAR,OPTIONS>(),
   rand_generator(seed),
   m_mean((ZERO_MEAN) ? Eigen::Matrix<SCALAR,1,DIM,OPTIONS>::Zero() : mean),
   m_covariance((UNIT_VAR) ? Eigen::Matrix<SCALAR,DIM,DIM,OPTIONS>::Identity() : covariance),
   m_cov_decomp(this->m_covariance),
   m_cov_eigenvectors(this->m_cov_decomp.eigenvectors().real()),
   m_cov_eigenvalues(this->m_cov_decomp.eigenvalues().real()),
   m_stddev(this->m_cov_eigenvectors * this->m_cov_eigenvalues.cwiseSqrt().asDiagonal())
{
  #ifndef NDEBUG
    assert(Eigen::NoConvergence != this->m_cov_decomp.info());
    assert(covariance.isApprox(covariance.transpose()));
    assert(covariance.isApprox((this->m_cov_decomp.eigenvectors() * this->m_covariance.eigenvalues().asDiagonal() * this->m_cov_decomp.eigenvectors().inverse()).real()));
    const Eigen::LDLT<Eigen::Matrix<SCALAR,DIM,DIM,OPTIONS>,Eigen::Upper> cov_decomp(covariance);
    assert(Eigen::NumericalIssue != cov_decomp.info());
    assert(cov_decomp.isPositive());
  #endif
}

template<Eigen::Index DIM, bool ENABLED, bool ZERO_MEAN, bool UNIT_VAR, typename SCALAR, Eigen::StorageOptions OPTIONS>
NormalDistribution<DIM,ENABLED,ZERO_MEAN,UNIT_VAR,SCALAR,OPTIONS>::NormalDistribution() noexcept
 : rand_generator(std::chrono::system_clock::now().time_since_epoch().count()),
   m_mean(Eigen::Matrix<SCALAR,1,DIM,OPTIONS>::Zero()),
   m_covariance(Eigen::Matrix<SCALAR,DIM,DIM,OPTIONS>::Identity()),
   m_cov_decomp(this->m_covariance),
   m_cov_eigenvectors(Eigen::Matrix<SCALAR,DIM,DIM,OPTIONS>::Identity()),
   m_cov_eigenvalues(Eigen::Matrix<SCALAR,1,DIM,OPTIONS>::Ones()),
   m_stddev(Eigen::Matrix<SCALAR,DIM,DIM,OPTIONS>::Identity())
{
  assert(ZERO_MEAN and UNIT_VAR);
}

template<Eigen::Index DIM, bool ENABLED, bool ZERO_MEAN, bool UNIT_VAR, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM,OPTIONS>
NormalDistribution<DIM,ENABLED,ZERO_MEAN,UNIT_VAR,SCALAR,OPTIONS>::getNoise()
{
  if constexpr(ENABLED)
  {
    Eigen::Matrix<SCALAR,1,DIM,OPTIONS> output
      = Eigen::Matrix<SCALAR,1,DIM,OPTIONS>::NullaryExpr(1, DIM, [&] () { return this->distribution(this->rand_generator); });

    if constexpr(not UNIT_VAR)
    {
      output = output * this->m_stddev;
    }
    if constexpr(not ZERO_MEAN)
    {
      output.array() += this->m_mean.array();
    }

    return output;
  }
  else
  {
    return Eigen::Matrix<SCALAR,1,DIM,OPTIONS>::Zero();
  }
}

template<Eigen::Index DIM, bool ENABLED, bool ZERO_MEAN, bool UNIT_VAR, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM,OPTIONS>
NormalDistribution<DIM,ENABLED,ZERO_MEAN,UNIT_VAR,SCALAR,OPTIONS>::getContinuousNoise(const SCALAR time_step)
{
  if constexpr(ENABLED)
  {
    Eigen::Matrix<SCALAR,1,DIM,OPTIONS> output
      = Eigen::Matrix<SCALAR,1,DIM,OPTIONS>::NullaryExpr(1, DIM, [&] () { return this->distribution(this->rand_generator); });

    if constexpr(not UNIT_VAR)
    {
      output.noalias() = output * (this->m_cov_eigenvectors *
                                   (this->m_cov_eigenvalues.array() / time_step).cwiseSqrt().matrix().asDiagonal());
    }
    if constexpr(not ZERO_MEAN)
    {
      output.array() += this->m_mean.array();
    }

    return output;
  }
  else
  {
    return Eigen::Matrix<SCALAR,1,DIM,OPTIONS>::Zero();
  }
}

template<Eigen::Index DIM, bool ENABLED, bool ZERO_MEAN, bool UNIT_VAR, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM,OPTIONS>
NormalDistribution<DIM,ENABLED,ZERO_MEAN,UNIT_VAR,SCALAR,OPTIONS>::getMean()
{
  return this->m_mean;
}

template<Eigen::Index DIM, bool ENABLED, bool ZERO_MEAN, bool UNIT_VAR, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,DIM,DIM,OPTIONS>
NormalDistribution<DIM,ENABLED,ZERO_MEAN,UNIT_VAR,SCALAR,OPTIONS>::getCovariance()
{
  return this->m_covariance;
}
} // namespace noise
} // namespace kf

#endif
/* normal_distribution.hpp */
