/**
 * @File: noise_wrapper.hpp
 * @Date: August 2022
 * @Author: James Swedeen
 *
 * @brief
 * A wrapper class that wraps noise objects and provides extra run-time functionality.
 **/

#ifndef KALMAN_FILTER_NOISE_NOISE_WRAPPER_HPP
#define KALMAN_FILTER_NOISE_NOISE_WRAPPER_HPP

/* C++ Headers */
#include<memory>
#include<string>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<kalman_filter/noise/noise_base.hpp>

namespace kf
{
namespace noise
{
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
class NoiseWrapper;

template<Eigen::Index DIM, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using NoiseWrapperPtr = std::shared_ptr<NoiseWrapper<DIM,SCALAR,OPTIONS>>;

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
class NoiseWrapper
 : public NoiseBase<DIM,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  NoiseWrapper() = delete;
  /**
   * @Copy Constructor
   **/
  NoiseWrapper(const NoiseWrapper&) noexcept = default;
  /**
   * @Move Constructor
   **/
  NoiseWrapper(NoiseWrapper&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * Initializes the object for use.
   *
   * @parameters
   * wrapped_noise: The noise object that this object wraps
   * name: The name of this noise source
   * init_enabled: True if this noise source should be initialized enabled and false if not
   **/
  explicit NoiseWrapper(const NoiseBasePtr<DIM,SCALAR,OPTIONS>& wrapped_noise,
                        const std::string&                      name         = std::string(),
                        const bool                              init_enabled = true) noexcept;
  /**
   * @Deconstructor
   **/
 ~NoiseWrapper() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  NoiseWrapper& operator=(const NoiseWrapper&)  noexcept = default;
  NoiseWrapper& operator=(      NoiseWrapper&&) noexcept = default;
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
  /**
   * @getName
   *
   * @brief
   * Used to get the name of this noise source.
   *
   * @return
   * The name of this noise source.
   **/
  inline const std::string& getName() const noexcept;
  /**
   * @isEnabled
   *
   * @brief
   * Tests if this object is returning true noise and covariance values or zeroed out ones.
   *
   * @return
   * True if this object is enabled and making real noise.
   **/
  inline bool isEnabled() const noexcept;
  /**
   * @set
   *
   * @brief
   * Sets the parameter in question.
   **/
  inline void setEnabled()  noexcept;
  inline void setDisabled() noexcept;
private:
  NoiseBasePtr<DIM,SCALAR,OPTIONS> wrapped_noise;
  std::string                      name;
  bool                             enabled;
};

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
NoiseWrapper<DIM,SCALAR,OPTIONS>::NoiseWrapper(const NoiseBasePtr<DIM,SCALAR,OPTIONS>& wrapped_noise,
                                               const std::string&                      name,
                                               const bool                              init_enabled) noexcept
: NoiseBase<DIM,SCALAR,OPTIONS>(),
  wrapped_noise(wrapped_noise),
  name(name),
  enabled(init_enabled)
{}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM,OPTIONS> NoiseWrapper<DIM,SCALAR,OPTIONS>::getNoise()
{
  if(this->isEnabled())
  {
    return this->wrapped_noise->getNoise();
  }
  return Eigen::Matrix<SCALAR,1,DIM,OPTIONS>::Zero();
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM,OPTIONS> NoiseWrapper<DIM,SCALAR,OPTIONS>::getContinuousNoise(const SCALAR time_step)
{
  if(this->isEnabled())
  {
    return this->wrapped_noise->getContinuousNoise(time_step);
  }
  return Eigen::Matrix<SCALAR,1,DIM,OPTIONS>::Zero();
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM,OPTIONS> NoiseWrapper<DIM,SCALAR,OPTIONS>::getMean()
{
  if(this->isEnabled())
  {
    return this->wrapped_noise->getMean();
  }
  return Eigen::Matrix<SCALAR,1,DIM,OPTIONS>::Zero();
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,DIM,DIM,OPTIONS> NoiseWrapper<DIM,SCALAR,OPTIONS>::getCovariance()
{
  if(this->isEnabled())
  {
    return this->wrapped_noise->getCovariance();
  }
  return Eigen::Matrix<SCALAR,DIM,DIM,OPTIONS>::Zero();
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline const std::string& NoiseWrapper<DIM,SCALAR,OPTIONS>::getName() const noexcept
{
  return this->name;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool NoiseWrapper<DIM,SCALAR,OPTIONS>::isEnabled() const noexcept
{
  return this->enabled;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void NoiseWrapper<DIM,SCALAR,OPTIONS>::setEnabled() noexcept
{
  this->enabled = true;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void NoiseWrapper<DIM,SCALAR,OPTIONS>::setDisabled() noexcept
{
  this->enabled = false;
}
} // namespace noise
} // namespace kf

#endif
/* noise_wrapper.hpp */
