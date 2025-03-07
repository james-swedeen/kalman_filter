/**
 * @File: performance_evaluation.hpp
 * @Date: May 2022
 * @Author: James Swedeen
 *
 * @brief
 * Defines helper functions for extracting performance related information from the output.
 **/

#ifndef KALMAN_FILTER_MATH_PERFORMANCE_EVALUATION_HPP
#define KALMAN_FILTER_MATH_PERFORMANCE_EVALUATION_HPP

/* C++ Headers */
#include<vector>
#include<execution>

/* Boost Headers */
#include<boost/range/irange.hpp>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<kalman_filter/mappings/mappings_base.hpp>

namespace kf
{
namespace math
{
/**
 * @approxMeanTruthStateTrajectory
 *
 * @brief
 * Using the output of a Monte Carlo simulation set this function finds the average of the truth state trajectory.
 *
 * @templates
 * DIM_S: The type of a Dimensions object or an inheriting object that has information
 *        about the size of the state vectors
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 *
 * @parameters
 * mc_state_vectors: The vectors that runMonteCarlo produces
 * mappings: A helper object that maps one state vector to another
 *
 * @return
 * The average truth state trajectory.
 **/
template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::TRUTH_DIM,OPTIONS>
  approxMeanTruthStateTrajectory(const std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::MC::FULL_STATE_LEN,OPTIONS>>& mc_state_vectors,
                                 const map::MappingsBasePtr<DIM_S,SCALAR,OPTIONS>&                                          mappings) noexcept;
/**
 * @approxMeanNavStateTrajectory
 *
 * @brief
 * Using the output of a Monte Carlo simulation set this function finds the average of the navigation state trajectory.
 *
 * @templates
 * DIM_S: The type of a Dimensions object or an inheriting object that has information
 *        about the size of the state vectors
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 *
 * @parameters
 * mc_state_vectors: The vectors that runMonteCarlo produces
 * mappings: A helper object that maps one state vector to another
 *
 * @return
 * The average navigation state trajectory.
 **/
template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::NAV_DIM,OPTIONS>
  approxMeanNavStateTrajectory(const std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::MC::FULL_STATE_LEN,OPTIONS>>& mc_state_vectors,
                               const map::MappingsBasePtr<DIM_S,SCALAR,OPTIONS>&                                          mappings) noexcept;
/**
 * @findTruthStateDispersion
 *
 * @brief
 * Find the truth state dispersion of each run in a Monte Carlo run.
 *
 * @parameters
 * mc_state_vectors: The vectors that runMonteCarlo produces
 * avg_truth_state: The averaged truth state trajectory
 * mappings: A helper object that maps one state vector to another
 *
 * @return
 * The truth state dispersion of each simulation.
 **/
template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
inline std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::TRUTH_DISP_DIM,OPTIONS>>
  findTruthStateDispersion(const std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::MC::FULL_STATE_LEN,OPTIONS>>& mc_state_vectors,
                           const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::TRUTH_DIM,OPTIONS>&                       avg_truth_state,
                           const map::MappingsBasePtr<DIM_S,SCALAR,OPTIONS>&                                          mappings) noexcept;
/**
 * @findNavStateDispersion
 *
 * @brief
 * Find the navigation state dispersion of each run in a Monte Carlo run.
 *
 * @parameters
 * mc_state_vectors: The vectors that runMonteCarlo produces
 * avg_nav_state: The averaged navigation state trajectory
 * mappings: A helper object that maps one state vector to another
 *
 * @return
 * The navigation state dispersion of each simulation.
 **/
template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
inline std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::ERROR_DIM,OPTIONS>>
  findNavStateDispersion(const std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::MC::FULL_STATE_LEN,OPTIONS>>& mc_state_vectors,
                         const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::NAV_DIM,OPTIONS>&                         avg_nav_state,
                         const map::MappingsBasePtr<DIM_S,SCALAR,OPTIONS>&                                          mappings) noexcept;
/**
 * @findErrorStates
 *
 * @brief
 * Find the error states of each run in a Monte Carlo run.
 *
 * @parameters
 * mc_state_vectors: The vectors that runMonteCarlo produces
 * mappings: A helper object that maps one state vector to another
 *
 * @return
 * The error states of each simulation.
 **/
template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
inline std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::ERROR_DIM,OPTIONS>>
  findErrorStates(const std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::MC::FULL_STATE_LEN,OPTIONS>>& mc_state_vectors,
                  const map::MappingsBasePtr<DIM_S,SCALAR,OPTIONS>&                                          mappings) noexcept;
/**
 * @approxStateDispersionCovariance
 *
 * @brief
 * Used to approximate the covariance of given state dispersions from the output of a Monte Carlo simulation.
 *
 * @templates
 * DIM: Size of the dispersions vectors
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 *
 * @parameters
 * dispersions: The dispersions of each run over time
 *
 * @return
 * The approximated covariance of state dispersions.
 **/
template<Eigen::Index DIM, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
inline std::vector<Eigen::Matrix<SCALAR,DIM,DIM,OPTIONS>>
  approxStateDispersionCovariance(const std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& dispersions) noexcept;
/**
 * @truthStateDispersionCovariance
 *
 * @brief
 * Used to extract the covariance of the truth state dispersions from the augmented covariance matrix used in LinCov.
 *
 * @templates
 * DIM_S: The type of a Dimensions object or an inheriting object that has information
 *        about the size of the state vectors
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 *
 * @parameters
 * lincov_state_vector: The vector that runLinCov produces
 *
 * @return
 * The covariance of the truth state dispersions.
 **/
template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
inline std::vector<Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS>>
  truthStateDispersionCovariance(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>& lincov_state_vector) noexcept;
/**
 * @navStateDispersionCovariance
 *
 * @brief
 * Used to extract the covariance of the navigation state dispersions from the augmented covariance matrix used in LinCov.
 *
 * @templates
 * DIM_S: The type of a Dimensions object or an inheriting object that has information
 *        about the size of the state vectors
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 *
 * @parameters
 * lincov_state_vector: The vector that runLinCov produces
 *
 * @return
 * The covariance of the navigation state dispersions.
 **/
template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
inline std::vector<Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS>>
  navStateDispersionCovariance(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>& lincov_state_vector) noexcept;
/**
 * @errorStateCovariance
 *
 * @brief
 * Used to extract the error state covariance from the augmented covariance matrix used in LinCov.
 *
 * @templates
 * DIM_S: The type of a Dimensions object or an inheriting object that has information
 *        about the size of the state vectors
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 *
 * @parameters
 * lincov_state_vector: The vector that runLinCov produces
 * mappings: A helper object that maps one state vector to another
 * mc_state_vectors: The vectors that runMonteCarlo produces
 *
 * @return
 * The error state covariance.
 **/
template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
inline std::vector<Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS>>
  errorStateCovariance(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>& lincov_state_vector,
                       const map::MappingsBasePtr<DIM_S,SCALAR,OPTIONS>&                                 mappings) noexcept;
template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
inline std::vector<std::vector<Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS>>>
  errorStateCovariance(const std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::MC::FULL_STATE_LEN,OPTIONS>>& mc_state_vectors) noexcept;
/**
 * @averageErrorStateCovariance
 *
 * @brief
 * Used to find the average error state covariance.
 *
 * @templates
 * DIM_S: The type of a Dimensions object or an inheriting object that has information
 *        about the size of the state vectors
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 *
 * @parameters
 * error_state_covs: The error state covariance trajectories
 *
 * @return
 * The average error state covariance trajectory.
 **/
template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
inline std::vector<Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS>>
  averageErrorStateCovariance(const std::vector<std::vector<Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS>>>& error_state_covs) noexcept;
} // namespace math

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::TRUTH_DIM,OPTIONS>
  math::approxMeanTruthStateTrajectory(const std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::MC::FULL_STATE_LEN,OPTIONS>>& mc_state_vectors,
                                       const map::MappingsBasePtr<DIM_S,SCALAR,OPTIONS>&                                          mappings) noexcept
{
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::TRUTH_DIM,OPTIONS> output;

  const Eigen::Index num_sims    = mc_state_vectors.size();
  const Eigen::Index sims_length = mc_state_vectors[0].rows();
  output.resize(sims_length, Eigen::NoChange);

  const boost::integer_range<Eigen::Index> time_inds(0, sims_length);
  std::for_each(std::execution::par_unseq, time_inds.begin(), time_inds.end(),
    [&mappings, &output, &mc_state_vectors, num_sims] (const Eigen::Index time_it) -> void
    {
      Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::TRUTH_DIM,OPTIONS> states;
      states.resize(num_sims, Eigen::NoChange);

      for(Eigen::Index sim_it = 0; sim_it < num_sims; ++sim_it)
      {
        states.row(sim_it) = mc_state_vectors[sim_it].template block<1,DIM_S::TRUTH_DIM>(time_it, DIM_S::MC::TRUTH_START_IND);
      }

      output.row(time_it) = mappings->calculateAverageTruthState(states);
    });

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::NAV_DIM,OPTIONS>
  math::approxMeanNavStateTrajectory(const std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::MC::FULL_STATE_LEN,OPTIONS>>& mc_state_vectors,
                                     const map::MappingsBasePtr<DIM_S,SCALAR,OPTIONS>&                                          mappings) noexcept
{
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::NAV_DIM,OPTIONS> output;

  const Eigen::Index num_sims    = mc_state_vectors.size();
  const Eigen::Index sims_length = mc_state_vectors[0].rows();
  output.resize(sims_length, Eigen::NoChange);

  const boost::integer_range<Eigen::Index> time_inds(0, sims_length);
  std::for_each(std::execution::par_unseq, time_inds.begin(), time_inds.end(),
    [&mappings, &output, &mc_state_vectors, num_sims] (const Eigen::Index time_it) -> void
    {
      Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::NAV_DIM,OPTIONS> states;
      states.resize(num_sims, Eigen::NoChange);

      for(Eigen::Index sim_it = 0; sim_it < num_sims; ++sim_it)
      {
        states.row(sim_it) = mc_state_vectors[sim_it].template block<1,DIM_S::NAV_DIM>(time_it, DIM_S::MC::NAV_START_IND);
      }

      output.row(time_it) = mappings->calculateAverageNavState(states);
    });

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::TRUTH_DISP_DIM,OPTIONS>>
  math::findTruthStateDispersion(const std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::MC::FULL_STATE_LEN,OPTIONS>>& mc_state_vectors,
                                 const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::TRUTH_DIM,OPTIONS>&                       avg_truth_state,
                                 const map::MappingsBasePtr<DIM_S,SCALAR,OPTIONS>&                                          mappings) noexcept
{
  std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::TRUTH_DISP_DIM,OPTIONS>> output;

  const Eigen::Index num_sims    = mc_state_vectors.size();
  const Eigen::Index sims_length = avg_truth_state.rows();
  output.resize(num_sims);

  const boost::integer_range<Eigen::Index> sim_inds(0, num_sims);
  std::for_each(std::execution::par_unseq, sim_inds.begin(), sim_inds.end(),
    [&mappings, &output, &mc_state_vectors, &avg_truth_state, sims_length] (const Eigen::Index sim_it) -> void
    {
      output[sim_it].resize(sims_length, Eigen::NoChange);
      for(Eigen::Index time_it = 0; time_it < sims_length; ++time_it)
      {
        output[sim_it].row(time_it) = mappings->calculateTruthStateDisp(mc_state_vectors[sim_it].template block<1,DIM_S::TRUTH_DIM>(time_it, DIM_S::MC::TRUTH_START_IND),
                                                                        avg_truth_state.row(time_it));
      }
    });

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::ERROR_DIM,OPTIONS>>
  math::findNavStateDispersion(const std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::MC::FULL_STATE_LEN,OPTIONS>>& mc_state_vectors,
                               const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::NAV_DIM,OPTIONS>&                         avg_nav_state,
                               const map::MappingsBasePtr<DIM_S,SCALAR,OPTIONS>&                                          mappings) noexcept
{
  std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::ERROR_DIM,OPTIONS>> output;

  const Eigen::Index num_sims    = mc_state_vectors.size();
  const Eigen::Index sims_length = avg_nav_state.rows();
  output.resize(num_sims);

  const boost::integer_range<Eigen::Index> sim_inds(0, num_sims);
  std::for_each(std::execution::par_unseq, sim_inds.begin(), sim_inds.end(),
    [&mappings, &output, &mc_state_vectors, &avg_nav_state, sims_length] (const Eigen::Index sim_it) -> void
    {
      output[sim_it].resize(sims_length, Eigen::NoChange);
      for(Eigen::Index time_it = 0; time_it < sims_length; ++time_it)
      {
        output[sim_it].row(time_it) = mappings->calculateNavStateDisp(mc_state_vectors[sim_it].template block<1,DIM_S::NAV_DIM>(time_it, DIM_S::MC::NAV_START_IND),
                                                                      avg_nav_state.row(time_it));
      }
    });

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::ERROR_DIM,OPTIONS>>
  math::findErrorStates(const std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::MC::FULL_STATE_LEN,OPTIONS>>& mc_state_vectors,
                        const map::MappingsBasePtr<DIM_S,SCALAR,OPTIONS>&                                          mappings) noexcept
{
  std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::ERROR_DIM,OPTIONS>> output;

  const Eigen::Index num_sims    = mc_state_vectors.size();
  const Eigen::Index sims_length = mc_state_vectors[0].rows();
  output.resize(num_sims);

  const boost::integer_range<Eigen::Index> sim_inds(0, num_sims);
  std::for_each(std::execution::par_unseq, sim_inds.begin(), sim_inds.end(),
    [&mappings, &output, &mc_state_vectors, sims_length] (const Eigen::Index sim_it) -> void
    {
      output[sim_it].resize(sims_length, Eigen::NoChange);
      for(Eigen::Index time_it = 0; time_it < sims_length; ++time_it)
      {
        output[sim_it].row(time_it) = mappings->calculateErrorState(mc_state_vectors[sim_it].template block<1,DIM_S::TRUTH_DIM>(time_it, DIM_S::MC::TRUTH_START_IND),
                                                                    mc_state_vectors[sim_it].template block<1,DIM_S::NAV_DIM>(  time_it, DIM_S::MC::NAV_START_IND));
      }
    });

  return output;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline std::vector<Eigen::Matrix<SCALAR,DIM,DIM,OPTIONS>>
  math::approxStateDispersionCovariance(const std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& dispersions) noexcept
{
  std::vector<Eigen::Matrix<SCALAR,DIM,DIM,OPTIONS>> output;

  const Eigen::Index num_sims    = dispersions.size();
  const Eigen::Index sims_length = dispersions[0].rows();
  output.resize(sims_length);

  const boost::integer_range<Eigen::Index> time_inds(0, sims_length);
  std::for_each(std::execution::par_unseq, time_inds.begin(), time_inds.end(),
    [&output, &dispersions, num_sims] (const Eigen::Index time_it) -> void
    {
      output[time_it].setZero();

      for(Eigen::Index sim_it = 0; sim_it < num_sims; ++sim_it)
      {
        output[time_it].noalias() += (dispersions[sim_it].row(time_it).transpose() * dispersions[sim_it].row(time_it));
      }

      output[time_it].array() /= SCALAR(num_sims - 1);
    });

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline std::vector<Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS>>
  math::truthStateDispersionCovariance(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>& lincov_state_vector) noexcept
{
  std::vector<Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS>> output;

  const Eigen::Index lincov_state_vector_len = lincov_state_vector.rows();
  output.resize(lincov_state_vector_len);

  const boost::integer_range<Eigen::Index> lincov_state_inds(0, lincov_state_vector_len);
  std::for_each(std::execution::par_unseq, lincov_state_inds.begin(), lincov_state_inds.end(),
                [&output, &lincov_state_vector] (const Eigen::Index row_it) -> void
  {
    const Eigen::Map<const Eigen::Matrix<SCALAR,DIM_S::LINCOV::AUG_DIM,DIM_S::LINCOV::AUG_DIM,OPTIONS>> aug_covariance(
      lincov_state_vector.template block<1,DIM_S::LINCOV::AUG_COV_LEN>(row_it, DIM_S::LINCOV::AUG_COV_START_IND).data());

    output[row_it] = aug_covariance.template block<DIM_S::TRUTH_DISP_DIM,
                                                   DIM_S::TRUTH_DISP_DIM>(DIM_S::LINCOV::TRUTH_DISP_START_IND,
                                                                          DIM_S::LINCOV::TRUTH_DISP_START_IND);
  });

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline std::vector<Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS>>
  math::navStateDispersionCovariance(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>& lincov_state_vector) noexcept
{
  std::vector<Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS>> output;

  const Eigen::Index lincov_state_vector_len = lincov_state_vector.rows();
  output.resize(lincov_state_vector_len);

  const boost::integer_range<Eigen::Index> lincov_state_inds(0, lincov_state_vector_len);
  std::for_each(std::execution::par_unseq, lincov_state_inds.begin(), lincov_state_inds.end(),
                [&output, &lincov_state_vector] (const Eigen::Index row_it) -> void
  {
    const Eigen::Map<const Eigen::Matrix<SCALAR,DIM_S::LINCOV::AUG_DIM,DIM_S::LINCOV::AUG_DIM,OPTIONS>> aug_covariance(
      lincov_state_vector.template block<1,DIM_S::LINCOV::AUG_COV_LEN>(row_it, DIM_S::LINCOV::AUG_COV_START_IND).data());

    output[row_it] = aug_covariance.template block<DIM_S::ERROR_DIM,
                                                   DIM_S::ERROR_DIM>(DIM_S::LINCOV::NAV_DISP_START_IND,
                                                                     DIM_S::LINCOV::NAV_DISP_START_IND);
  });

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline std::vector<Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS>>
  math::errorStateCovariance(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>& lincov_state_vector,
                             const map::MappingsBasePtr<DIM_S,SCALAR,OPTIONS>&                                 mappings) noexcept
{
  std::vector<Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS>> output;

  const Eigen::Index lincov_state_vector_len = lincov_state_vector.rows();
  output.resize(lincov_state_vector_len);

  const boost::integer_range<Eigen::Index> lincov_state_inds(0, lincov_state_vector_len);
  std::for_each(std::execution::par_unseq, lincov_state_inds.begin(), lincov_state_inds.end(),
                [&output, &lincov_state_vector, &mappings] (const Eigen::Index row_it) -> void
  {
    const Eigen::Map<const Eigen::Matrix<SCALAR,DIM_S::LINCOV::AUG_DIM,DIM_S::LINCOV::AUG_DIM,OPTIONS>> aug_covariance(
      lincov_state_vector.template block<1,DIM_S::LINCOV::AUG_COV_LEN>(row_it, DIM_S::LINCOV::AUG_COV_START_IND).data());

    Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::LINCOV::AUG_DIM,OPTIONS> temp;
    temp.template middleCols<DIM_S::TRUTH_DISP_DIM>(DIM_S::LINCOV::TRUTH_DISP_START_IND) =
      -mappings->getTruthNavMapPDWRDispersionState(mappings->mapRefTruth(lincov_state_vector.template block<1,DIM_S::REF_DIM>(row_it, DIM_S::REF_START_IND)));
    temp.template middleCols<DIM_S::ERROR_DIM>(DIM_S::LINCOV::NAV_DISP_START_IND).setIdentity();

    output[row_it].noalias() = temp * aug_covariance * temp.transpose();
  });

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline std::vector<std::vector<Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS>>>
  math::errorStateCovariance(const std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::MC::FULL_STATE_LEN,OPTIONS>>& mc_state_vectors) noexcept
{
  std::vector<std::vector<Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS>>> output;

  const Eigen::Index num_sims = mc_state_vectors.size();
  const Eigen::Index sim_len  = mc_state_vectors[0].rows();
  output.resize(num_sims);

  const boost::integer_range<Eigen::Index> sim_inds(0, num_sims);
  std::for_each(std::execution::par_unseq, sim_inds.begin(), sim_inds.end(),
                [&output, &mc_state_vectors, sim_len] (const Eigen::Index sim_it) -> void
  {
    output[sim_it].resize(sim_len);

    for(Eigen::Index time_ind = 0; time_ind < sim_len; ++time_ind)
    {
      output[sim_it][time_ind] =
        Eigen::Map<const Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS>>(
          mc_state_vectors[sim_it].template block<1,DIM_S::ERROR_COV_LEN>(time_ind, DIM_S::ERROR_COV_START_IND).data());
    }
  });

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline std::vector<Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS>>
  math::averageErrorStateCovariance(const std::vector<std::vector<Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS>>>& error_state_covs) noexcept
{
  std::vector<Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS>> output;

  const Eigen::Index num_sims = error_state_covs.size();
  const Eigen::Index sim_len  = error_state_covs[0].size();

  output.resize(sim_len);
  for(Eigen::Index time_it = 0; time_it < sim_len; ++time_it)
  {
    output[time_it].setZero();
    for(Eigen::Index sim_it = 0; sim_it < num_sims; ++sim_it)
    {
      output[time_it] += error_state_covs[sim_it][time_it];
    }
    output[time_it] /= SCALAR(num_sims);
  }

  return output;
}
} // namespace kf

#endif
/* performance_evaluation.hpp */
