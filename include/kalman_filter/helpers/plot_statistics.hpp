/**
 * @File: plot_statistics.hpp
 * @Date: May 2022
 * @Author: James Swedeen
 *
 * @brief
 * Defines helper functions for extracting and plotting performance related information from the outputs of
 * Monte Carlo and LinCov runs.
 **/

#ifndef KALMAN_FILTER_HELPERS_PLOT_STATISTICS_HPP
#define KALMAN_FILTER_HELPERS_PLOT_STATISTICS_HPP

/* C++ Headers */
#include<vector>
#include<array>
#include<list>
#include<string>
#include<variant>
#include<cmath>
#include<execution>

/* Boost Headers */
#include<boost/range/irange.hpp>

/* Eigen Headers */
#include<Eigen/Dense>

/* Mat Plot Lib CPP Headers */
#include<matplotlibcpp/matplotlibcpp.hpp>

/* Local Headers */
#include<kalman_filter/helpers/versions.hpp>
#include<kalman_filter/mappings/mappings_base.hpp>
#include<kalman_filter/controllers/controller_base.hpp>
#include<kalman_filter/noise/noise_wrapper.hpp>
#include<kalman_filter/math/performance_evaluation.hpp>
#include<kalman_filter/helpers/tools.hpp>
#include<kalman_filter/run_lin_cov.hpp>
#include<kalman_filter/helpers/plot_statistics_tools.hpp>

using PlotTypeInds = struct kf::plottools::PlotTypeInds;
namespace kf
{
namespace plot
{
/**
 * @plotDebuggingStatistics
 *
 * @brief
 * Plots statistics related to debugging the Kalman Filter equations.
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
 * names: The names of each vector as they show up in the error states, how many states there are,
 *        and the starting indexes for the navigation dispersion states.
 * mc_run_down_sample: Will only plot every 1 in mc_run_down_sample Monte Carlo runs
 * block: True if you want this function to block and make the plots
 **/
template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
void
  plotDebuggingStatistics(const std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::MC::FULL_STATE_LEN,OPTIONS>>& mc_state_vectors,
                          const map::MappingsBasePtr<DIM_S,SCALAR,OPTIONS>&                                          mappings,
                          const std::vector<std::pair<std::string,std::tuple<Eigen::Index,Eigen::Index>>>&           names,
                          const size_t                                                                               mc_run_down_sample = 1,
                          const bool                                                                                 block = true) noexcept;
/**
 * @plotControlStatistics
 *
 * @brief
 * Plots all related statistics to the controller for both Monte Carlo and LinCov results.
 *
 * @templates
 * DIM_S: The type of a Dimensions object or an inheriting object that has information
 *        about the size of the state vectors
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 *
 * @parameters
 * mc_state_vectors: The vectors that runMonteCarlo produces
 * lincov_state_vector: The vector that runLinCov produces
 * dynamics: Defines the truth and navigation state dynamics
 * mappings: A helper object that maps one state vector to another
 * controller: A helper that is used to calculate control inputs
 * names: The names of each state as they show up in the control vector and their indexes for the control vector.
 * mc_run_down_sample: Will only plot every 1 in mc_run_down_sample Monte Carlo runs
 * block: True if you want this function to block and make the plots
 **/
template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
void
  plotControlStatistics(const std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::MC::FULL_STATE_LEN,OPTIONS>>& mc_state_vectors,
                        const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>&          lincov_state_vector,
                        const dynamics::DynamicsBasePtr<DIM_S,SCALAR,OPTIONS>&                                     dynamics,
                        const map::MappingsBasePtr<DIM_S,SCALAR,OPTIONS>&                                          mappings,
                        const control::ControllerBasePtr<DIM_S,SCALAR,OPTIONS>&                                    controller,
                        const std::vector<std::pair<std::string,Eigen::Index>>&                                    names,
                        const size_t                                                                               mc_run_down_sample = 1,
                        const bool                                                                                 block = true) noexcept;
/**
 * @plotAllStatistics
 *
 * @brief
 * Plots all related statistics for both Monte Carlo and LinCov results.
 *
 * @templates
 * DIM_S: The type of a Dimensions object or an inheriting object that has information
 *        about the size of the state vectors
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 *
 * @parameters
 * mc_state_vectors: The vectors that runMonteCarlo produces
 * lincov_state_vector: The vector that runLinCov produces
 * mappings: A helper object that maps one state vector to another
 * truth_to_plot: Functor that maps the truth state vector to something that is plottable
 * nav_to_plot: Functor that maps the navigation state vector to something that is plottable
 * names: The names of each vector as they show up in the error states, how many states there are,
 *        and the starting indexes for the reference states, truth dispersion states, and navigation dispersion states
 *        in that order. -1 if not part of that state
 * plot_types: True at every index that corresponds to the types of plots that this function should generate, see
 *             PlotTypeInds to see what indexes correspond to what plots
 * mc_run_down_sample: Will only plot every 1 in mc_run_down_sample Monte Carlo runs
 * block: True if you want this function to block and make the plots
 **/
template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
void
  plotAllStatistics(const std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::MC::FULL_STATE_LEN,OPTIONS>>&                 mc_state_vectors,
                    const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>&                          lincov_state_vector,
                    const map::MappingsBasePtr<DIM_S,SCALAR,OPTIONS>&                                                          mappings,
                    const std::function<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::TRUTH_DISP_DIM,OPTIONS>(
                                          const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::TRUTH_DIM,OPTIONS>&)>&              truth_to_plot,
                    const std::function<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::ERROR_DIM,OPTIONS>(
                                          const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::NAV_DIM,OPTIONS>&)>&                nav_to_plot,
                    const std::vector<std::pair<std::string,std::tuple<Eigen::Index,Eigen::Index,Eigen::Index,Eigen::Index>>>& names,
                    const std::array<bool,6>&                                                                                  plot_types,
                    const size_t                                                                                               mc_run_down_sample = 1,
                    const bool                                                                                                 block = true) noexcept;

template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
void
  plotAllStatistics(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>&                          lincov_state_vector,
                    const map::MappingsBasePtr<DIM_S,SCALAR,OPTIONS>&                                                          mappings,
                    const std::vector<std::pair<std::string,std::tuple<Eigen::Index,Eigen::Index,Eigen::Index,Eigen::Index>>>& names,
                    const std::array<bool,4>&                                                                                  plot_types,
                    const bool                                                                                                 block = true) noexcept;
/**
 * @plotErrorBudget
 *
 * @brief
 * Plot the error budget for each sensor along the given trajectory.
 *
 * @templates
 * NUM_PLOTS: The number of plots to make
 * DIM_S: The type of a Dimensions object or an inheriting object that has information
 *        about the size of the state vectors
 * VERSION: Controls what type of simulation will be ran
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 *
 * @parameters
 * reference_trajectory: The nominal reference trajectory for this set of simulations
 * nominal_initial_state: The starting_state for the simulation
 * tools: Holds all of the needed helper functions
 * noise_sources: Each source of noise in the simulation
 * data_extraction_func: Function that extracts the data to be plotted
 * data_names: The names of each thing being plotted
 * block: True if you want this function to block and make the plots
 **/
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
using GeneralNoiseWrapper = std::variant<noise::NoiseWrapperPtr<1,SCALAR,OPTIONS>,
                                         noise::NoiseWrapperPtr<3,SCALAR,OPTIONS>>;

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
using DATA_FUNC = std::function<Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS>(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>&)>;

template<typename DIM_S, Versions VERSION, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
void
  plotErrorBudget(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::REF_DIM,OPTIONS>>&   reference_trajectory,
                  const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& nominal_initial_state,
                  const Tools<DIM_S,SCALAR,OPTIONS>&                                                     tools,
                  const std::list<GeneralNoiseWrapper<SCALAR,OPTIONS>>&                                  noise_sources,
                  const DATA_FUNC<DIM_S,SCALAR,OPTIONS>&                                                 data_extraction_func,
                  const std::string&                                                                     data_name,
                  const bool                                                                             block = true) noexcept;
namespace helpers
{
  template<typename SCALAR, Eigen::StorageOptions OPTIONS>
  inline void setNoiseEnabled(const GeneralNoiseWrapper<SCALAR,OPTIONS>& noise) noexcept;
  template<typename SCALAR, Eigen::StorageOptions OPTIONS>
  inline void setNoiseDisabled(const GeneralNoiseWrapper<SCALAR,OPTIONS>& noise) noexcept;
  template<typename SCALAR, Eigen::StorageOptions OPTIONS>
  inline const std::string& getNoiseName(const GeneralNoiseWrapper<SCALAR,OPTIONS>& noise) noexcept;
} // namespace helpers
} // namespace plot

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
void plot::
  plotDebuggingStatistics(const std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::MC::FULL_STATE_LEN,OPTIONS>>& mc_state_vectors,
                          const map::MappingsBasePtr<DIM_S,SCALAR,OPTIONS>&                                          mappings,
                          const std::vector<std::pair<std::string,std::tuple<Eigen::Index,Eigen::Index>>>&           names,
                          const size_t                                                                               mc_run_down_sample,
                          const bool                                                                                 block) noexcept
{
  const size_t sim_len   = mc_state_vectors[0].rows();
  const size_t names_len = names.size();
  const size_t num_sims  = mc_state_vectors.size();

  const boost::integer_range<Eigen::Index> time_inds(0, sim_len);

  // Calculate information about this problem
  std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,  DIM_S::ERROR_DIM,OPTIONS>> error_states;
  std::vector<Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS>> avg_error_cov;
  std::vector<Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS>> avg_kf_error_cov;

  std::vector<std::vector<Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,OPTIONS>>> kf_error_cov;

  error_states     = math::findErrorStates<DIM_S,SCALAR,OPTIONS>(mc_state_vectors, mappings);
  avg_error_cov    = math::approxStateDispersionCovariance<DIM_S::ERROR_DIM,SCALAR,OPTIONS>(error_states);
  kf_error_cov     = math::errorStateCovariance<DIM_S,SCALAR,OPTIONS>(mc_state_vectors);
  avg_kf_error_cov = math::averageErrorStateCovariance<DIM_S,SCALAR,OPTIONS>(kf_error_cov);

  // Plot
  // Get time vector
  std::vector<SCALAR> time(sim_len);
  std::for_each(std::execution::par_unseq, time_inds.begin(), time_inds.end(),
  [&time, &mc_state_vectors] (const Eigen::Index time_it) -> void
  {
    time[time_it] = mc_state_vectors[0](time_it, DIM_S::TIME_IND);
  });

  for(size_t name_it = 0; name_it < names_len; ++name_it)
  {
    const Eigen::Index num_states    = std::get<0>(names[name_it].second);
    const Eigen::Index nav_start_ind = std::get<1>(names[name_it].second);

    std::vector<std::vector<std::vector<SCALAR>>> sub_error;
    std::vector<std::vector<std::vector<SCALAR>>> sub_kf_error_cov;

    std::vector<std::vector<SCALAR>> mc_three_sig_error;
    std::vector<std::vector<SCALAR>> kf_three_sig_error;

    sub_error.         resize(num_states);
    sub_kf_error_cov.  resize(num_states);
    mc_three_sig_error.resize(num_states);
    kf_three_sig_error.resize(num_states);

    for(Eigen::Index dim_it = 0; dim_it < num_states; ++dim_it)
    {
      sub_error[         dim_it].resize(num_sims);
      sub_kf_error_cov[  dim_it].resize(num_sims);
      mc_three_sig_error[dim_it].resize(sim_len);
      kf_three_sig_error[dim_it].resize(sim_len);

      for(size_t sim_it = 0; sim_it < num_sims; ++sim_it)
      {
        sub_error[       dim_it][sim_it].resize(sim_len);
        sub_kf_error_cov[dim_it][sim_it].resize(sim_len);
      }
    }

    // Convert to the form that matplotlib needs
    std::for_each(std::execution::par_unseq, time_inds.begin(), time_inds.end(),
    [&] (const Eigen::Index time_it) -> void
    {
      for(Eigen::Index dim_it = 0; dim_it < num_states; ++dim_it)
      {
        for(size_t sim_it = 0; sim_it < num_sims; ++sim_it)
        {
          sub_error[       dim_it][sim_it][time_it] = error_states[sim_it](time_it, nav_start_ind + dim_it);
          sub_kf_error_cov[dim_it][sim_it][time_it] = SCALAR(3) * std::sqrt(kf_error_cov[sim_it][time_it](nav_start_ind + dim_it, nav_start_ind + dim_it));
        }
        mc_three_sig_error[dim_it][time_it] = SCALAR(3) * std::sqrt(avg_error_cov[   time_it](nav_start_ind + dim_it, nav_start_ind + dim_it));
        kf_three_sig_error[dim_it][time_it] = SCALAR(3) * std::sqrt(avg_kf_error_cov[time_it](nav_start_ind + dim_it, nav_start_ind + dim_it));
      }
    });

    // Plot the states
    matplotlibcpp::figure();
    for(Eigen::Index dim_it = 0; dim_it < num_states; ++dim_it)
    {
      matplotlibcpp::subplot(num_states, 1, dim_it+1);
      for(size_t sim_it = 0; sim_it < num_sims; sim_it += mc_run_down_sample)
      {
        matplotlibcpp::plot<SCALAR>(time, sub_error[       dim_it][sim_it], "y");
        matplotlibcpp::plot<SCALAR>(time, sub_kf_error_cov[dim_it][sim_it], "g");
      }
      matplotlibcpp::named_plot<SCALAR,SCALAR>("3-sigma True Avg", time, mc_three_sig_error[dim_it], "r-.");
      std::transform(mc_three_sig_error[dim_it].cbegin(), mc_three_sig_error[dim_it].cend(), mc_three_sig_error[dim_it].begin(), std::negate<SCALAR>());
      matplotlibcpp::plot<SCALAR,SCALAR>(time, mc_three_sig_error[dim_it], "r-.");

      matplotlibcpp::named_plot<SCALAR,SCALAR>("3-sigma KF Avg", time, kf_three_sig_error[dim_it], "b--");
      std::transform(kf_three_sig_error[dim_it].cbegin(), kf_three_sig_error[dim_it].cend(), kf_three_sig_error[dim_it].begin(), std::negate<SCALAR>());
      matplotlibcpp::plot<SCALAR>(time, kf_three_sig_error[dim_it], "b--");
      matplotlibcpp::xlabel("Time (sec)");
      switch(dim_it)
      {
        case 0:
          matplotlibcpp::ylabel(names[name_it].first + " X Est. Error");
          matplotlibcpp::legend();
          matplotlibcpp::title(names[name_it].first + " Est. Error");
          break;
        case 1:
          matplotlibcpp::ylabel(names[name_it].first + " Y Est. Error");
          break;
        case 2:
          matplotlibcpp::ylabel(names[name_it].first + " Z Est. Error");
          break;
        default:
          assert(false);
          break;
      };
    }
  }

  if(block)
  {
    matplotlibcpp::show();
  }
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
void plot::
  plotControlStatistics(const std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::MC::FULL_STATE_LEN,OPTIONS>>& mc_state_vectors_full,
                        const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>&          lincov_state_vector,
                        const dynamics::DynamicsBasePtr<DIM_S,SCALAR,OPTIONS>&                                     dynamics,
                        const map::MappingsBasePtr<DIM_S,SCALAR,OPTIONS>&                                          mappings,
                        const control::ControllerBasePtr<DIM_S,SCALAR,OPTIONS>&                                    controller,
                        const std::vector<std::pair<std::string,Eigen::Index>>&                                    names,
                        const size_t                                                                               mc_run_down_sample,
                        const bool                                                                                 block) noexcept
{
  const size_t sim_len   = lincov_state_vector.rows();
  const size_t names_len = names.size();
  const size_t num_sims  = mc_state_vectors_full.size();

  const boost::integer_range<Eigen::Index> time_inds(0, sim_len-1);
  const boost::integer_range<Eigen::Index> sim_inds (0, num_sims);

  // Calculate information about this problem
  std::vector<SCALAR>                                             time_steps(    sim_len-1);
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::CONTROL_DIM,OPTIONS> ol_control(    sim_len-1, DIM_S::CONTROL_DIM);
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::CONTROL_DIM,OPTIONS> avg_control(   sim_len-1, DIM_S::CONTROL_DIM);
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::CONTROL_DIM,OPTIONS> avg_cl_control(sim_len-1, DIM_S::CONTROL_DIM);
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::NAV_DIM,    OPTIONS> ref_nav_state( sim_len,   DIM_S::NAV_DIM);

  std::vector<Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,  DIM_S::ERROR_DIM,  OPTIONS>> nav_disp_cov;
  std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,    DIM_S::CONTROL_DIM,OPTIONS>> control(          num_sims);
  std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,    DIM_S::CONTROL_DIM,OPTIONS>> cl_control(       num_sims);
  std::vector<Eigen::Matrix<SCALAR,DIM_S::CONTROL_DIM,DIM_S::CONTROL_DIM,OPTIONS>> mc_control_cov(   sim_len-1);
  std::vector<Eigen::Matrix<SCALAR,DIM_S::CONTROL_DIM,DIM_S::CONTROL_DIM,OPTIONS>> mc_cl_control_cov;
  std::vector<Eigen::Matrix<SCALAR,DIM_S::CONTROL_DIM,DIM_S::CONTROL_DIM,OPTIONS>> lc_control_cov(   sim_len-1);

  std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::MC::FULL_STATE_LEN,OPTIONS>> mc_state_vectors(num_sims);
  {
    std::for_each(std::execution::par_unseq, sim_inds.begin(), sim_inds.end(),
    [&mc_state_vectors, &mc_state_vectors_full, sim_len] (const Eigen::Index sim_it) -> void
    {
      mc_state_vectors[sim_it].resize(sim_len, DIM_S::MC::FULL_STATE_LEN);
      mc_state_vectors[sim_it].row(0) = mc_state_vectors_full[sim_it].row(0);
    });
    const Eigen::Index mc_sim_len = mc_state_vectors_full.front().rows();
    size_t mc_time_ind = 0;
    for(size_t lc_time_ind = 1; lc_time_ind < sim_len; ++lc_time_ind)
    {
      // Find corresponding MC index
      for(; mc_time_ind < mc_sim_len; ++mc_time_ind)
      {
        if(lincov_state_vector(lc_time_ind, DIM_S::TIME_IND) <= mc_state_vectors_full.front()(mc_time_ind, DIM_S::TIME_IND))
        {
          break;
        }
      }
      // Copy MC states over
      std::for_each(std::execution::par_unseq, sim_inds.begin(), sim_inds.end(),
      [&mc_state_vectors, &mc_state_vectors_full, lc_time_ind, mc_time_ind] (const Eigen::Index sim_it) -> void
      {
        mc_state_vectors[sim_it].row(lc_time_ind) = mc_state_vectors_full[sim_it].row(mc_time_ind);
      });
    }
  }

  for(size_t sim_it = 0; sim_it < num_sims; ++sim_it)
  {
    control[   sim_it].resize(sim_len-1, Eigen::NoChange);
    cl_control[sim_it].resize(sim_len-1, Eigen::NoChange);
  }
  std::for_each(std::execution::par_unseq, time_inds.begin(), time_inds.end(),
  [&ref_nav_state, &mappings, &lincov_state_vector] (const Eigen::Index time_it) -> void
  {
    ref_nav_state.row(time_it) = mappings->mapRefNav(lincov_state_vector.template block<1,DIM_S::REF_DIM>(time_it, DIM_S::REF_START_IND));
  });
  std::for_each(std::execution::par_unseq, time_inds.begin(), time_inds.end(),
  [&time_steps, &dynamics, &lincov_state_vector, &controller, &ol_control, &ref_nav_state] (const Eigen::Index time_it) -> void
  {
    time_steps[time_it] =
      dynamics->findTimeStep(lincov_state_vector.template block<1,DIM_S::REF_DIM>(time_it,   DIM_S::REF_START_IND),
                             lincov_state_vector.template block<1,DIM_S::REF_DIM>(time_it+1, DIM_S::REF_START_IND));
    ol_control.row(time_it) = controller->getControl(lincov_state_vector(time_it, DIM_S::TIME_IND),
                                                     ref_nav_state.row(time_it),
                                                     lincov_state_vector.template block<1,DIM_S::REF_DIM>(time_it,   DIM_S::REF_START_IND),
                                                     lincov_state_vector.template block<1,DIM_S::REF_DIM>(time_it+1, DIM_S::REF_START_IND),
                                                     time_steps[time_it]);
  });
  for(size_t sim_it = 0; sim_it < num_sims; ++sim_it)
  {
    std::for_each(std::execution::par_unseq, time_inds.begin(), time_inds.end(),
    [&time_steps, &controller, &control, &lincov_state_vector, &mc_state_vectors, sim_it] (const Eigen::Index time_it) -> void
    {
      control[sim_it].row(time_it) = controller->getControl(lincov_state_vector(time_it, DIM_S::TIME_IND),
                                                            mc_state_vectors[sim_it].template block<1,DIM_S::NAV_DIM>(time_it,   DIM_S::MC::NAV_START_IND),
                                                            lincov_state_vector.     template block<1,DIM_S::REF_DIM>(time_it,   DIM_S::REF_START_IND),
                                                            lincov_state_vector.     template block<1,DIM_S::REF_DIM>(time_it+1, DIM_S::REF_START_IND),
                                                            time_steps[time_it]);
    });
    cl_control[sim_it] = control[sim_it] - ol_control;
  }
  avg_control.   setZero();
  avg_cl_control.setZero();
  for(size_t sim_it = 0; sim_it < num_sims; ++sim_it)
  {
    avg_control    += control[   sim_it];
    avg_cl_control += cl_control[sim_it];
  }
  avg_control    /= SCALAR(num_sims);
  avg_cl_control /= SCALAR(num_sims);

  std::for_each(std::execution::par_unseq, time_inds.begin(), time_inds.end(),
  [&mc_control_cov, &avg_control, &control, num_sims] (const Eigen::Index time_it) -> void
  {
    mc_control_cov[time_it].setZero();
    for(Eigen::Index sim_it = 0; sim_it < num_sims; ++sim_it)
    {
      const Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,OPTIONS> zero_mean_control = control[sim_it].row(time_it) - avg_control.row(time_it);

      mc_control_cov[time_it].noalias() += (zero_mean_control.transpose() * zero_mean_control);
    }
    mc_control_cov[time_it].array() /= SCALAR(num_sims-1);
  });
  mc_cl_control_cov = math::approxStateDispersionCovariance<DIM_S::CONTROL_DIM,SCALAR,OPTIONS>(cl_control);
  nav_disp_cov      = math::navStateDispersionCovariance<DIM_S,SCALAR,OPTIONS>(lincov_state_vector);
  std::for_each(std::execution::par_unseq, time_inds.begin(), time_inds.end(),
  [&controller, &lincov_state_vector, &ref_nav_state, &nav_disp_cov, &lc_control_cov] (const Eigen::Index time_it) -> void
  {
    const Eigen::Matrix<SCALAR,DIM_S::CONTROL_DIM,DIM_S::ERROR_DIM,OPTIONS> control_pd_error_state =
      controller->getControlPDWRErrorState(lincov_state_vector(time_it, DIM_S::TIME_IND),
                                           ref_nav_state.row(time_it),
                                           lincov_state_vector.template block<1,DIM_S::REF_DIM>(time_it, DIM_S::REF_START_IND));
    lc_control_cov[time_it] = control_pd_error_state * nav_disp_cov[time_it] * control_pd_error_state.transpose();
  });

  // Plot
  // Get time vector
  std::vector<SCALAR> time(sim_len-1);
  std::for_each(std::execution::par_unseq, time_inds.begin(), time_inds.end(),
  [&time, &lincov_state_vector] (const Eigen::Index time_it) -> void
  {
    time[time_it] = lincov_state_vector(time_it, DIM_S::TIME_IND);
  });

  for(size_t name_it = 0; name_it < names_len; ++name_it)
  {
    std::vector<std::vector<SCALAR>> sub_control(   num_sims);
    std::vector<std::vector<SCALAR>> sub_cl_control(num_sims);

    std::vector<SCALAR> sub_ol_control(               sim_len-1);
    std::vector<SCALAR> sub_avg_control(              sim_len-1);
    std::vector<SCALAR> sub_avg_cl_control(           sim_len-1);
    std::vector<SCALAR> sub_mc_control_three_sigma(   sim_len-1);
    std::vector<SCALAR> sub_mc_cl_control_three_sigma(sim_len-1);
    std::vector<SCALAR> sub_lc_control_three_sigma(   sim_len-1);

    for(size_t sim_it = 0; sim_it < num_sims; ++sim_it)
    {
      sub_control[   sim_it].resize(sim_len-1);
      sub_cl_control[sim_it].resize(sim_len-1);
    }

    // Convert to the form that matplotlib needs
    std::for_each(std::execution::par_unseq, time_inds.begin(), time_inds.end(),
    [&] (const Eigen::Index time_it) -> void
    {
      sub_ol_control[    time_it] = ol_control(    time_it, names[name_it].second);
      sub_avg_control[   time_it] = avg_control(   time_it, names[name_it].second);
      sub_avg_cl_control[time_it] = avg_cl_control(time_it, names[name_it].second);

      sub_mc_control_three_sigma[   time_it] = SCALAR(3) * std::sqrt(mc_control_cov[   time_it](names[name_it].second, names[name_it].second));
      sub_mc_cl_control_three_sigma[time_it] = SCALAR(3) * std::sqrt(mc_cl_control_cov[time_it](names[name_it].second, names[name_it].second));
      sub_lc_control_three_sigma[   time_it] = SCALAR(3) * std::sqrt(lc_control_cov[   time_it](names[name_it].second, names[name_it].second));
      for(size_t sim_it = 0; sim_it < num_sims; ++sim_it)
      {
        sub_control[   sim_it][time_it] = control[   sim_it](time_it, names[name_it].second);
        sub_cl_control[sim_it][time_it] = cl_control[sim_it](time_it, names[name_it].second);
      }
    });

    // Plot the states
    matplotlibcpp::figure();
    for(size_t sim_it = 0; sim_it < num_sims; sim_it += mc_run_down_sample)
    {
      matplotlibcpp::plot<SCALAR>(time, sub_control[sim_it], "y");
    }

    std::vector<SCALAR> temp(sim_len-1);
    std::for_each(std::execution::par_unseq, time_inds.begin(), time_inds.end(),
    [&temp, &sub_mc_control_three_sigma, &sub_avg_control] (const Eigen::Index time_it) -> void
    {
      temp[time_it] = sub_mc_control_three_sigma[time_it] + sub_avg_control[time_it];
    });
    matplotlibcpp::named_plot<SCALAR,SCALAR>("3-sigma MC", time, temp, "r-.");
    std::for_each(std::execution::par_unseq, time_inds.begin(), time_inds.end(),
    [&temp, &sub_mc_control_three_sigma, &sub_avg_control] (const Eigen::Index time_it) -> void
    {
      temp[time_it] = -sub_mc_control_three_sigma[time_it] + sub_avg_control[time_it];
    });
    matplotlibcpp::plot<SCALAR>(time, temp, "r-.");

    std::for_each(std::execution::par_unseq, time_inds.begin(), time_inds.end(),
    [&temp, &sub_lc_control_three_sigma, &sub_ol_control] (const Eigen::Index time_it) -> void
    {
      temp[time_it] = sub_lc_control_three_sigma[time_it] + sub_ol_control[time_it];
    });
    matplotlibcpp::named_plot<SCALAR,SCALAR>("3-sigma LC", time, temp, "b--");
    std::for_each(std::execution::par_unseq, time_inds.begin(), time_inds.end(),
    [&temp, &sub_lc_control_three_sigma, &sub_ol_control] (const Eigen::Index time_it) -> void
    {
      temp[time_it] = -sub_lc_control_three_sigma[time_it] + sub_ol_control[time_it];
    });
    matplotlibcpp::plot<SCALAR>(time, temp, "b--");

    matplotlibcpp::named_plot<SCALAR,SCALAR>("Avg.", time, sub_avg_control, "m");
    matplotlibcpp::named_plot<SCALAR,SCALAR>("Ref.", time, sub_ol_control,  "g");
    matplotlibcpp::xlabel("Time (sec)");
    matplotlibcpp::ylabel(names[name_it].first);
    matplotlibcpp::legend();
    matplotlibcpp::title(names[name_it].first + " Control");

    matplotlibcpp::figure();
    for(size_t sim_it = 0; sim_it < num_sims; sim_it += mc_run_down_sample)
    {
      matplotlibcpp::plot<SCALAR>(time, sub_cl_control[sim_it], "y");
    }
    matplotlibcpp::named_plot<SCALAR,SCALAR>("3-sigma MC", time, sub_mc_cl_control_three_sigma, "r-.");
    std::transform(sub_mc_cl_control_three_sigma.cbegin(), sub_mc_cl_control_three_sigma.cend(), sub_mc_cl_control_three_sigma.begin(), std::negate<SCALAR>());
    matplotlibcpp::plot<SCALAR>(time, sub_mc_cl_control_three_sigma, "r-.");
    matplotlibcpp::named_plot<SCALAR,SCALAR>("3-sigma LC", time, sub_lc_control_three_sigma, "b--");
    std::transform(sub_lc_control_three_sigma.cbegin(), sub_lc_control_three_sigma.cend(), sub_lc_control_three_sigma.begin(), std::negate<SCALAR>());
    matplotlibcpp::plot<SCALAR>(time, sub_lc_control_three_sigma, "b--");
    matplotlibcpp::named_plot<SCALAR,SCALAR>("Avg.", time, sub_avg_cl_control, "g");
    matplotlibcpp::xlabel("Time (sec)");
    matplotlibcpp::ylabel(names[name_it].first);
    matplotlibcpp::legend();
    matplotlibcpp::title(names[name_it].first + " CL Control");
  }

  if(block)
  {
    matplotlibcpp::show();
  }
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
void plot::
  plotAllStatistics(const std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::MC::FULL_STATE_LEN,OPTIONS>>&                 mc_state_vectors_full,
                    const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>&                          lincov_state_vector,
                    const map::MappingsBasePtr<DIM_S,SCALAR,OPTIONS>&                                                          mappings,
                    const std::function<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::TRUTH_DISP_DIM,OPTIONS>(
                                          const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::TRUTH_DIM,OPTIONS>&)>&              truth_to_plot,
                    const std::function<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::ERROR_DIM,OPTIONS>(
                                          const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::NAV_DIM,OPTIONS>&)>&                nav_to_plot,
                    const std::vector<std::pair<std::string,std::tuple<Eigen::Index,Eigen::Index,Eigen::Index,Eigen::Index>>>& names,
                    const std::array<bool,6>&                                                                                  plot_types,
                    const size_t                                                                                               mc_run_down_sample,
                    const bool                                                                                                 block) noexcept
{
  const size_t sim_len   = lincov_state_vector.rows();
  const size_t names_len = names.size();
  const size_t num_sims  = mc_state_vectors_full.size();

  const boost::integer_range<Eigen::Index> time_inds(0, sim_len);
  const boost::integer_range<Eigen::Index> sim_inds (0, num_sims);

  // Calculate information about this problem
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::TRUTH_DIM,     OPTIONS> avg_truth_state;
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::NAV_DIM,       OPTIONS> avg_nav_state;
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::TRUTH_DIM,     OPTIONS> ref_truth_state;
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::NAV_DIM,       OPTIONS> ref_nav_state;
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::TRUTH_DISP_DIM,OPTIONS> avg_truth_state_plot;
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::ERROR_DIM,     OPTIONS> avg_nav_state_plot;

  std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,       DIM_S::TRUTH_DISP_DIM,OPTIONS>> truth;
  std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,       DIM_S::TRUTH_DISP_DIM,OPTIONS>> truth_disp;
  std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,       DIM_S::ERROR_DIM,     OPTIONS>> nav_disp;
  std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,       DIM_S::ERROR_DIM,     OPTIONS>> error_states;
  std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,       DIM_S::TRUTH_DISP_DIM,OPTIONS>> truth_disp_ref;
  std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,       DIM_S::ERROR_DIM,     OPTIONS>> nav_disp_ref;
  std::vector<Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS>> avg_truth_disp_cov;
  std::vector<Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,     DIM_S::ERROR_DIM,     OPTIONS>> avg_nav_disp_cov;
  std::vector<Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,     DIM_S::ERROR_DIM,     OPTIONS>> avg_error_cov;
  std::vector<Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS>> avg_truth_disp_ref_cov;
  std::vector<Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,     DIM_S::ERROR_DIM,     OPTIONS>> avg_nav_disp_ref_cov;
  std::vector<Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS>> truth_disp_cov;
  std::vector<Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,     DIM_S::ERROR_DIM,     OPTIONS>> nav_disp_cov;
  std::vector<Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,     DIM_S::ERROR_DIM,     OPTIONS>> error_cov;

  std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::MC::FULL_STATE_LEN,OPTIONS>> mc_state_vectors(num_sims);
  {
    std::for_each(std::execution::par_unseq, sim_inds.begin(), sim_inds.end(),
    [&mc_state_vectors, &mc_state_vectors_full, sim_len] (const Eigen::Index sim_it) -> void
    {
      mc_state_vectors[sim_it].resize(sim_len, DIM_S::MC::FULL_STATE_LEN);
      mc_state_vectors[sim_it].row(0) = mc_state_vectors_full[sim_it].row(0);
    });
    const Eigen::Index mc_sim_len = mc_state_vectors_full.front().rows();
    size_t mc_time_ind = 0;
    for(size_t lc_time_ind = 1; lc_time_ind < sim_len; ++lc_time_ind)
    {
      // Find corresponding MC index
      for(; mc_time_ind < mc_sim_len; ++mc_time_ind)
      {
        if(lincov_state_vector(lc_time_ind, DIM_S::TIME_IND) <= mc_state_vectors_full.front()(mc_time_ind, DIM_S::TIME_IND))
        {
          break;
        }
      }
      // Copy MC states over
      std::for_each(std::execution::par_unseq, sim_inds.begin(), sim_inds.end(),
      [&mc_state_vectors, &mc_state_vectors_full, lc_time_ind, mc_time_ind] (const Eigen::Index sim_it) -> void
      {
        mc_state_vectors[sim_it].row(lc_time_ind) = mc_state_vectors_full[sim_it].row(mc_time_ind);
      });
    }
  }

  if(plot_types[PlotTypeInds::TRUTH_DISP_PLOTS] or plot_types[PlotTypeInds::STATE_PLOTS])
  { avg_truth_state = math::approxMeanTruthStateTrajectory<DIM_S,SCALAR,OPTIONS>(mc_state_vectors, mappings); }
  if(plot_types[PlotTypeInds::NAV_DISP_PLOTS] or plot_types[PlotTypeInds::STATE_PLOTS])
  { avg_nav_state = math::approxMeanNavStateTrajectory<DIM_S,SCALAR,OPTIONS>(mc_state_vectors, mappings); }

  if(plot_types[PlotTypeInds::TRUTH_DISP_OFF_REF_PLOTS])
  {
    ref_truth_state.resize(sim_len, Eigen::NoChange);
    std::for_each(std::execution::par_unseq, time_inds.begin(), time_inds.end(),
    [&ref_truth_state, &mappings, &lincov_state_vector] (const Eigen::Index time_it) -> void
    {
      ref_truth_state.row(time_it) = mappings->mapRefTruth(lincov_state_vector.template block<1,DIM_S::REF_DIM>(time_it, DIM_S::REF_START_IND));
    });
    truth_disp_ref = math::findTruthStateDispersion<DIM_S,SCALAR,OPTIONS>(mc_state_vectors, ref_truth_state, mappings);
    avg_truth_disp_ref_cov = math::approxStateDispersionCovariance<DIM_S::TRUTH_DISP_DIM,SCALAR,OPTIONS>(truth_disp_ref);
  }
  if(plot_types[PlotTypeInds::NAV_DISP_OFF_REF_PLOTS])
  {
    ref_nav_state.resize(sim_len, Eigen::NoChange);
    std::for_each(std::execution::par_unseq, time_inds.begin(), time_inds.end(),
    [&ref_nav_state, &mappings, &lincov_state_vector] (const Eigen::Index time_it) -> void
    {
      ref_nav_state.row(time_it) = mappings->mapRefNav(lincov_state_vector.template block<1,DIM_S::REF_DIM>(time_it, DIM_S::REF_START_IND));
    });
    nav_disp_ref = math::findNavStateDispersion<DIM_S,SCALAR,OPTIONS>(mc_state_vectors, ref_nav_state, mappings);
    avg_nav_disp_ref_cov = math::approxStateDispersionCovariance<DIM_S::ERROR_DIM,SCALAR,OPTIONS>(nav_disp_ref);
  }
  if(plot_types[PlotTypeInds::TRUTH_DISP_PLOTS])
  {
    truth_disp = math::findTruthStateDispersion<DIM_S,SCALAR,OPTIONS>(mc_state_vectors, avg_truth_state, mappings);
    avg_truth_disp_cov = math::approxStateDispersionCovariance<DIM_S::TRUTH_DISP_DIM,SCALAR,OPTIONS>(truth_disp);
    truth_disp_cov = math::truthStateDispersionCovariance<DIM_S,SCALAR,OPTIONS>(lincov_state_vector);
  }
  if(plot_types[PlotTypeInds::NAV_DISP_PLOTS])
  {
    nav_disp = math::findNavStateDispersion<DIM_S,SCALAR,OPTIONS>(mc_state_vectors, avg_nav_state, mappings);
    avg_nav_disp_cov = math::approxStateDispersionCovariance<DIM_S::ERROR_DIM,SCALAR,OPTIONS>(nav_disp);
    nav_disp_cov = math::navStateDispersionCovariance<DIM_S,SCALAR,OPTIONS>(lincov_state_vector);
  }
  if(plot_types[PlotTypeInds::EST_ERROR_PLOTS])
  {
    error_states = math::findErrorStates<DIM_S,SCALAR,OPTIONS>(mc_state_vectors, mappings);
    avg_error_cov = math::approxStateDispersionCovariance<DIM_S::ERROR_DIM,SCALAR,OPTIONS>(error_states);
    error_cov = math::errorStateCovariance<DIM_S,SCALAR,OPTIONS>(lincov_state_vector, mappings);
  }
  if(plot_types[PlotTypeInds::STATE_PLOTS])
  {
    avg_truth_state_plot = truth_to_plot(avg_truth_state);
    avg_nav_state_plot   = nav_to_plot(  avg_nav_state);

    truth.resize(num_sims);
    std::for_each(std::execution::par_unseq, sim_inds.begin(), sim_inds.end(),
    [&truth, &truth_to_plot, &mc_state_vectors] (const Eigen::Index sim_it) -> void
    {
      truth[sim_it] = truth_to_plot(mc_state_vectors[sim_it].template middleCols<DIM_S::TRUTH_DIM>(DIM_S::MC::TRUTH_START_IND));
    });
  }

  // Plot
  // Get time vector
  std::vector<SCALAR> time(sim_len);
  std::for_each(std::execution::par_unseq, time_inds.begin(), time_inds.end(),
  [&time, &lincov_state_vector] (const Eigen::Index time_it) -> void
  {
    time[time_it] = lincov_state_vector(time_it, DIM_S::TIME_IND);
  });

  for(size_t name_it = 0; name_it < names_len; ++name_it)
  {
    const Eigen::Index num_states      = std::get<0>(names[name_it].second);
    const Eigen::Index ref_start_ind   = std::get<1>(names[name_it].second);
    const Eigen::Index truth_start_ind = std::get<2>(names[name_it].second);
    const Eigen::Index nav_start_ind   = std::get<3>(names[name_it].second);
    const bool         has_ref         = -1 != ref_start_ind;
    const bool         has_truth       = -1 != truth_start_ind;
    const bool         has_nav         = -1 != nav_start_ind;

    std::vector<std::vector<std::vector<SCALAR>>> sub_truth;
    std::vector<std::vector<std::vector<SCALAR>>> sub_truth_disp;
    std::vector<std::vector<std::vector<SCALAR>>> sub_nav_disp;
    std::vector<std::vector<std::vector<SCALAR>>> sub_error;
    std::vector<std::vector<std::vector<SCALAR>>> sub_truth_disp_ref;
    std::vector<std::vector<std::vector<SCALAR>>> sub_nav_disp_ref;

    std::vector<std::vector<SCALAR>> sub_ref;
    std::vector<std::vector<SCALAR>> sub_truth_avg;
    std::vector<std::vector<SCALAR>> sub_nav_avg;

    std::vector<std::vector<SCALAR>> mc_three_sig_truth_disp;
    std::vector<std::vector<SCALAR>> mc_three_sig_nav_disp;
    std::vector<std::vector<SCALAR>> mc_three_sig_error;
    std::vector<std::vector<SCALAR>> mc_three_sig_truth_disp_ref;
    std::vector<std::vector<SCALAR>> mc_three_sig_nav_disp_ref;
    std::vector<std::vector<SCALAR>> lc_three_sig_truth_disp;
    std::vector<std::vector<SCALAR>> lc_three_sig_nav_disp;
    std::vector<std::vector<SCALAR>> lc_three_sig_error;

    if(has_truth and plot_types[PlotTypeInds::TRUTH_DISP_PLOTS])
    {
      sub_truth_disp.         resize(num_states);
      mc_three_sig_truth_disp.resize(num_states);
      lc_three_sig_truth_disp.resize(num_states);
    }
    if(has_nav and plot_types[PlotTypeInds::NAV_DISP_PLOTS])
    {
      sub_nav_disp.         resize(num_states);
      mc_three_sig_nav_disp.resize(num_states);
      lc_three_sig_nav_disp.resize(num_states);
    }
    if(has_nav and plot_types[PlotTypeInds::EST_ERROR_PLOTS])
    {
      sub_error.         resize(num_states);
      mc_three_sig_error.resize(num_states);
      lc_three_sig_error.resize(num_states);
    }
    if(has_truth and plot_types[PlotTypeInds::TRUTH_DISP_OFF_REF_PLOTS])
    {
      sub_truth_disp_ref.         resize(num_states);
      mc_three_sig_truth_disp_ref.resize(num_states);
    }
    if(has_nav and plot_types[PlotTypeInds::NAV_DISP_OFF_REF_PLOTS])
    {
      sub_nav_disp_ref.         resize(num_states);
      mc_three_sig_nav_disp_ref.resize(num_states);
    }
    if(plot_types[PlotTypeInds::STATE_PLOTS])
    {
      if(has_ref)
      { sub_ref.resize(num_states); }
      if(has_truth)
      {
        sub_truth_avg.resize(num_states);
        sub_truth.    resize(num_states);
      }
      if(has_nav)
      { sub_nav_avg.resize(num_states); }
    }

    for(Eigen::Index dim_it = 0; dim_it < num_states; ++dim_it)
    {
      if(has_ref and plot_types[PlotTypeInds::STATE_PLOTS])
      { sub_ref[dim_it].resize(sim_len); }
      if(has_truth and plot_types[PlotTypeInds::STATE_PLOTS])
      {
        sub_truth_avg[dim_it].resize(sim_len);
        sub_truth[    dim_it].resize(num_sims);
      }
      if(has_nav and plot_types[PlotTypeInds::STATE_PLOTS])
      { sub_nav_avg[dim_it].resize(sim_len); }

      if(has_truth and plot_types[PlotTypeInds::TRUTH_DISP_PLOTS])
      {
        sub_truth_disp[         dim_it].resize(num_sims);
        mc_three_sig_truth_disp[dim_it].resize(sim_len);
        lc_three_sig_truth_disp[dim_it].resize(sim_len);
      }
      if(has_nav and plot_types[PlotTypeInds::NAV_DISP_PLOTS])
      {
        sub_nav_disp[         dim_it].resize(num_sims);
        mc_three_sig_nav_disp[dim_it].resize(sim_len);
        lc_three_sig_nav_disp[dim_it].resize(sim_len);
      }
      if(has_nav and plot_types[PlotTypeInds::EST_ERROR_PLOTS])
      {
        sub_error[         dim_it].resize(num_sims);
        mc_three_sig_error[dim_it].resize(sim_len);
        lc_three_sig_error[dim_it].resize(sim_len);
      }
      if(has_truth and plot_types[PlotTypeInds::TRUTH_DISP_OFF_REF_PLOTS])
      {
        sub_truth_disp_ref[         dim_it].resize(num_sims);
        mc_three_sig_truth_disp_ref[dim_it].resize(sim_len);
      }
      if(has_nav and plot_types[PlotTypeInds::NAV_DISP_OFF_REF_PLOTS])
      {
        sub_nav_disp_ref[         dim_it].resize(num_sims);
        mc_three_sig_nav_disp_ref[dim_it].resize(sim_len);
      }
      for(size_t sim_it = 0; sim_it < num_sims; ++sim_it)
      {
        if(has_truth and plot_types[PlotTypeInds::STATE_PLOTS])
        { sub_truth[dim_it][sim_it].resize(sim_len); }
        if(has_truth and plot_types[PlotTypeInds::TRUTH_DISP_PLOTS])
        { sub_truth_disp[dim_it][sim_it].resize(sim_len); }
        if(has_nav and plot_types[PlotTypeInds::NAV_DISP_PLOTS])
        { sub_nav_disp[dim_it][sim_it].resize(sim_len); }
        if(has_nav and plot_types[PlotTypeInds::EST_ERROR_PLOTS])
        { sub_error[dim_it][sim_it].resize(sim_len); }
        if(has_truth and plot_types[PlotTypeInds::TRUTH_DISP_OFF_REF_PLOTS])
        { sub_truth_disp_ref[dim_it][sim_it].resize(sim_len); }
        if(has_nav and plot_types[PlotTypeInds::NAV_DISP_OFF_REF_PLOTS])
        { sub_nav_disp_ref[dim_it][sim_it].resize(sim_len); }
      }
    }

    // Convert to the form that matplotlib needs
    std::for_each(std::execution::par_unseq, time_inds.begin(), time_inds.end(),
    [&] (const Eigen::Index time_it) -> void
    {
      for(Eigen::Index dim_it = 0; dim_it < num_states; ++dim_it)
      {
        if(has_ref and plot_types[PlotTypeInds::STATE_PLOTS])
        { sub_ref[dim_it][time_it] = lincov_state_vector(time_it, DIM_S::REF_START_IND + ref_start_ind + dim_it); }
        if(has_truth and plot_types[PlotTypeInds::STATE_PLOTS])
        { sub_truth_avg[dim_it][time_it] = avg_truth_state_plot(time_it, truth_start_ind + dim_it); }
        if(has_nav and plot_types[PlotTypeInds::STATE_PLOTS])
        { sub_nav_avg[dim_it][time_it] = avg_nav_state_plot(time_it, nav_start_ind + dim_it); }

        for(size_t sim_it = 0; sim_it < num_sims; ++sim_it)
        {
          if(has_truth and plot_types[PlotTypeInds::STATE_PLOTS])
          { sub_truth[dim_it][sim_it][time_it] = truth[sim_it](time_it, truth_start_ind + dim_it); }
          if(has_nav and plot_types[PlotTypeInds::NAV_DISP_PLOTS])
          { sub_nav_disp[dim_it][sim_it][time_it] = nav_disp[sim_it](time_it, nav_start_ind + dim_it); }
          if(has_truth and plot_types[PlotTypeInds::TRUTH_DISP_PLOTS])
          { sub_truth_disp[dim_it][sim_it][time_it] = truth_disp[sim_it](time_it, truth_start_ind + dim_it); }
          if(has_nav and plot_types[PlotTypeInds::EST_ERROR_PLOTS])
          { sub_error[dim_it][sim_it][time_it] = error_states[sim_it](time_it, nav_start_ind + dim_it); }
          if(has_nav and plot_types[PlotTypeInds::NAV_DISP_OFF_REF_PLOTS])
          { sub_nav_disp_ref[dim_it][sim_it][time_it] = nav_disp_ref[sim_it](time_it, nav_start_ind + dim_it); }
          if(has_truth and plot_types[PlotTypeInds::TRUTH_DISP_OFF_REF_PLOTS])
          { sub_truth_disp_ref[dim_it][sim_it][time_it] = truth_disp_ref[sim_it](time_it, truth_start_ind + dim_it); }
        }

        if(has_truth and plot_types[PlotTypeInds::TRUTH_DISP_PLOTS])
        {
          mc_three_sig_truth_disp[dim_it][time_it] = SCALAR(3) * std::sqrt(avg_truth_disp_cov[time_it](truth_start_ind + dim_it, truth_start_ind + dim_it));
          lc_three_sig_truth_disp[dim_it][time_it] = SCALAR(3) * std::sqrt(truth_disp_cov[    time_it](truth_start_ind + dim_it, truth_start_ind + dim_it));
        }
        if(has_nav and plot_types[PlotTypeInds::NAV_DISP_PLOTS])
        {
          mc_three_sig_nav_disp[dim_it][time_it] = SCALAR(3) * std::sqrt(avg_nav_disp_cov[time_it](nav_start_ind + dim_it, nav_start_ind + dim_it));
          lc_three_sig_nav_disp[dim_it][time_it] = SCALAR(3) * std::sqrt(nav_disp_cov[    time_it](nav_start_ind + dim_it, nav_start_ind + dim_it));
        }
        if(has_nav and plot_types[PlotTypeInds::EST_ERROR_PLOTS])
        {
          mc_three_sig_error[dim_it][time_it] = SCALAR(3) * std::sqrt(avg_error_cov[time_it](nav_start_ind + dim_it, nav_start_ind + dim_it));
          lc_three_sig_error[dim_it][time_it] = SCALAR(3) * std::sqrt(error_cov[    time_it](nav_start_ind + dim_it, nav_start_ind + dim_it));
        }
        if(has_truth and plot_types[PlotTypeInds::TRUTH_DISP_OFF_REF_PLOTS])
        { mc_three_sig_truth_disp_ref[dim_it][time_it] = SCALAR(3) * std::sqrt(avg_truth_disp_ref_cov[time_it](truth_start_ind + dim_it, truth_start_ind + dim_it)); }
        if(has_nav and plot_types[PlotTypeInds::NAV_DISP_OFF_REF_PLOTS])
        { mc_three_sig_nav_disp_ref[dim_it][time_it] = SCALAR(3) * std::sqrt(avg_nav_disp_ref_cov[time_it](nav_start_ind + dim_it, nav_start_ind + dim_it)); }
      }
    });

    // Plot the states
    if(has_truth and plot_types[PlotTypeInds::TRUTH_DISP_PLOTS])
    {
      matplotlibcpp::figure();
      for(Eigen::Index dim_it = 0; dim_it < num_states; ++dim_it)
      {
        matplotlibcpp::subplot(num_states, 1, dim_it+1);
        for(size_t sim_it = 0; sim_it < num_sims; sim_it += mc_run_down_sample)
        {
          matplotlibcpp::plot<SCALAR>(time, sub_truth_disp[dim_it][sim_it], "y");
        }
        matplotlibcpp::named_plot<SCALAR,SCALAR>("3-sigma MC", time, mc_three_sig_truth_disp[dim_it], "r-.");
        std::transform(mc_three_sig_truth_disp[dim_it].cbegin(), mc_three_sig_truth_disp[dim_it].cend(), mc_three_sig_truth_disp[dim_it].begin(), std::negate<SCALAR>());
        matplotlibcpp::plot<SCALAR>(time, mc_three_sig_truth_disp[dim_it], "r-.");

        matplotlibcpp::named_plot<SCALAR,SCALAR>("3-sigma LC", time, lc_three_sig_truth_disp[dim_it], "b--");
        std::transform(lc_three_sig_truth_disp[dim_it].cbegin(), lc_three_sig_truth_disp[dim_it].cend(), lc_three_sig_truth_disp[dim_it].begin(), std::negate<SCALAR>());
        matplotlibcpp::plot<SCALAR>(time, lc_three_sig_truth_disp[dim_it], "b--");
        matplotlibcpp::xlabel("Time (sec)");
        switch(dim_it)
        {
          case 0:
            matplotlibcpp::ylabel(names[name_it].first + " X Truth Disp");
            matplotlibcpp::legend();
            matplotlibcpp::title(names[name_it].first + " Truth Dispersion");
            break;
          case 1:
            matplotlibcpp::ylabel(names[name_it].first + " Y Truth Disp");
            break;
          case 2:
            matplotlibcpp::ylabel(names[name_it].first + " Z Truth Disp");
            break;
          default:
            assert(false);
            break;
        };
      }
    }

    if(has_truth and plot_types[PlotTypeInds::TRUTH_DISP_OFF_REF_PLOTS])
    {
      matplotlibcpp::figure();
      for(Eigen::Index dim_it = 0; dim_it < num_states; ++dim_it)
      {
        matplotlibcpp::subplot(num_states, 1, dim_it+1);
        for(size_t sim_it = 0; sim_it < num_sims; sim_it += mc_run_down_sample)
        {
          matplotlibcpp::plot<SCALAR>(time, sub_truth_disp_ref[dim_it][sim_it], "y");
        }
        matplotlibcpp::named_plot<SCALAR,SCALAR>("3-sigma MC", time, mc_three_sig_truth_disp_ref[dim_it], "r-.");
        std::transform(mc_three_sig_truth_disp_ref[dim_it].cbegin(), mc_three_sig_truth_disp_ref[dim_it].cend(), mc_three_sig_truth_disp_ref[dim_it].begin(), std::negate<SCALAR>());
        matplotlibcpp::plot<SCALAR>(time, mc_three_sig_truth_disp_ref[dim_it], "r-.");

        matplotlibcpp::xlabel("Time (sec)");
        switch(dim_it)
        {
          case 0:
            matplotlibcpp::ylabel(names[name_it].first + " X Truth Disp Ref");
            matplotlibcpp::legend();
            matplotlibcpp::title(names[name_it].first + " Truth Dispersion Ref off Reference");
            break;
          case 1:
            matplotlibcpp::ylabel(names[name_it].first + " Y Truth Disp Ref");
            break;
          case 2:
            matplotlibcpp::ylabel(names[name_it].first + " Z Truth Disp Ref");
            break;
          default:
            assert(false);
            break;
        };
      }
    }

    if(has_nav and plot_types[PlotTypeInds::NAV_DISP_PLOTS])
    {
      matplotlibcpp::figure();
      for(Eigen::Index dim_it = 0; dim_it < num_states; ++dim_it)
      {
        matplotlibcpp::subplot(num_states, 1, dim_it+1);
        for(size_t sim_it = 0; sim_it < num_sims; sim_it += mc_run_down_sample)
        {
          matplotlibcpp::plot<SCALAR>(time, sub_nav_disp[dim_it][sim_it], "y");
        }
        matplotlibcpp::named_plot<SCALAR,SCALAR>("3-sigma MC", time, mc_three_sig_nav_disp[dim_it], "r-.");
        std::transform(mc_three_sig_nav_disp[dim_it].cbegin(), mc_three_sig_nav_disp[dim_it].cend(), mc_three_sig_nav_disp[dim_it].begin(), std::negate<SCALAR>());
        matplotlibcpp::plot<SCALAR>(time, mc_three_sig_nav_disp[dim_it], "r-.");

        matplotlibcpp::named_plot<SCALAR,SCALAR>("3-sigma LC", time, lc_three_sig_nav_disp[dim_it], "b--");
        std::transform(lc_three_sig_nav_disp[dim_it].cbegin(), lc_three_sig_nav_disp[dim_it].cend(), lc_three_sig_nav_disp[dim_it].begin(), std::negate<SCALAR>());
        matplotlibcpp::plot<SCALAR>(time, lc_three_sig_nav_disp[dim_it], "b--");
        matplotlibcpp::xlabel("Time (sec)");
        switch(dim_it)
        {
          case 0:
            matplotlibcpp::ylabel(names[name_it].first + " X Nav Disp");
            matplotlibcpp::legend();
            matplotlibcpp::title(names[name_it].first + " Navigation Dispersion");
            break;
          case 1:
            matplotlibcpp::ylabel(names[name_it].first + " Y Nav Disp");
            break;
          case 2:
            matplotlibcpp::ylabel(names[name_it].first + " Z Nav Disp");
            break;
          default:
            assert(false);
            break;
        };
      }
    }

    if(has_nav and plot_types[PlotTypeInds::NAV_DISP_OFF_REF_PLOTS])
    {
      matplotlibcpp::figure();
      for(Eigen::Index dim_it = 0; dim_it < num_states; ++dim_it)
      {
        matplotlibcpp::subplot(num_states, 1, dim_it+1);
        for(size_t sim_it = 0; sim_it < num_sims; sim_it += mc_run_down_sample)
        {
          matplotlibcpp::plot<SCALAR>(time, sub_nav_disp_ref[dim_it][sim_it], "y");
        }
        matplotlibcpp::named_plot<SCALAR,SCALAR>("3-sigma MC", time, mc_three_sig_nav_disp_ref[dim_it], "r-.");
        std::transform(mc_three_sig_nav_disp_ref[dim_it].cbegin(), mc_three_sig_nav_disp_ref[dim_it].cend(), mc_three_sig_nav_disp_ref[dim_it].begin(), std::negate<SCALAR>());
        matplotlibcpp::plot<SCALAR>(time, mc_three_sig_nav_disp_ref[dim_it], "r-.");

        matplotlibcpp::xlabel("Time (sec)");
        switch(dim_it)
        {
          case 0:
            matplotlibcpp::ylabel(names[name_it].first + " X Nav Disp Ref");
            matplotlibcpp::legend();
            matplotlibcpp::title(names[name_it].first + " Navigation Dispersion off Reference");
            break;
          case 1:
            matplotlibcpp::ylabel(names[name_it].first + " Y Nav Disp Ref");
            break;
          case 2:
            matplotlibcpp::ylabel(names[name_it].first + " Z Nav Disp Ref");
            break;
          default:
            assert(false);
            break;
        };
      }
    }

    if(has_nav and plot_types[PlotTypeInds::EST_ERROR_PLOTS])
    {
      matplotlibcpp::figure();
      for(Eigen::Index dim_it = 0; dim_it < num_states; ++dim_it)
      {
        matplotlibcpp::subplot(num_states, 1, dim_it+1);
        for(size_t sim_it = 0; sim_it < num_sims; sim_it += mc_run_down_sample)
        {
          matplotlibcpp::plot<SCALAR>(time, sub_error[dim_it][sim_it], "y");
        }
        matplotlibcpp::named_plot<SCALAR,SCALAR>("3-sigma MC", time, mc_three_sig_error[dim_it], "r-.");
        std::transform(mc_three_sig_error[dim_it].cbegin(), mc_three_sig_error[dim_it].cend(), mc_three_sig_error[dim_it].begin(), std::negate<SCALAR>());
        matplotlibcpp::plot<SCALAR,SCALAR>(time, mc_three_sig_error[dim_it], "r-.");

        matplotlibcpp::named_plot<SCALAR,SCALAR>("3-sigma LC", time, lc_three_sig_error[dim_it], "b--");
        std::transform(lc_three_sig_error[dim_it].cbegin(), lc_three_sig_error[dim_it].cend(), lc_three_sig_error[dim_it].begin(), std::negate<SCALAR>());
        matplotlibcpp::plot<SCALAR>(time, lc_three_sig_error[dim_it], "b--");
        matplotlibcpp::xlabel("Time (sec)");
        switch(dim_it)
        {
          case 0:
            matplotlibcpp::ylabel(names[name_it].first + " X Est. Error");
            matplotlibcpp::legend();
            matplotlibcpp::title(names[name_it].first + " Est. Error");
            break;
          case 1:
            matplotlibcpp::ylabel(names[name_it].first + " Y Est. Error");
            break;
          case 2:
            matplotlibcpp::ylabel(names[name_it].first + " Z Est. Error");
            break;
          default:
            assert(false);
            break;
        };
      }
    }

    if(plot_types[PlotTypeInds::STATE_PLOTS])
    {
      matplotlibcpp::figure();
      for(Eigen::Index dim_it = 0; dim_it < num_states; ++dim_it)
      {
        matplotlibcpp::subplot(num_states, 1, dim_it+1);
        if(has_truth)
        {
          for(size_t sim_it = 0; sim_it < num_sims; sim_it += mc_run_down_sample)
          {
            matplotlibcpp::plot<SCALAR>(time, sub_truth[dim_it][sim_it], "y");
          }
        }
        if(has_ref)
        { matplotlibcpp::named_plot<SCALAR,SCALAR>("Reference", time, sub_ref[dim_it], "g"); }
        if(has_nav)
        { matplotlibcpp::named_plot<SCALAR,SCALAR>("Avg. Navigation", time, sub_nav_avg[dim_it], "r"); }
        if(has_truth)
        { matplotlibcpp::named_plot<SCALAR,SCALAR>("Avg. Truth", time, sub_truth_avg[dim_it], "b"); }
        matplotlibcpp::xlabel("Time (sec)");
        switch(dim_it)
        {
          case 0:
            matplotlibcpp::ylabel(names[name_it].first + " Avg. X");
            matplotlibcpp::legend();
            matplotlibcpp::title(names[name_it].first);
            break;
          case 1:
            matplotlibcpp::ylabel(names[name_it].first + " Avg. Y");
            break;
          case 2:
            matplotlibcpp::ylabel(names[name_it].first + " Avg. Z");
            break;
          default:
            assert(false);
            break;
        };
      }
    }
  }

  if(block)
  {
    matplotlibcpp::show();
  }
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
void plot::
  plotAllStatistics(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>&                          lincov_state_vector,
                    const map::MappingsBasePtr<DIM_S,SCALAR,OPTIONS>&                                                          mappings,
                    const std::vector<std::pair<std::string,std::tuple<Eigen::Index,Eigen::Index,Eigen::Index,Eigen::Index>>>& names,
                    const std::array<bool,4>&                                                                                  plot_types,
                    const bool                                                                                                 block) noexcept
{
  const size_t sim_len = lincov_state_vector.rows();

  const boost::integer_range<Eigen::Index> time_inds(0, sim_len);

  // Calculate information about this problem
  std::vector<Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_DISP_DIM,OPTIONS>> truth_disp_cov;
  std::vector<Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,     DIM_S::ERROR_DIM,     OPTIONS>> nav_disp_cov;
  std::vector<Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,     DIM_S::ERROR_DIM,     OPTIONS>> error_cov;

  if(plot_types[PlotTypeInds::TRUTH_DISP_PLOTS])
  { truth_disp_cov = math::truthStateDispersionCovariance<DIM_S,SCALAR,OPTIONS>(lincov_state_vector); }
  if(plot_types[PlotTypeInds::NAV_DISP_PLOTS])
  { nav_disp_cov = math::navStateDispersionCovariance<DIM_S,SCALAR,OPTIONS>(lincov_state_vector); }
  if(plot_types[PlotTypeInds::EST_ERROR_PLOTS])
  { error_cov = math::errorStateCovariance<DIM_S,SCALAR,OPTIONS>(lincov_state_vector, mappings); }

  // Plot
  const size_t names_len = names.size();
  // Get time vector
  std::vector<SCALAR> time(sim_len);
  std::for_each(std::execution::par_unseq, time_inds.begin(), time_inds.end(),
  [&time, &lincov_state_vector] (const Eigen::Index time_it) -> void
  {
    time[time_it] = lincov_state_vector(time_it, DIM_S::TIME_IND);
  });

  for(size_t name_it = 0; name_it < names_len; ++name_it)
  {
    const Eigen::Index num_states      = std::get<0>(names[name_it].second);
    const Eigen::Index ref_start_ind   = std::get<1>(names[name_it].second);
    const Eigen::Index truth_start_ind = std::get<2>(names[name_it].second);
    const Eigen::Index nav_start_ind   = std::get<3>(names[name_it].second);
    const bool         has_ref         = -1 != ref_start_ind;

    std::vector<std::vector<SCALAR>> sub_ref;

    std::vector<std::vector<SCALAR>> lc_three_sig_truth_disp;
    std::vector<std::vector<SCALAR>> lc_three_sig_nav_disp;
    std::vector<std::vector<SCALAR>> lc_three_sig_error;

    if(plot_types[PlotTypeInds::TRUTH_DISP_PLOTS])
    { lc_three_sig_truth_disp.resize(num_states); }
    if(plot_types[PlotTypeInds::NAV_DISP_PLOTS])
    { lc_three_sig_nav_disp.resize(num_states); }
    if(plot_types[PlotTypeInds::EST_ERROR_PLOTS])
    { lc_three_sig_error.resize(num_states); }
    if(has_ref and plot_types[PlotTypeInds::STATE_PLOTS])
    { sub_ref.resize(num_states); }

    for(Eigen::Index dim_it = 0; dim_it < num_states; ++dim_it)
    {
      if(has_ref and plot_types[PlotTypeInds::STATE_PLOTS])
      { sub_ref[dim_it].resize(sim_len); }

      if(plot_types[PlotTypeInds::TRUTH_DISP_PLOTS])
      { lc_three_sig_truth_disp[dim_it].resize(sim_len); }
      if(plot_types[PlotTypeInds::NAV_DISP_PLOTS])
      { lc_three_sig_nav_disp[dim_it].resize(sim_len); }
      if(plot_types[PlotTypeInds::EST_ERROR_PLOTS])
      { lc_three_sig_error[dim_it].resize(sim_len); }
    }

    // Convert to the form that matplotlib needs
    std::for_each(std::execution::par_unseq, time_inds.begin(), time_inds.end(),
    [&] (const Eigen::Index time_it) -> void
    {
      for(Eigen::Index dim_it = 0; dim_it < num_states; ++dim_it)
      {
        if(has_ref and plot_types[PlotTypeInds::STATE_PLOTS])
        { sub_ref[dim_it][time_it] = lincov_state_vector(time_it, DIM_S::REF_START_IND + ref_start_ind + dim_it); }

        if(plot_types[PlotTypeInds::TRUTH_DISP_PLOTS])
        { lc_three_sig_truth_disp[dim_it][time_it] = SCALAR(3) * std::sqrt(truth_disp_cov[time_it](truth_start_ind + dim_it, truth_start_ind + dim_it)); }
        if(plot_types[PlotTypeInds::NAV_DISP_PLOTS])
        { lc_three_sig_nav_disp[dim_it][time_it] = SCALAR(3) * std::sqrt(nav_disp_cov[time_it](nav_start_ind + dim_it, nav_start_ind + dim_it)); }
        if(plot_types[PlotTypeInds::EST_ERROR_PLOTS])
        { lc_three_sig_error[dim_it][time_it] = SCALAR(3) * std::sqrt(error_cov[time_it](nav_start_ind + dim_it, nav_start_ind + dim_it)); }
      }
    });

    // Plot the states
    if(plot_types[PlotTypeInds::TRUTH_DISP_PLOTS])
    {
      matplotlibcpp::figure();
      for(Eigen::Index dim_it = 0; dim_it < num_states; ++dim_it)
      {
        matplotlibcpp::subplot(num_states, 1, dim_it+1);
        matplotlibcpp::named_plot<SCALAR,SCALAR>("3-sigma LC", time, lc_three_sig_truth_disp[dim_it], "b--");
        std::transform(lc_three_sig_truth_disp[dim_it].cbegin(), lc_three_sig_truth_disp[dim_it].cend(), lc_three_sig_truth_disp[dim_it].begin(), std::negate<SCALAR>());
        matplotlibcpp::plot<SCALAR>(time, lc_three_sig_truth_disp[dim_it], "b--");
        matplotlibcpp::xlabel("Time (sec)");
        switch(dim_it)
        {
          case 0:
            matplotlibcpp::ylabel(names[name_it].first + " X Truth Disp");
            matplotlibcpp::legend();
            matplotlibcpp::title(names[name_it].first + " Truth Dispersion");
            break;
          case 1:
            matplotlibcpp::ylabel(names[name_it].first + " Y Truth Disp");
            break;
          case 2:
            matplotlibcpp::ylabel(names[name_it].first + " Z Truth Disp");
            break;
          default:
            assert(false);
            break;
        };
      }
    }

    if(plot_types[PlotTypeInds::NAV_DISP_PLOTS])
    {
      matplotlibcpp::figure();
      for(Eigen::Index dim_it = 0; dim_it < num_states; ++dim_it)
      {
        matplotlibcpp::subplot(num_states, 1, dim_it+1);
        matplotlibcpp::named_plot<SCALAR,SCALAR>("3-sigma LC", time, lc_three_sig_nav_disp[dim_it], "b--");
        std::transform(lc_three_sig_nav_disp[dim_it].cbegin(), lc_three_sig_nav_disp[dim_it].cend(), lc_three_sig_nav_disp[dim_it].begin(), std::negate<SCALAR>());
        matplotlibcpp::plot<SCALAR>(time, lc_three_sig_nav_disp[dim_it], "b--");
        matplotlibcpp::xlabel("Time (sec)");
        switch(dim_it)
        {
          case 0:
            matplotlibcpp::ylabel(names[name_it].first + " X Nav Disp");
            matplotlibcpp::legend();
            matplotlibcpp::title(names[name_it].first + " Navigation Dispersion");
            break;
          case 1:
            matplotlibcpp::ylabel(names[name_it].first + " Y Nav Disp");
            break;
          case 2:
            matplotlibcpp::ylabel(names[name_it].first + " Z Nav Disp");
            break;
          default:
            assert(false);
            break;
        };
      }
    }

    if(plot_types[PlotTypeInds::EST_ERROR_PLOTS])
    {
      matplotlibcpp::figure();
      for(Eigen::Index dim_it = 0; dim_it < num_states; ++dim_it)
      {
        matplotlibcpp::subplot(num_states, 1, dim_it+1);
        matplotlibcpp::named_plot<SCALAR,SCALAR>("3-sigma LC", time, lc_three_sig_error[dim_it], "b--");
        std::transform(lc_three_sig_error[dim_it].cbegin(), lc_three_sig_error[dim_it].cend(), lc_three_sig_error[dim_it].begin(), std::negate<SCALAR>());
        matplotlibcpp::plot<SCALAR>(time, lc_three_sig_error[dim_it], "b--");
        matplotlibcpp::xlabel("Time (sec)");
        switch(dim_it)
        {
          case 0:
            matplotlibcpp::ylabel(names[name_it].first + " X Est. Error");
            matplotlibcpp::legend();
            matplotlibcpp::title(names[name_it].first + " Est. Error");
            break;
          case 1:
            matplotlibcpp::ylabel(names[name_it].first + " Y Est. Error");
            break;
          case 2:
            matplotlibcpp::ylabel(names[name_it].first + " Z Est. Error");
            break;
          default:
            assert(false);
            break;
        };
      }
    }

    if(has_ref and plot_types[PlotTypeInds::STATE_PLOTS])
    {
      matplotlibcpp::figure();
      for(Eigen::Index dim_it = 0; dim_it < num_states; ++dim_it)
      {
        matplotlibcpp::subplot(num_states, 1, dim_it+1);
        matplotlibcpp::named_plot<SCALAR,SCALAR>("Reference", time, sub_ref[dim_it], "g");
        matplotlibcpp::xlabel("Time (sec)");
        switch(dim_it)
        {
          case 0:
            matplotlibcpp::ylabel(names[name_it].first + " Avg. X");
            matplotlibcpp::legend();
            matplotlibcpp::title(names[name_it].first);
            break;
          case 1:
            matplotlibcpp::ylabel(names[name_it].first + " Avg. Y");
            break;
          case 2:
            matplotlibcpp::ylabel(names[name_it].first + " Avg. Z");
            break;
          default:
            assert(false);
            break;
        };
      }
    }
  }

  if(block)
  {
    matplotlibcpp::show();
  }
}

template<typename DIM_S, Versions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
void plot::
  plotErrorBudget(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::REF_DIM,OPTIONS>>&   reference_trajectory,
                  const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& nominal_initial_state,
                  const Tools<DIM_S,SCALAR,OPTIONS>&                                                     tools,
                  const std::list<GeneralNoiseWrapper<SCALAR,OPTIONS>>&                                  noise_sources,
                  const DATA_FUNC<DIM_S,SCALAR,OPTIONS>&                                                 data_extraction_func,
                  const std::string&                                                                     data_name,
                  const bool                                                                             block) noexcept
{
  const Eigen::Index                                                         traj_length = reference_trajectory.rows();
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS> state_vector;
  std::vector<SCALAR>                                                        time_vec;
  Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS>                             plotting_data;
  std::vector<SCALAR>                                                        plotting_data_vec;

  /// All sources enabled
  std::for_each(noise_sources.cbegin(), noise_sources.cend(),
                [](const GeneralNoiseWrapper<SCALAR,OPTIONS>& it) -> void { helpers::setNoiseEnabled<SCALAR,OPTIONS>(it); });
  // Set up state vector
  state_vector.resize(traj_length, Eigen::NoChange);
  state_vector.template topRows<1>()                                     = nominal_initial_state;
  state_vector.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND) = reference_trajectory;
  // Run lincov
  runLinCov<DIM_S,VERSION,SCALAR,OPTIONS>(state_vector, tools);
  // Extract plotting data
  plotting_data = data_extraction_func(state_vector);
  // Convert to std
  time_vec.resize(traj_length);
  plotting_data_vec.resize(traj_length);
  for(Eigen::Index row_it = 0; row_it < traj_length; ++row_it)
  {
    time_vec[row_it]          = state_vector(row_it, DIM_S::TIME_IND);
    plotting_data_vec[row_it] = plotting_data[row_it];
  }
  // Plot the data
  matplotlibcpp::figure();
  matplotlibcpp::named_plot<SCALAR,SCALAR>("Total", time_vec, plotting_data_vec);

  /// One source enabled at a time
  std::for_each(noise_sources.cbegin(), noise_sources.cend(),
                [](const GeneralNoiseWrapper<SCALAR,OPTIONS>& it) -> void { helpers::setNoiseDisabled<SCALAR,OPTIONS>(it); });
  // Contributions from initial covariance conditions
  state_vector.template topRows<1>().template middleCols<DIM_S::LINCOV::AUG_COV_LEN>(DIM_S::LINCOV::AUG_COV_START_IND).setZero();

  const auto noise_sources_end = noise_sources.cend();
  for(auto noise_source_it = noise_sources.cbegin(); noise_source_it != noise_sources_end; ++noise_source_it)
  {
    helpers::setNoiseEnabled<SCALAR,OPTIONS>(*noise_source_it);
    // Run lincov
    runLinCov<DIM_S,Versions(VERSION bitor Versions::RUNNING_ERROR_BUDGET),SCALAR,OPTIONS>(state_vector, tools);
    // Extract plotting data
    plotting_data = data_extraction_func(state_vector);
    // Convert to std
    for(Eigen::Index row_it = 0; row_it < traj_length; ++row_it)
    {
      plotting_data_vec[row_it] = plotting_data[row_it];
    }
    // Plot the data
    matplotlibcpp::named_plot<SCALAR,SCALAR>(helpers::getNoiseName<SCALAR,OPTIONS>(*noise_source_it),
                                             time_vec,
                                             plotting_data_vec);
    helpers::setNoiseDisabled<SCALAR,OPTIONS>(*noise_source_it);
  }
  /// Contributions from initial covariance conditions
  state_vector.template topRows<1>().template middleCols<DIM_S::LINCOV::AUG_COV_LEN>(DIM_S::LINCOV::AUG_COV_START_IND) =
    nominal_initial_state.template middleCols<DIM_S::LINCOV::AUG_COV_LEN>(DIM_S::LINCOV::AUG_COV_START_IND);
  // Run lincov
  runLinCov<DIM_S,Versions(VERSION bitor Versions::RUNNING_ERROR_BUDGET),SCALAR,OPTIONS>(state_vector, tools);
  // Extract plotting data
  plotting_data = data_extraction_func(state_vector);
  // Convert to std
  for(Eigen::Index row_it = 0; row_it < traj_length; ++row_it)
  {
    plotting_data_vec[row_it] = plotting_data[row_it];
  }
  // Plot the data
  matplotlibcpp::named_plot<SCALAR,SCALAR>("Initial Truth State Variance", time_vec, plotting_data_vec);

  /// Legends and axis labels
  matplotlibcpp::title("Error Budget");
  matplotlibcpp::xlabel("Time (sec)");
  matplotlibcpp::ylabel(data_name);
  matplotlibcpp::legend();

  /// Turn all noise back on
  std::for_each(noise_sources.cbegin(), noise_sources.cend(),
                [](const GeneralNoiseWrapper<SCALAR,OPTIONS>& it) -> void { helpers::setNoiseEnabled<SCALAR,OPTIONS>(it); });

  if(block)
  {
    matplotlibcpp::show();
  }
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void plot::helpers::setNoiseEnabled(const GeneralNoiseWrapper<SCALAR,OPTIONS>& noise) noexcept
{
  switch(noise.index())
  {
    case size_t(0):
      std::get<0>(noise)->setEnabled();
      break;
    case size_t(1):
      std::get<1>(noise)->setEnabled();
      break;
    default:
      assert(false);
      break;
  };
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void plot::helpers::setNoiseDisabled(const GeneralNoiseWrapper<SCALAR,OPTIONS>& noise) noexcept
{
  switch(noise.index())
  {
    case size_t(0):
      std::get<0>(noise)->setDisabled();
      break;
    case size_t(1):
      std::get<1>(noise)->setDisabled();
      break;
    default:
      assert(false);
      break;
  };
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline const std::string& plot::helpers::getNoiseName(const GeneralNoiseWrapper<SCALAR,OPTIONS>& noise) noexcept
{
  switch(noise.index())
  {
    case size_t(0):
      return std::get<0>(noise)->getName();
      break;
    case size_t(1):
      return std::get<1>(noise)->getName();
      break;
    default:
      assert(false);
      break;
  };
  assert(false);
  return std::get<0>(noise)->getName();
}
} // namespace kf

#endif
/* plot_statistics.hpp */
