/**
 * @File: plot_statistics_tools.hpp
 * @Date: January 2023
 * @Author: Daren Swasey
 *
 * @brief
 * Defines helper functions for extracting and plotting performance related information from the outputs of
 * Monte Carlo and LinCov runs.
 **/

#ifndef KALMAN_FILTER_HELPERS_PLOT_STATISTICS_TOOLS_HPP
#define KALMAN_FILTER_HELPERS_PLOT_STATISTICS_TOOLS_HPP

/* C++ Headers */
#include<vector>
#include<string>

/* Eigen Headers */
#include<Eigen/Dense>

namespace kf
{
namespace plottools
{
/**
 * @PlotTypeInds
 *
 * @brief
 * The indexes that correspond to each type of plot in the plot_types array.
 **/
struct PlotTypeInds
{
public:
  inline static constexpr const Eigen::Index STATE_PLOTS              = 0;
  inline static constexpr const Eigen::Index TRUTH_DISP_PLOTS         = 1;
  inline static constexpr const Eigen::Index NAV_DISP_PLOTS           = 2;
  inline static constexpr const Eigen::Index EST_ERROR_PLOTS          = 3;
  inline static constexpr const Eigen::Index TRUTH_DISP_OFF_REF_PLOTS = 4;
  inline static constexpr const Eigen::Index NAV_DISP_OFF_REF_PLOTS   = 5;
};
}  // namespace plottools
}  // namespace kf
#endif  // KALMAN_FILTER__HELPERS__PLOT_STATISTICS_TOOLS_HPP_
