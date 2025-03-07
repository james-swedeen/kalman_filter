/**
 * @File: feature_range_test.cpp
 * @Date: May 2024
 * @Author: James Swedeen
 **/

/* Testing Headers */
#include<gtest/gtest.h>

/* Boost Headers */
#include<boost/math/differentiation/finite_difference.hpp>

/* Local Headers */
#include<kalman_filter/helpers/dimension_struct.hpp>
#include<kalman_filter/math/quaternion.hpp>
#include<kalman_filter/sensors/measurements/feature_range.hpp>

using FLOAT = double;
//using FLOAT = long double;

// Testing class
class FeatureRangeTest : public ::testing::Test
{
public:
  struct FeatureDim
   : public kf::Dimensions<0,3,3,0,3,3,0,0,0,0,false>
  {
  public:
    struct TRUTH
    {
    public:
      inline static constexpr const Eigen::Index NORTH_IND     = 0;
      inline static constexpr const Eigen::Index EAST_IND      = 1;
      inline static constexpr const Eigen::Index DOWN_IND      = 2;

      inline static constexpr const Eigen::Index POS_START_IND = NORTH_IND;
      inline static constexpr const Eigen::Index POS_END_IND   = DOWN_IND;
    };
    struct NAV
    {
    public:
      inline static constexpr const Eigen::Index NORTH_IND     = 0;
      inline static constexpr const Eigen::Index EAST_IND      = 1;
      inline static constexpr const Eigen::Index DOWN_IND      = 2;

      inline static constexpr const Eigen::Index POS_START_IND = NORTH_IND;
      inline static constexpr const Eigen::Index POS_END_IND   = DOWN_IND;
    };
    struct ERROR
    {
    public:
      inline static constexpr const Eigen::Index NORTH_IND     = 0;
      inline static constexpr const Eigen::Index EAST_IND      = 1;
      inline static constexpr const Eigen::Index DOWN_IND      = 2;

      inline static constexpr const Eigen::Index POS_START_IND = NORTH_IND;
      inline static constexpr const Eigen::Index POS_END_IND   = DOWN_IND;
    };
    struct TRUTH_DISP
    {
    public:
      inline static constexpr const Eigen::Index NORTH_IND     = 0;
      inline static constexpr const Eigen::Index EAST_IND      = 1;
      inline static constexpr const Eigen::Index DOWN_IND      = 2;

      inline static constexpr const Eigen::Index POS_START_IND = NORTH_IND;
      inline static constexpr const Eigen::Index POS_END_IND   = DOWN_IND;
    };
  };

  inline static constexpr const FLOAT  output_error_eps   = 1.0e-10;
  inline static constexpr const size_t numeric_diff_order = 4;

  inline static constexpr const FLOAT step_size_linear        = 10;
  inline static constexpr const FLOAT uav_position_bounds     = 100;
  inline static constexpr const FLOAT feature_position_bounds = 1;

  FeatureRangeTest()
  {}
};

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

TEST_F(FeatureRangeTest, getMeasurementEstimatePDWRErrorState)
{
  Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> feature_loc;

  for(feature_loc[0] = -this->feature_position_bounds; feature_loc[0] < this->feature_position_bounds; feature_loc[0] += this->step_size_linear)
  {
  for(feature_loc[1] = -this->feature_position_bounds; feature_loc[1] < this->feature_position_bounds; feature_loc[1] += this->step_size_linear)
  {
  //for(feature_loc[2] = -this->feature_position_bounds; feature_loc[2] < this->feature_position_bounds; feature_loc[2] += this->step_size_linear)
  feature_loc[2] = 0;
  {
    kf::sensors::FeatureRange<FeatureDim,true,false,FLOAT,Eigen::RowMajor> test_model(
      std::numeric_limits<FLOAT>::quiet_NaN(),
      std::numeric_limits<FLOAT>::quiet_NaN(),
      feature_loc);

    Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> nav_state;

    for(nav_state[FeatureDim::NAV::NORTH_IND] = -this->uav_position_bounds; nav_state[FeatureDim::NAV::NORTH_IND] < this->uav_position_bounds; nav_state[FeatureDim::NAV::NORTH_IND] += this->step_size_linear)
    {
    for(nav_state[FeatureDim::NAV::EAST_IND] = -this->uav_position_bounds; nav_state[FeatureDim::NAV::EAST_IND] < this->uav_position_bounds; nav_state[FeatureDim::NAV::EAST_IND] += this->step_size_linear)
    {
//    for(nav_state[FeatureDim::NAV::DOWN_IND] = -this->uav_position_bounds; nav_state[FeatureDim::NAV::DOWN_IND] < this->uav_position_bounds; nav_state[FeatureDim::NAV::DOWN_IND] += this->step_size_linear)
    nav_state[FeatureDim::NAV::DOWN_IND] = -500;
    {
      Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> numeric_jacobian;
      Eigen::Index nav_state_dim_it;
      Eigen::Index reading_dim_it;

      const std::function<FLOAT(const FLOAT)> dim_func =
        [&nav_state,&nav_state_dim_it,&reading_dim_it,&test_model] (const FLOAT x) -> FLOAT
        {
          Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> input_nav_state = nav_state;
          input_nav_state[nav_state_dim_it] = x;
          return test_model.estimateMeasurement(0, input_nav_state)[reading_dim_it];
        };

      for(nav_state_dim_it = 0; nav_state_dim_it < 3; ++nav_state_dim_it)
      {
      for(reading_dim_it = 0; reading_dim_it < 1; ++reading_dim_it)
      {
        FLOAT input_val;
        input_val = nav_state[nav_state_dim_it];
        numeric_jacobian(reading_dim_it,nav_state_dim_it) = boost::math::differentiation::finite_difference_derivative<const std::function<FLOAT(FLOAT)>,FLOAT,numeric_diff_order>(
          dim_func, input_val);
      }
      }

      const Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> analytic_jacobian = test_model.getMeasurementEstimatePDWRErrorState(0, nav_state);

      const bool is_equal = ((numeric_jacobian - analytic_jacobian).array().abs() < this->output_error_eps).all();
      EXPECT_TRUE(is_equal);
      if(not is_equal)
      {
        std::cout << "numeric_jacobian:\n" << numeric_jacobian << std::endl;
        std::cout << "analytic_jacobian:\n" << analytic_jacobian << std::endl;
        std::cout << "diff:\n" << numeric_jacobian - analytic_jacobian << std::endl;
        std::cout << "measurement:\n" << test_model.estimateMeasurement(0, nav_state) << std::endl;
        std::cout << "nav_state:\n" << nav_state << std::endl;
        std::cout << "feature_location:\n" << feature_loc << std::endl;
      }
    }
    }
    }
  }
  }
  }
}

/* feature_bearing_test.cpp */
