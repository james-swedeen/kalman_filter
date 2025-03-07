/**
 * @File: differential_pressure_test.cpp
 * @Date: February 2024
 * @Author: James Swedeen
 **/

/* Testing Headers */
#include<gtest/gtest.h>

/* Boost Headers */
#include<boost/math/differentiation/finite_difference.hpp>

/* Local Headers */
#include<kalman_filter/dynamics/dubins_airplane_model.hpp>
#include<kalman_filter/math/quaternion.hpp>
#include<kalman_filter/sensors/measurements/differential_pressure.hpp>

using FLOAT = double;
//using FLOAT = long double;
using DIM = kf::dynamics::DubinsAirplaneDim;

// Testing class
class DifferentialPressureTest : public ::testing::Test
{
public:
  inline static constexpr const FLOAT  output_error_eps   = 0.0000001;
  inline static constexpr const size_t numeric_diff_order = 4;

  inline static constexpr const FLOAT step_size_linear        = 10;
  inline static constexpr const FLOAT step_size_angular       = kf::math::pi<FLOAT>()/FLOAT(32);
  inline static constexpr const FLOAT uav_position_bounds     = 100;

  DifferentialPressureTest()
  {
  }
};

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

TEST_F(DifferentialPressureTest, getMeasurementEstimatePDWRErrorState)
{
  kf::sensors::DifferentialPressure<DIM,true,false,FLOAT,Eigen::RowMajor> test_model(
    std::numeric_limits<FLOAT>::quiet_NaN(),
    4.0);

  Eigen::Matrix<FLOAT,1,DIM::NAV_DIM,Eigen::RowMajor> nav_state;
  nav_state.setOnes();
  nav_state.array() *= -10;

  for(Eigen::Index state_it = 0; state_it < DIM::NAV_DIM; ++state_it)
  {
    for(; nav_state[state_it] < 10; nav_state[state_it] += 1)
    {
      Eigen::Matrix<FLOAT,1,DIM::ERROR_DIM,Eigen::RowMajor> numeric_jacobian;
      Eigen::Index nav_state_dim_it;

      const std::function<FLOAT(const FLOAT)> dim_func =
        [&nav_state,&nav_state_dim_it,&test_model] (const FLOAT x) -> FLOAT
        {
          // Set input
        Eigen::Matrix<FLOAT,1,DIM::NAV_DIM,Eigen::RowMajor> input = nav_state;
        if((nav_state_dim_it >= DIM::ERROR::EULER_START_IND) and (nav_state_dim_it <= DIM::ERROR::EULER_END_IND))
        {
          Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> rpy = kf::math::quat::quaternionToRollPitchYaw(input.middleCols<4>(DIM::NAV::QUAT_START_IND));
          rpy[nav_state_dim_it-DIM::ERROR::EULER_START_IND] = x;
          input.middleCols<4>(DIM::NAV::QUAT_START_IND) = kf::math::quat::normalize(kf::math::quat::rollPitchYawToQuaternion(rpy));
        }
        else
        {
          if(nav_state_dim_it >= DIM::ERROR::EULER_START_IND)
          {
            input[nav_state_dim_it+1] = x;
          }
          else
          {
            input[nav_state_dim_it] = x;
          }
        }
          return test_model.estimateMeasurement(0, input)[0];
        };

      for(nav_state_dim_it = 0; nav_state_dim_it < DIM::ERROR_DIM; ++nav_state_dim_it)
      {
        FLOAT input_val;
        if((nav_state_dim_it >= DIM::ERROR::EULER_START_IND) and (nav_state_dim_it <= DIM::ERROR::EULER_END_IND))
        {
          const Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> rpy = kf::math::quat::quaternionToRollPitchYaw(nav_state.middleCols<4>(DIM::NAV::QUAT_START_IND));
          input_val = rpy[nav_state_dim_it-DIM::ERROR::EULER_START_IND];
        }
        else
        {
          if(nav_state_dim_it >= DIM::ERROR::EULER_START_IND)
          {
            input_val = nav_state[nav_state_dim_it+1];
          }
          else
          {
            input_val = nav_state[nav_state_dim_it];
          }
        }
        numeric_jacobian[nav_state_dim_it] = boost::math::differentiation::finite_difference_derivative<const std::function<FLOAT(FLOAT)>,FLOAT,numeric_diff_order>(
          dim_func, input_val);
      }

      const Eigen::Matrix<FLOAT,1,DIM::ERROR_DIM,Eigen::RowMajor> analytic_jacobian = test_model.getMeasurementEstimatePDWRErrorState(0, nav_state);

      const bool is_equal = ((numeric_jacobian - analytic_jacobian).array().abs() < this->output_error_eps).all();
      EXPECT_TRUE(is_equal);
      if(not is_equal)
      {
        std::cout << "numeric_jacobian:\n" << numeric_jacobian << std::endl;
        std::cout << "analytic_jacobian:\n" << analytic_jacobian << std::endl;
        std::cout << "diff:\n" << numeric_jacobian - analytic_jacobian << std::endl;
        std::cout << "measurement:\n" << test_model.estimateMeasurement(0, nav_state) << std::endl;
        std::cout << "nav_state:\n" << nav_state << std::endl;
      }
    }
  }
}

/* differential_pressure_test.cpp */
