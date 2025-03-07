/**
 * @File: dubins_airplane_mapping_test.cpp
 * @Date: August 2023
 * @Author: James Swedeen
 **/

/* C++ Headers */
#include<array>

/* Testing Headers */
#include<gtest/gtest.h>

/* Boost Headers */
#include<boost/math/differentiation/finite_difference.hpp>

/* Local Headers */
#include<kalman_filter/mappings/dubins_airplane_mapping.hpp>

using DIM_S = kf::dynamics::DubinsAirplaneDim;
using SCALAR = double;

class DubinsAirplaneMappingTest
 : public ::testing::Test
{
public:
  DubinsAirplaneMappingTest()
  {
    const SCALAR small_val = 1;
    for(Eigen::Index dim_ind = 0; dim_ind < DIM_S::REF_DIM; ++dim_ind)
    {
      this->ref_state_min_max_num[dim_ind] = std::make_tuple(-small_val, small_val, 1);
    }
    for(Eigen::Index dim_ind = 0; dim_ind < DIM_S::TRUTH_DIM; ++dim_ind)
    {
      this->truth_state_min_max_num[dim_ind] = std::make_tuple(-small_val, small_val, 1);
    }
    std::get<0>(this->truth_state_min_max_num[DIM_S::TRUTH::AIR_SPEED_IND]) += 10;
    std::get<1>(this->truth_state_min_max_num[DIM_S::TRUTH::AIR_SPEED_IND]) += 10;
    std::get<0>(this->truth_state_min_max_num[DIM_S::TRUTH::ROLL_IND]) += 1.14;
    std::get<1>(this->truth_state_min_max_num[DIM_S::TRUTH::ROLL_IND]) += 1.14;
    //std::get<0>(this->truth_state_min_max_num[DIM_S::TRUTH::PITCH_IND]) += 1.14;
    //std::get<1>(this->truth_state_min_max_num[DIM_S::TRUTH::PITCH_IND]) += 1.14;
    //std::get<0>(this->truth_state_min_max_num[DIM_S::TRUTH::YAW_IND]) += 3.14;
    //std::get<1>(this->truth_state_min_max_num[DIM_S::TRUTH::YAW_IND]) += 3.14;
    for(Eigen::Index dim_ind = 0; dim_ind < DIM_S::ERROR_DIM; ++dim_ind)
    {
      this->nav_state_min_max_num[dim_ind] = std::make_tuple(-small_val, small_val, 1);
    }
    std::get<0>(this->nav_state_min_max_num[DIM_S::ERROR::NORTH_VEL_IND]) += 10;
    std::get<1>(this->nav_state_min_max_num[DIM_S::ERROR::NORTH_VEL_IND]) += 10;
  }

  inline SCALAR findStep(const std::tuple<SCALAR,SCALAR,SCALAR>& min_max_num) const
  {
    return (std::get<1>(min_max_num) - std::get<0>(min_max_num)) / std::get<2>(min_max_num);
  }
  template<Eigen::Index DIM, bool IS_NAV = false>
  inline SCALAR inc(Eigen::Ref<Eigen::Matrix<SCALAR,1,(IS_NAV) ? DIM_S::NAV_DIM : DIM,Eigen::RowMajor>> state,
                    const std::array<std::tuple<SCALAR,SCALAR,SCALAR>,DIM>&                             min_max_nums)
  {
    for(Eigen::Index dim_ind = 0; dim_ind < DIM; ++dim_ind)
    {
      if constexpr(IS_NAV)
      {
        if((dim_ind >= DIM_S::ERROR::EULER_START_IND) and (dim_ind <= DIM_S::ERROR::EULER_END_IND))
        {
          Eigen::Matrix<SCALAR,1,3,Eigen::RowMajor> rpy = kf::math::quat::quaternionToRollPitchYaw(state.template middleCols<4>(DIM_S::NAV::QUAT_START_IND));
          rpy[dim_ind-DIM_S::ERROR::EULER_START_IND] += this->findStep(min_max_nums[dim_ind]);
          if(std::get<1>(min_max_nums[dim_ind]) >= rpy[dim_ind-DIM_S::ERROR::EULER_START_IND])
          {
            state.template middleCols<4>(DIM_S::NAV::QUAT_START_IND) = kf::math::quat::normalize(kf::math::quat::rollPitchYawToQuaternion(rpy));
            return true;
          }
          rpy[dim_ind-DIM_S::ERROR::EULER_START_IND] = std::get<0>(min_max_nums[dim_ind]);
          state.template middleCols<4>(DIM_S::NAV::QUAT_START_IND) = kf::math::quat::normalize(kf::math::quat::rollPitchYawToQuaternion(rpy));
        }
        else
        {
          if(dim_ind >= DIM_S::ERROR::EULER_START_IND)
          {
            state[dim_ind+1] += this->findStep(min_max_nums[dim_ind]);
            if(std::get<1>(min_max_nums[dim_ind]) >= state[dim_ind+1])
            {
              return true;
            }
            state[dim_ind+1] = std::get<0>(min_max_nums[dim_ind]);
          }
          else
          {
            state[dim_ind] += this->findStep(min_max_nums[dim_ind]);
            if(std::get<1>(min_max_nums[dim_ind]) >= state[dim_ind])
            {
              return true;
            }
            state[dim_ind] = std::get<0>(min_max_nums[dim_ind]);
          }
        }
      }
      else // Not navigation state
      {
        state[dim_ind] += this->findStep(min_max_nums[dim_ind]);
        if(std::get<1>(min_max_nums[dim_ind]) >= state[dim_ind])
        {
          return true;
        }
        state[dim_ind] = std::get<0>(min_max_nums[dim_ind]);
      }
    }
    return false;
  }
  template<Eigen::Index DIM, bool IS_NAV = false>
  inline Eigen::Matrix<SCALAR,1,(IS_NAV) ? DIM_S::NAV_DIM : DIM,Eigen::RowMajor> init(const std::array<std::tuple<SCALAR,SCALAR,SCALAR>,DIM>& min_max_nums)
  {
    Eigen::Matrix<SCALAR,1,(IS_NAV) ? DIM_S::NAV_DIM : DIM,Eigen::RowMajor> output;
    for(Eigen::Index dim_ind = 0; dim_ind < DIM; ++dim_ind)
    {
      if constexpr(IS_NAV)
      {
        if((dim_ind >= DIM_S::ERROR::EULER_START_IND) and (dim_ind <= DIM_S::ERROR::EULER_END_IND))
        {
          const Eigen::Matrix<SCALAR,1,3,Eigen::RowMajor> rpy({std::get<0>(min_max_nums[DIM_S::ERROR::EULER_START_IND]),
                                                               std::get<0>(min_max_nums[DIM_S::ERROR::EULER_START_IND+1]),
                                                               std::get<0>(min_max_nums[DIM_S::ERROR::EULER_START_IND+2])});
          output.template middleCols<4>(DIM_S::NAV::QUAT_START_IND) = kf::math::quat::normalize(kf::math::quat::rollPitchYawToQuaternion(rpy));
        }
        else
        {
          if(dim_ind >= DIM_S::ERROR::EULER_START_IND)
          {
            output[dim_ind+1] = std::get<0>(min_max_nums[dim_ind]);
          }
          else
          {
            output[dim_ind] = std::get<0>(min_max_nums[dim_ind]);
          }
        }
      }
      else // Not navigation state
      {
        output[dim_ind] = std::get<0>(min_max_nums[dim_ind]);
      }
    }
    return output;
  }

  inline static constexpr const SCALAR output_error_eps   = 1e-8;
  inline static constexpr const size_t numeric_diff_order = 8;

  kf::map::DubinsAirplaneMapping<DIM_S,SCALAR,Eigen::RowMajor>  test_obj;
  std::array<std::tuple<SCALAR,SCALAR,SCALAR>,DIM_S::REF_DIM>   ref_state_min_max_num;
  std::array<std::tuple<SCALAR,SCALAR,SCALAR>,DIM_S::TRUTH_DIM> truth_state_min_max_num;
  std::array<std::tuple<SCALAR,SCALAR,SCALAR>,DIM_S::ERROR_DIM> nav_state_min_max_num;
};


TEST_F(DubinsAirplaneMappingTest, correctErrors)
{
  return;
  Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,Eigen::RowMajor> nav_state = this->init<DIM_S::ERROR_DIM,true>(this->nav_state_min_max_num);
  do
  {
    Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,Eigen::RowMajor> truth_state = this->init<DIM_S::TRUTH_DIM>(this->truth_state_min_max_num);
    do
    {
      const Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,Eigen::RowMajor> error = this->test_obj.calculateErrorState(truth_state, nav_state);
      EXPECT_FALSE(error.hasNaN());
      EXPECT_TRUE(error.allFinite());

      const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,Eigen::RowMajor> corrected_nav_state = this->test_obj.correctErrors(nav_state, error);
      EXPECT_FALSE(corrected_nav_state.hasNaN());
      EXPECT_TRUE(corrected_nav_state.allFinite());

      const Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,Eigen::RowMajor> corrected_error_state = this->test_obj.calculateErrorState(truth_state, corrected_nav_state);
      EXPECT_FALSE(corrected_error_state.hasNaN());
      EXPECT_TRUE(corrected_error_state.allFinite());

      const bool is_zero = (corrected_error_state.array().abs() < this->output_error_eps).all();
      if(not is_zero)
      {
        std::cout << "nav_state:\n" << nav_state << std::endl;
        std::cout << "nav_state mapped to truth:\n" << this->test_obj.mapNavTruth(nav_state) << std::endl;
        std::cout << "truth_state:\n" << truth_state << std::endl;
        std::cout << "truth_state mapped to nav:\n" << this->test_obj.mapTruthNav(truth_state) << std::endl;
        std::cout << "error:\n" << error << std::endl;
        std::cout << "corrected_nav_state:\n" << corrected_nav_state << std::endl;
        std::cout << "corrected_error_state:\n" << corrected_error_state << std::endl;
      }
      EXPECT_TRUE(is_zero);

      const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,Eigen::RowMajor> corrected_truth_state = this->test_obj.mapNavTruth(corrected_nav_state);
      EXPECT_FALSE(corrected_truth_state.hasNaN());
      EXPECT_TRUE(corrected_truth_state.allFinite());

      const bool is_equal = ((truth_state - corrected_truth_state).array().abs() < this->output_error_eps).all();
      EXPECT_TRUE(is_equal);
    } while(this->inc<DIM_S::TRUTH_DIM>(truth_state, this->truth_state_min_max_num));
  } while(this->inc<DIM_S::ERROR_DIM,true>(nav_state, this->nav_state_min_max_num));
}

TEST_F(DubinsAirplaneMappingTest, mapRefTruthNav)
{
  Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,Eigen::RowMajor> ref_state = this->init<DIM_S::REF_DIM>(this->ref_state_min_max_num);
  do
  {
    Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,Eigen::RowMajor> ref_state_copy = ref_state;
    // Set velocities correctly
    const SCALAR air_speed = ref_state.middleCols<3>(DIM_S::REF::VEL_START_IND).norm();
    ref_state_copy[DIM_S::REF::NORTH_VEL_IND] =  air_speed * std::cos(ref_state[DIM_S::REF::YAW_IND]) * std::cos(ref_state[DIM_S::REF::PITCH_IND]);
    ref_state_copy[DIM_S::REF::EAST_VEL_IND]  =  air_speed * std::sin(ref_state[DIM_S::REF::YAW_IND]) * std::cos(ref_state[DIM_S::REF::PITCH_IND]);
    ref_state_copy[DIM_S::REF::DOWN_VEL_IND]  = -air_speed * std::sin(ref_state[DIM_S::REF::PITCH_IND]);

    const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,Eigen::RowMajor> ref_truth_state = this->test_obj.mapRefTruth(ref_state_copy);
    const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,  Eigen::RowMajor> ref_nav_state   = this->test_obj.mapRefNav(  ref_state_copy);

    EXPECT_FALSE(ref_truth_state.hasNaN());
    EXPECT_TRUE(ref_truth_state.allFinite());
    EXPECT_FALSE(ref_nav_state.hasNaN());
    EXPECT_TRUE(ref_nav_state.allFinite());

    const Eigen::Matrix<SCALAR,1,DIM_S::ERROR_DIM,Eigen::RowMajor> error_state = this->test_obj.calculateErrorState(ref_truth_state, ref_nav_state);
    EXPECT_TRUE((error_state.array().abs() < this->output_error_eps).all());
  } while(this->inc<DIM_S::REF_DIM>(ref_state, this->ref_state_min_max_num));
}

TEST_F(DubinsAirplaneMappingTest, mapTruthNav)
{
  return;
  Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,Eigen::RowMajor> truth_state = this->init<DIM_S::TRUTH_DIM>(this->truth_state_min_max_num);
  do
  {
    const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,Eigen::RowMajor> output = this->test_obj.mapTruthNav(truth_state);

    EXPECT_FALSE(output.hasNaN());
    EXPECT_TRUE(output.allFinite());

    const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,Eigen::RowMajor> mapped_truth_state = this->test_obj.mapNavTruth(output);
    EXPECT_TRUE(((truth_state - mapped_truth_state).array().abs() < this->output_error_eps).all());
  } while(this->inc<DIM_S::TRUTH_DIM>(truth_state, this->truth_state_min_max_num));
}

TEST_F(DubinsAirplaneMappingTest, mapNavTruth)
{
  Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,Eigen::RowMajor> nav_state = this->init<DIM_S::ERROR_DIM,true>(this->nav_state_min_max_num);
  do
  {
    const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,Eigen::RowMajor> nav_state_copy = this->test_obj.mapTruthNav(this->test_obj.mapNavTruth(nav_state)); // Set velocities correctly

    const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,Eigen::RowMajor> output = this->test_obj.mapNavTruth(nav_state_copy);

    EXPECT_FALSE(output.hasNaN());
    EXPECT_TRUE(output.allFinite());

    const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,Eigen::RowMajor> mapped_nav_state = this->test_obj.mapTruthNav(output);
    const bool is_equal = ((nav_state_copy - mapped_nav_state).array().abs() < this->output_error_eps).all();
    if(not is_equal)
    {
      std::cout << "nav_state:\n" << nav_state_copy << std::endl;
      std::cout << "truth_state:\n" << output << std::endl;
      std::cout << "mapped_nav_state:\n" << mapped_nav_state << std::endl;
      std::cout << "diff:\n" << nav_state - mapped_nav_state << std::endl;
    }
    EXPECT_TRUE(is_equal);
  } while(this->inc<DIM_S::ERROR_DIM,true>(nav_state, this->nav_state_min_max_num));
}

TEST_F(DubinsAirplaneMappingTest, getTruthNavMapPDWRDispersionState)
{
  SCALAR max_error_found = -std::numeric_limits<SCALAR>::infinity();
  Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,Eigen::RowMajor> truth_state = this->init<DIM_S::TRUTH_DIM>(this->truth_state_min_max_num);
  do
  {
    // Make numeric jacobian
    Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::TRUTH_DISP_DIM,Eigen::RowMajor> numeric_jacobian;
    Eigen::Index out_dim_it;
    Eigen::Index state_dim_it;

    const std::function<SCALAR(const SCALAR)> dim_func =
      [&truth_state,&out_dim_it,&state_dim_it,this] (const SCALAR x) -> SCALAR
      {
        // Set input
        Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,Eigen::RowMajor> input = truth_state;
        input[state_dim_it] = x;

        // Make reading
        const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,Eigen::RowMajor> reading = this->test_obj.mapTruthNav(input);

        SCALAR output;
        if((out_dim_it >= DIM_S::ERROR::EULER_START_IND) and (out_dim_it <= DIM_S::ERROR::EULER_END_IND))
        {
          const Eigen::Matrix<SCALAR,1,3,Eigen::RowMajor> rpy = kf::math::quat::quaternionToRollPitchYaw(reading.middleCols<4>(DIM_S::NAV::QUAT_START_IND));
          output = rpy[out_dim_it-DIM_S::ERROR::EULER_START_IND];
        }
        else
        {
          if(out_dim_it >= DIM_S::ERROR::EULER_START_IND)
          {
            output = reading[out_dim_it+1];
          }
          else
          {
            output =  reading[out_dim_it];
          }
        }

        return output;
      };

    for(state_dim_it = 0; state_dim_it < DIM_S::TRUTH_DISP_DIM; ++state_dim_it)
    {
      for(out_dim_it = 0; out_dim_it < DIM_S::ERROR_DIM; ++out_dim_it)
      {
        numeric_jacobian(out_dim_it,state_dim_it) = boost::math::differentiation::finite_difference_derivative<const std::function<SCALAR(SCALAR)>,SCALAR,numeric_diff_order>(
          dim_func, truth_state[state_dim_it]);
      }
    }
    // Get analytic_jacobian
    Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::TRUTH_DISP_DIM,Eigen::RowMajor> analytic_jacobian =
      this->test_obj.getTruthNavMapPDWRDispersionState(truth_state);
    // Test
    EXPECT_FALSE(analytic_jacobian.hasNaN());
    const SCALAR max_abs_error = (numeric_jacobian - analytic_jacobian).array().abs().maxCoeff();
    if(max_abs_error > max_error_found)
    {
      max_error_found = max_abs_error;
    }
    const bool is_equal = max_abs_error < this->output_error_eps;
    if(not is_equal)
    {
      std::cout << "numeric_jacobian:\n" << numeric_jacobian << std::endl;
      std::cout << "analytic_jacobian:\n" << analytic_jacobian << std::endl;
      std::cout << "diff:\n" << numeric_jacobian - analytic_jacobian << std::endl;
      std::cout << "truth_state:\n" << truth_state << std::endl;
    }
    EXPECT_TRUE(is_equal);
  } while(this->inc<DIM_S::TRUTH_DIM>(truth_state, this->truth_state_min_max_num));
  std::cout << "Max Error Found: " << max_error_found << std::endl;
}

TEST_F(DubinsAirplaneMappingTest, getNavTruthMapPDWRErrorState)
{
  SCALAR max_error_found = -std::numeric_limits<SCALAR>::infinity();
  Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,Eigen::RowMajor> nav_state = this->init<DIM_S::ERROR_DIM,true>(this->nav_state_min_max_num);
  do
  {
    // Make numeric jacobian
    Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::ERROR_DIM,Eigen::RowMajor> numeric_jacobian;
    Eigen::Index out_dim_it;
    Eigen::Index state_dim_it;

    const std::function<SCALAR(const SCALAR)> dim_func =
      [&nav_state,&out_dim_it,&state_dim_it,this] (const SCALAR x) -> SCALAR
      {
        // Set input
        Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,Eigen::RowMajor> input = nav_state;
        if((state_dim_it >= DIM_S::ERROR::EULER_START_IND) and (state_dim_it <= DIM_S::ERROR::EULER_END_IND))
        {
          Eigen::Matrix<SCALAR,1,3,Eigen::RowMajor> rpy = kf::math::quat::quaternionToRollPitchYaw(input.middleCols<4>(DIM_S::NAV::QUAT_START_IND));
          rpy[state_dim_it-DIM_S::ERROR::EULER_START_IND] = x;
          input.middleCols<4>(DIM_S::NAV::QUAT_START_IND) = kf::math::quat::normalize(kf::math::quat::rollPitchYawToQuaternion(rpy));
        }
        else
        {
          if(state_dim_it >= DIM_S::ERROR::EULER_START_IND)
          {
            input[state_dim_it+1] = x;
          }
          else
          {
            input[state_dim_it] = x;
          }
        }

        // Make reading
        const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,Eigen::RowMajor> reading = this->test_obj.mapNavTruth(input);

        return reading[out_dim_it];
      };

    for(state_dim_it = 0; state_dim_it < DIM_S::ERROR_DIM; ++state_dim_it)
    {
      for(out_dim_it = 0; out_dim_it < DIM_S::TRUTH_DISP_DIM; ++out_dim_it)
      {
        SCALAR input_val;
        if((state_dim_it >= DIM_S::ERROR::EULER_START_IND) and (state_dim_it <= DIM_S::ERROR::EULER_END_IND))
        {
          const Eigen::Matrix<SCALAR,1,3,Eigen::RowMajor> rpy = kf::math::quat::quaternionToRollPitchYaw(nav_state.middleCols<4>(DIM_S::NAV::QUAT_START_IND));
          input_val = rpy[state_dim_it-DIM_S::ERROR::EULER_START_IND];
        }
        else
        {
          if(state_dim_it >= DIM_S::ERROR::EULER_START_IND)
          {
            input_val = nav_state[state_dim_it+1];
          }
          else
          {
            input_val = nav_state[state_dim_it];
          }
        }

        numeric_jacobian(out_dim_it,state_dim_it) = boost::math::differentiation::finite_difference_derivative<const std::function<SCALAR(SCALAR)>,SCALAR,numeric_diff_order>(
          dim_func, input_val);
      }
    }
    // Get analytic_jacobian
    Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::ERROR_DIM,Eigen::RowMajor> analytic_jacobian =
      this->test_obj.getNavTruthMapPDWRErrorState(nav_state);
    // Test
    EXPECT_FALSE(analytic_jacobian.hasNaN());
    const SCALAR max_abs_error = (numeric_jacobian - analytic_jacobian).array().abs().maxCoeff();
    if(max_abs_error > max_error_found)
    {
      max_error_found = max_abs_error;
    }
    const bool is_equal = max_abs_error < this->output_error_eps;
    if(not is_equal)
    {
      std::cout << "numeric_jacobian:\n" << numeric_jacobian << std::endl;
      std::cout << "analytic_jacobian:\n" << analytic_jacobian << std::endl;
      std::cout << "diff:\n" << numeric_jacobian - analytic_jacobian << std::endl;
      std::cout << "nav_state:\n" << nav_state << std::endl;
    }
    EXPECT_TRUE(is_equal);
  } while(this->inc<DIM_S::ERROR_DIM,true>(nav_state, this->nav_state_min_max_num));
  std::cout << "Max Error Found: " << max_error_found << std::endl;
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

/* dubins_airplane_model_test.cpp */
