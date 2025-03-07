/**
 * @File: dubins_airplane_controller_test.cpp
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
#include<kalman_filter/controllers/dubins_airplane_controller.hpp>
#include<kalman_filter/dynamics/dubins_airplane_model.hpp>
#include<kalman_filter/mappings/dubins_airplane_mapping.hpp>


using DIM_S = kf::dynamics::DubinsAirplaneDim;
using SCALAR = double;

class DubinsAirplaneControllerTest
 : public ::testing::Test
{
public:
  DubinsAirplaneControllerTest()
   : test_obj(std::make_shared<kf::dynamics::DubinsAirplane<DIM_S,SCALAR,Eigen::RowMajor>>(9.81, 70, 60),
              std::make_shared<kf::map::DubinsAirplaneMapping<DIM_S,SCALAR,Eigen::RowMajor>>(),
              9.81, kf::math::oneHalfPi<SCALAR>(), 0.05, 1, 10, 0.01, 0.1, 0.3, 0.7)
  {
    const SCALAR small_val = 1;
    for(Eigen::Index dim_ind = 0; dim_ind < DIM_S::REF_DIM; ++dim_ind)
    {
      this->ref_state_min_max_num[dim_ind] = std::make_tuple(-small_val, small_val, 1);
    }
    std::get<0>(this->ref_state_min_max_num[DIM_S::REF::NORTH_VEL_IND]) += 10;
    std::get<1>(this->ref_state_min_max_num[DIM_S::REF::NORTH_VEL_IND]) += 10;
    for(Eigen::Index dim_ind = 0; dim_ind < DIM_S::ERROR_DIM; ++dim_ind)
    {
      this->nav_state_min_max_num[dim_ind] = std::make_tuple(-small_val, small_val, 1);
    }
    std::get<0>(this->nav_state_min_max_num[DIM_S::ERROR::NORTH_VEL_IND]) += 10;
    std::get<1>(this->nav_state_min_max_num[DIM_S::ERROR::NORTH_VEL_IND]) += 10;
    this->next_ref_state.setZero();
    this->next_ref_state[DIM_S::REF::NORTH_VEL_IND] += 10;
    this->ref_dt = 0.1;
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

  inline static constexpr const SCALAR output_error_eps   = 1e-6;
  inline static constexpr const size_t numeric_diff_order = 8;

  kf::control::DubinsAirplaneController<DIM_S,SCALAR,Eigen::RowMajor>  test_obj;
  kf::map::DubinsAirplaneMapping<DIM_S,SCALAR,Eigen::RowMajor>         map_obj;
  inline static constexpr const SCALAR                                 time = std::numeric_limits<SCALAR>::quiet_NaN();
  Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,Eigen::RowMajor>               ref_state;
  Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,Eigen::RowMajor>               next_ref_state;
  SCALAR                                                               ref_dt;
  std::array<std::tuple<SCALAR,SCALAR,SCALAR>,DIM_S::ERROR_DIM>        nav_state_min_max_num;
  std::array<std::tuple<SCALAR,SCALAR,SCALAR>,DIM_S::REF_DIM>          ref_state_min_max_num;
};

TEST_F(DubinsAirplaneControllerTest, getControl)
{
  Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,Eigen::RowMajor> nav_state = this->init<DIM_S::ERROR_DIM,true>(this->nav_state_min_max_num);
  do
  {
    Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,Eigen::RowMajor> ref_state = this->init<DIM_S::REF_DIM,false>(this->ref_state_min_max_num);
    do
    {
      const Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,Eigen::RowMajor> output =
        this->test_obj.getControl(this->time, nav_state, ref_state, this->next_ref_state, this->ref_dt);

      EXPECT_FALSE(output.hasNaN());
      EXPECT_TRUE(output.allFinite());
    } while(this->inc<DIM_S::REF_DIM,false>(ref_state, this->ref_state_min_max_num));
  } while(this->inc<DIM_S::ERROR_DIM,true>(nav_state, this->nav_state_min_max_num));
}

TEST_F(DubinsAirplaneControllerTest, getControlPDWRErrorState)
{
  SCALAR max_error_found = -std::numeric_limits<SCALAR>::infinity();
  Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,Eigen::RowMajor> nav_state = this->init<DIM_S::ERROR_DIM,true>(this->nav_state_min_max_num);
  do
  {
    Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,Eigen::RowMajor> ref_state = this->init<DIM_S::REF_DIM,false>(this->ref_state_min_max_num);
    do
    {
      const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,Eigen::RowMajor> nav_state_copy = this->map_obj.mapTruthNav(this->map_obj.mapNavTruth(nav_state)); // Set velocities correctly
      // Make numeric jacobian
      Eigen::Matrix<SCALAR,DIM_S::CONTROL_DIM,DIM_S::ERROR_DIM,Eigen::RowMajor> numeric_jacobian;
      Eigen::Index out_dim_it;
      Eigen::Index state_dim_it;

      const std::function<SCALAR(const SCALAR)> dim_func =
        [&nav_state_copy,&out_dim_it,&state_dim_it,this,&ref_state] (const SCALAR x) -> SCALAR
        {
          // Set input
          Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,Eigen::RowMajor> input = nav_state_copy;
          if((state_dim_it >= DIM_S::ERROR::EULER_START_IND) and (state_dim_it <= DIM_S::ERROR::EULER_END_IND))
          {
            Eigen::Matrix<SCALAR,1,3,Eigen::RowMajor> rpy = kf::math::quat::quaternionToRollPitchYaw(input.middleCols<4>(DIM_S::NAV::QUAT_START_IND));
            rpy[state_dim_it-DIM_S::ERROR::EULER_START_IND] = x;
            //const SCALAR diff = rpy[state_dim_it-DIM_S::ERROR::EULER_START_IND] - x;
            //rpy[state_dim_it-DIM_S::ERROR::EULER_START_IND] = rpy[state_dim_it-DIM_S::ERROR::EULER_START_IND] - (SCALAR(2) * diff);
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
          const Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,Eigen::RowMajor> reading = this->test_obj.getControl(this->time, input, ref_state, this->next_ref_state, this->ref_dt);

          return reading[out_dim_it];
        };

      for(state_dim_it = 0; state_dim_it < DIM_S::ERROR_DIM; ++state_dim_it)
      {
        for(out_dim_it = 0; out_dim_it < DIM_S::CONTROL_DIM; ++out_dim_it)
        {
          SCALAR input_val;
          if((state_dim_it >= DIM_S::ERROR::EULER_START_IND) and (state_dim_it <= DIM_S::ERROR::EULER_END_IND))
          {
            const Eigen::Matrix<SCALAR,1,3,Eigen::RowMajor> rpy = kf::math::quat::quaternionToRollPitchYaw(nav_state_copy.middleCols<4>(DIM_S::NAV::QUAT_START_IND));
            input_val = rpy[state_dim_it-DIM_S::ERROR::EULER_START_IND];
          }
          else
          {
            if(state_dim_it >= DIM_S::ERROR::EULER_START_IND)
            {
              input_val = nav_state_copy[state_dim_it+1];
            }
            else
            {
              input_val = nav_state_copy[state_dim_it];
            }
          }

          numeric_jacobian(out_dim_it,state_dim_it) = boost::math::differentiation::finite_difference_derivative<const std::function<SCALAR(SCALAR)>,SCALAR,numeric_diff_order>(
            dim_func, input_val);
        }
      }
      //numeric_jacobian.template middleCols<3>(DIM_S::ERROR::EULER_START_IND) /= SCALAR(2);
      // Get analytic_jacobian
      const Eigen::Matrix<SCALAR,DIM_S::CONTROL_DIM,DIM_S::ERROR_DIM,Eigen::RowMajor> analytic_jacobian =
        this->test_obj.getControlPDWRErrorState(this->time, nav_state_copy, ref_state);
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
        std::cout << "nav_state:\n" << nav_state_copy << std::endl;
        std::cout << "ref_state:\n" << ref_state << std::endl;
      }
      EXPECT_TRUE(is_equal);
    } while(this->inc<DIM_S::REF_DIM,false>(ref_state, this->ref_state_min_max_num));
  } while(this->inc<DIM_S::ERROR_DIM,true>(nav_state, this->nav_state_min_max_num));
  std::cout << "Max Error Found: " << max_error_found << std::endl;
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

/* dubins_airplane_model_test.cpp */
