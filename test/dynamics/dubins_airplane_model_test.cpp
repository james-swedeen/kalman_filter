/**
 * @File: dubins_airplane_model_test.cpp
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
#include<kalman_filter/dynamics/dubins_airplane_model.hpp>

using DIM_S = kf::dynamics::DubinsAirplaneDim;
using SCALAR = double;

class DubinsAirplaneTest
 : public ::testing::Test
{
public:
  DubinsAirplaneTest()
   : test_obj(9.81, 60, 60)
  {
    const SCALAR small_val = 1e-4;
    for(Eigen::Index dim_ind = 0; dim_ind < DIM_S::TRUTH_DIM; ++dim_ind)
    {
      this->truth_state_min_max_num[dim_ind] = std::make_tuple(-small_val, small_val, 1);
    }
    std::get<0>(this->truth_state_min_max_num[DIM_S::TRUTH::AIR_SPEED_IND]) += 80;
    std::get<1>(this->truth_state_min_max_num[DIM_S::TRUTH::AIR_SPEED_IND]) += 80;
    /*std::get<0>(this->truth_state_min_max_num[DIM_S::TRUTH::YAW_IND]) += 3.14/4.0;
    std::get<1>(this->truth_state_min_max_num[DIM_S::TRUTH::YAW_IND]) += 3.14/4.0;
    std::get<0>(this->truth_state_min_max_num[DIM_S::TRUTH::PITCH_IND]) += 3.14/4.0;
    std::get<1>(this->truth_state_min_max_num[DIM_S::TRUTH::PITCH_IND]) += 3.14/4.0;
    std::get<0>(this->truth_state_min_max_num[DIM_S::TRUTH::ROLL_IND]) += 3.14/4.0;
    std::get<1>(this->truth_state_min_max_num[DIM_S::TRUTH::ROLL_IND]) += 3.14/4.0;*/
    for(Eigen::Index dim_ind = 0; dim_ind < DIM_S::ERROR_DIM; ++dim_ind)
    {
      this->nav_state_min_max_num[dim_ind] = std::make_tuple(-small_val, small_val, 1);
    }
    /*std::get<0>(this->nav_state_min_max_num[DIM_S::ERROR::YAW_IND])   += 3.14/8.0;
    std::get<1>(this->nav_state_min_max_num[DIM_S::ERROR::YAW_IND])   += 3.14/8.0;
    std::get<0>(this->nav_state_min_max_num[DIM_S::ERROR::PITCH_IND]) += 3.14/8.0;
    std::get<1>(this->nav_state_min_max_num[DIM_S::ERROR::PITCH_IND]) += 3.14/8.0;
    std::get<0>(this->nav_state_min_max_num[DIM_S::ERROR::ROLL_IND])  += 3.14/8.0;
    std::get<1>(this->nav_state_min_max_num[DIM_S::ERROR::ROLL_IND])  += 3.14/8.0;*/
    for(Eigen::Index dim_ind = 0; dim_ind < DIM_S::CONTROL_DIM; ++dim_ind)
    {
      this->control_min_max_num[dim_ind] = std::make_tuple(-small_val, small_val, 1);
    }
    for(Eigen::Index dim_ind = 0; dim_ind < DIM_S::TRUTH_NOISE_DIM; ++dim_ind)
    {
      this->process_noise_min_max_num[dim_ind] = std::make_tuple(-small_val, small_val, 1);
    }
    for(Eigen::Index dim_ind = 0; dim_ind < DIM_S::INER_MEAS_DIM; ++dim_ind)
    {
      this->inertial_meas_min_max_num[dim_ind] = std::make_tuple(-small_val, small_val, 1);
    }
    for(Eigen::Index dim_ind = 0; dim_ind < DIM_S::INER_MEAS_NOISE_DIM; ++dim_ind)
    {
      this->inertial_noise_min_max_num[dim_ind] = std::make_tuple(-small_val, small_val, 1);
    }
  }

  inline SCALAR findStep(const std::tuple<SCALAR,SCALAR,SCALAR>& min_max_num) const
  {
    return (std::get<1>(min_max_num) - std::get<0>(min_max_num)) / std::get<2>(min_max_num);
  }
  template<Eigen::Index DIM>
  inline SCALAR inc(Eigen::Ref<Eigen::Matrix<SCALAR,1,(DIM == DIM_S::ERROR_DIM) ? DIM_S::NAV_DIM : DIM,Eigen::RowMajor>> state,
                    const std::array<std::tuple<SCALAR,SCALAR,SCALAR>,DIM>&                                              min_max_nums)
  {
    for(Eigen::Index dim_ind = 0; dim_ind < DIM; ++dim_ind)
    {
      if constexpr(DIM == DIM_S::ERROR_DIM)
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
  template<Eigen::Index DIM>
  inline Eigen::Matrix<SCALAR,1,(DIM == DIM_S::ERROR_DIM) ? DIM_S::NAV_DIM : DIM,Eigen::RowMajor> init(const std::array<std::tuple<SCALAR,SCALAR,SCALAR>,DIM>& min_max_nums)
  {
    Eigen::Matrix<SCALAR,1,(DIM == DIM_S::ERROR_DIM) ? DIM_S::NAV_DIM : DIM,Eigen::RowMajor> output;
    for(Eigen::Index dim_ind = 0; dim_ind < DIM; ++dim_ind)
    {
      if constexpr(DIM == DIM_S::ERROR_DIM)
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

  inline static constexpr const SCALAR output_error_eps   = 1e-4;
  inline static constexpr const size_t numeric_diff_order = 8;

  kf::dynamics::DubinsAirplane<DIM_S,SCALAR,Eigen::RowMajor>              test_obj;
  inline static constexpr const SCALAR                                    time = std::numeric_limits<SCALAR>::quiet_NaN();
  std::array<std::tuple<SCALAR,SCALAR,SCALAR>,DIM_S::TRUTH_DIM>           truth_state_min_max_num;
  std::array<std::tuple<SCALAR,SCALAR,SCALAR>,DIM_S::ERROR_DIM>           nav_state_min_max_num;
  std::array<std::tuple<SCALAR,SCALAR,SCALAR>,DIM_S::CONTROL_DIM>         control_min_max_num;
  std::array<std::tuple<SCALAR,SCALAR,SCALAR>,DIM_S::TRUTH_NOISE_DIM>     process_noise_min_max_num;
  std::array<std::tuple<SCALAR,SCALAR,SCALAR>,DIM_S::INER_MEAS_DIM>       inertial_meas_min_max_num;
  std::array<std::tuple<SCALAR,SCALAR,SCALAR>,DIM_S::INER_MEAS_NOISE_DIM> inertial_noise_min_max_num;
};

TEST_F(DubinsAirplaneTest, getTruthStateDynamics)
{
  Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,Eigen::RowMajor> truth_state = this->init<DIM_S::TRUTH_DIM>(this->truth_state_min_max_num);
  do
  {
    Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,Eigen::RowMajor> control = this->init<DIM_S::CONTROL_DIM>(this->control_min_max_num);
    do
    {
      Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_NOISE_DIM,Eigen::RowMajor> process_noise = this->init<DIM_S::TRUTH_NOISE_DIM>(this->process_noise_min_max_num);
      do
      {
        const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,Eigen::RowMajor> output =
          this->test_obj.getTruthStateDynamics(this->time, truth_state, control, process_noise);

        EXPECT_FALSE(output.hasNaN());
        EXPECT_TRUE(output.allFinite());
      } while(this->inc<DIM_S::TRUTH_NOISE_DIM>(process_noise, this->process_noise_min_max_num));
    } while(this->inc<DIM_S::CONTROL_DIM>(control, this->control_min_max_num));
  } while(this->inc<DIM_S::TRUTH_DIM>(truth_state, this->truth_state_min_max_num));
}

TEST_F(DubinsAirplaneTest, getNavStateDynamics)
{
  Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,Eigen::RowMajor> nav_state = this->init<DIM_S::ERROR_DIM>(this->nav_state_min_max_num);
  do
  {
    Eigen::Matrix<SCALAR,1,DIM_S::INER_MEAS_DIM,Eigen::RowMajor> inertial_meas = this->init<DIM_S::INER_MEAS_DIM>(this->inertial_meas_min_max_num);
    do
    {
      const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,Eigen::RowMajor> output =
        this->test_obj.getNavStateDynamics(this->time, nav_state, Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,Eigen::RowMajor>::Zero(), inertial_meas);

      EXPECT_FALSE(output.hasNaN());
      EXPECT_TRUE(output.allFinite());
    } while(this->inc<DIM_S::INER_MEAS_DIM>(inertial_meas, this->inertial_meas_min_max_num));
  } while(this->inc<DIM_S::ERROR_DIM>(nav_state, this->nav_state_min_max_num));
}

TEST_F(DubinsAirplaneTest, getTruthStateDynamicsPDWRDispersionState)
{
  SCALAR max_error_found = -std::numeric_limits<SCALAR>::infinity();
  Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,Eigen::RowMajor> truth_state = this->init<DIM_S::TRUTH_DIM>(this->truth_state_min_max_num);
  do
  {
    Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,Eigen::RowMajor> control = this->init<DIM_S::CONTROL_DIM>(this->control_min_max_num);
    do
    {
      Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_NOISE_DIM,Eigen::RowMajor> process_noise = this->init<DIM_S::TRUTH_NOISE_DIM>(this->process_noise_min_max_num);
      do
      {
        // Make numeric jacobian
        Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_DISP_DIM,Eigen::RowMajor> numeric_jacobian;
        Eigen::Index out_dim_it;
        Eigen::Index state_dim_it;

        const std::function<SCALAR(const SCALAR)> dim_func =
          [&truth_state,&control,&process_noise,&out_dim_it,&state_dim_it,this] (const SCALAR x) -> SCALAR
          {
            // Set input
            Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,Eigen::RowMajor> input = truth_state;
            input[state_dim_it] = x;

            // Make reading
            const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,Eigen::RowMajor> reading = this->test_obj.getTruthStateDynamics(this->time, input, control, process_noise);

            return reading[out_dim_it];
          };

        for(state_dim_it = 0; state_dim_it < DIM_S::TRUTH_DISP_DIM; ++state_dim_it)
        {
          for(out_dim_it = 0; out_dim_it < DIM_S::TRUTH_DISP_DIM; ++out_dim_it)
          {
            numeric_jacobian(out_dim_it,state_dim_it) = boost::math::differentiation::finite_difference_derivative<const std::function<SCALAR(SCALAR)>,SCALAR,numeric_diff_order>(
              dim_func, truth_state[state_dim_it]);
          }
        }
        // Get analytic_jacobian
        Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_DISP_DIM,Eigen::RowMajor> analytic_jacobian =
          this->test_obj.getTruthStateDynamicsPDWRDispersionState(this->time, truth_state, control);
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
          std::cout << "control:\n" << control << std::endl;
          std::cout << "process_noise:\n" << process_noise << std::endl;
        }
        EXPECT_TRUE(is_equal);
      } while(this->inc<DIM_S::TRUTH_NOISE_DIM>(process_noise, this->process_noise_min_max_num));
    } while(this->inc<DIM_S::CONTROL_DIM>(control, this->control_min_max_num));
  } while(this->inc<DIM_S::TRUTH_DIM>(truth_state, this->truth_state_min_max_num));
  std::cout << "Max Error Found: " << max_error_found << std::endl;
}

TEST_F(DubinsAirplaneTest, getTruthStateDynamicsPDWRControl)
{
  SCALAR max_error_found = -std::numeric_limits<SCALAR>::infinity();
  Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,Eigen::RowMajor> truth_state = this->init<DIM_S::TRUTH_DIM>(this->truth_state_min_max_num);
  do
  {
    Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,Eigen::RowMajor> control = this->init<DIM_S::CONTROL_DIM>(this->control_min_max_num);
    do
    {
      Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_NOISE_DIM,Eigen::RowMajor> process_noise = this->init<DIM_S::TRUTH_NOISE_DIM>(this->process_noise_min_max_num);
      do
      {
        // Make numeric jacobian
        Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::CONTROL_DIM,Eigen::RowMajor> numeric_jacobian;
        Eigen::Index out_dim_it;
        Eigen::Index state_dim_it;

        const std::function<SCALAR(const SCALAR)> dim_func =
          [&truth_state,&control,&process_noise,&out_dim_it,&state_dim_it,this] (const SCALAR x) -> SCALAR
          {
            // Set input
            Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,Eigen::RowMajor> input = control;
            input[state_dim_it] = x;

            // Make reading
            const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,Eigen::RowMajor> reading = this->test_obj.getTruthStateDynamics(this->time, truth_state, input, process_noise);

            return reading[out_dim_it];
          };

        for(state_dim_it = 0; state_dim_it < DIM_S::CONTROL_DIM; ++state_dim_it)
        {
          for(out_dim_it = 0; out_dim_it < DIM_S::TRUTH_DISP_DIM; ++out_dim_it)
          {
            numeric_jacobian(out_dim_it,state_dim_it) = boost::math::differentiation::finite_difference_derivative<const std::function<SCALAR(SCALAR)>,SCALAR,numeric_diff_order>(
              dim_func, control[state_dim_it]);
          }
        }
        // Get analytic_jacobian
        Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::CONTROL_DIM,Eigen::RowMajor> analytic_jacobian =
          this->test_obj.getTruthStateDynamicsPDWRControl(this->time, truth_state, control).transpose();
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
          std::cout << "control:\n" << control << std::endl;
          std::cout << "process_noise:\n" << process_noise << std::endl;
        }
        EXPECT_TRUE(is_equal);
      } while(this->inc<DIM_S::TRUTH_NOISE_DIM>(process_noise, this->process_noise_min_max_num));
    } while(this->inc<DIM_S::CONTROL_DIM>(control, this->control_min_max_num));
  } while(this->inc<DIM_S::TRUTH_DIM>(truth_state, this->truth_state_min_max_num));
  std::cout << "Max Error Found: " << max_error_found << std::endl;
}

TEST_F(DubinsAirplaneTest, getTruthStateDynamicsPDWRNoise)
{
  SCALAR max_error_found = -std::numeric_limits<SCALAR>::infinity();
  Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,Eigen::RowMajor> truth_state = this->init<DIM_S::TRUTH_DIM>(this->truth_state_min_max_num);
  do
  {
    Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,Eigen::RowMajor> control = this->init<DIM_S::CONTROL_DIM>(this->control_min_max_num);
    do
    {
      Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_NOISE_DIM,Eigen::RowMajor> process_noise = this->init<DIM_S::TRUTH_NOISE_DIM>(this->process_noise_min_max_num);
      do
      {
        // Make numeric jacobian
        Eigen::Matrix<SCALAR,DIM_S::TRUTH_NOISE_DIM,DIM_S::TRUTH_DISP_DIM,Eigen::RowMajor> numeric_jacobian;
        Eigen::Index out_dim_it;
        Eigen::Index state_dim_it;

        const std::function<SCALAR(const SCALAR)> dim_func =
          [&truth_state,&control,&process_noise,&out_dim_it,&state_dim_it,this] (const SCALAR x) -> SCALAR
          {
            // Set input
            Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_NOISE_DIM,Eigen::RowMajor> input = process_noise;
            input[state_dim_it] = x;

            // Make reading
            const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,Eigen::RowMajor> reading = this->test_obj.getTruthStateDynamics(this->time, truth_state, control, input);

            return reading[out_dim_it];
          };

        for(state_dim_it = 0; state_dim_it < DIM_S::TRUTH_NOISE_DIM; ++state_dim_it)
        {
          for(out_dim_it = 0; out_dim_it < DIM_S::TRUTH_DISP_DIM; ++out_dim_it)
          {
            numeric_jacobian(state_dim_it,out_dim_it) = boost::math::differentiation::finite_difference_derivative<const std::function<SCALAR(SCALAR)>,SCALAR,numeric_diff_order>(
              dim_func, process_noise[state_dim_it]);
          }
        }
        // Get analytic_jacobian
        Eigen::Matrix<SCALAR,DIM_S::TRUTH_NOISE_DIM,DIM_S::TRUTH_DISP_DIM,Eigen::RowMajor> analytic_jacobian =
          this->test_obj.getTruthStateDynamicsPDWRNoise(this->time, truth_state, control).transpose();
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
          std::cout << "control:\n" << control << std::endl;
          std::cout << "process_noise:\n" << process_noise << std::endl;
        }
        EXPECT_TRUE(is_equal);
      } while(this->inc<DIM_S::TRUTH_NOISE_DIM>(process_noise, this->process_noise_min_max_num));
    } while(this->inc<DIM_S::CONTROL_DIM>(control, this->control_min_max_num));
  } while(this->inc<DIM_S::TRUTH_DIM>(truth_state, this->truth_state_min_max_num));
  std::cout << "Max Error Found: " << max_error_found << std::endl;
}

TEST_F(DubinsAirplaneTest, getNavStateDynamicsPDWRErrorState)
{
  SCALAR max_error_found = -std::numeric_limits<SCALAR>::infinity();
  Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,Eigen::RowMajor> nav_state = this->init<DIM_S::ERROR_DIM>(this->nav_state_min_max_num);
  do
  {
    Eigen::Matrix<SCALAR,1,DIM_S::INER_MEAS_DIM,Eigen::RowMajor> inertial_meas = this->init<DIM_S::INER_MEAS_DIM>(this->inertial_meas_min_max_num);
    do
    {
      // Make numeric jacobian
      Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,Eigen::RowMajor> numeric_jacobian;
      Eigen::Index out_dim_it;
      Eigen::Index state_dim_it;

      const std::function<SCALAR(const SCALAR)> dim_func =
        [&nav_state,&inertial_meas,&out_dim_it,&state_dim_it,this] (const SCALAR x) -> SCALAR
        {
          // Set input
          Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,Eigen::RowMajor> input = nav_state;
          if((state_dim_it >= DIM_S::ERROR::EULER_START_IND) and (state_dim_it <= DIM_S::ERROR::EULER_END_IND))
          {
            Eigen::Matrix<SCALAR,1,3,Eigen::RowMajor> rpy = kf::math::quat::quaternionToRollPitchYaw(input.middleCols<4>(DIM_S::NAV::QUAT_START_IND));
            rpy[state_dim_it-DIM_S::ERROR::EULER_START_IND] = x;
            //const SCALAR diff = rpy[state_dim_it-DIM_S::ERROR::EULER_START_IND] - x;
            //rpy[state_dim_it-DIM_S::ERROR::EULER_START_IND] = rpy[state_dim_it-DIM_S::ERROR::EULER_START_IND] - (SCALAR(2) * diff);
            input.middleCols<4>(DIM_S::NAV::QUAT_START_IND) = kf::math::quat::rollPitchYawToQuaternion(rpy);
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
          const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,Eigen::RowMajor> reading = this->test_obj.getNavStateDynamics(this->time, input, Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,Eigen::RowMajor>::Zero(), inertial_meas);

          SCALAR output;
          if((out_dim_it >= DIM_S::ERROR::EULER_START_IND) and (out_dim_it <= DIM_S::ERROR::EULER_END_IND))
          {
            output = reading[out_dim_it+1];
          }
          else
          {
            if(out_dim_it >= DIM_S::ERROR::EULER_START_IND)
            {
              output = reading[out_dim_it+1];
            }
            else
            {
              output = reading[out_dim_it];
            }
          }

          return output;
        };

      for(state_dim_it = 0; state_dim_it < DIM_S::ERROR_DIM; ++state_dim_it)
      {
        for(out_dim_it = 0; out_dim_it < DIM_S::ERROR_DIM; ++out_dim_it)
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
      //numeric_jacobian.template middleRows<3>(DIM_S::ERROR::EULER_START_IND) *= SCALAR(2);
      //numeric_jacobian.template middleCols<3>(DIM_S::ERROR::EULER_START_IND).template middleRows<3>(DIM_S::ERROR::EULER_START_IND) *= SCALAR(2);
      //numeric_jacobian.template middleCols<3>(DIM_S::ERROR::EULER_START_IND) /= SCALAR(2);
      // Get analytic_jacobian
      Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,Eigen::RowMajor> analytic_jacobian =
        this->test_obj.getNavStateDynamicsPDWRErrorState(this->time, nav_state, Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,Eigen::RowMajor>::Zero(), inertial_meas);
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
        std::cout << "inertial_meas:\n" << inertial_meas << std::endl;
      }
      EXPECT_TRUE(is_equal);
    } while(this->inc<DIM_S::INER_MEAS_DIM>(inertial_meas, this->inertial_meas_min_max_num));
  } while(this->inc<DIM_S::ERROR_DIM>(nav_state, this->nav_state_min_max_num));
  std::cout << "Max Error Found: " << max_error_found << std::endl;
}

TEST_F(DubinsAirplaneTest, getNavStateDynamicsPDWRInertialMeasurement)
{
  SCALAR max_error_found = -std::numeric_limits<SCALAR>::infinity();
  Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,Eigen::RowMajor> nav_state = this->init<DIM_S::ERROR_DIM>(this->nav_state_min_max_num);
  do
  {
    Eigen::Matrix<SCALAR,1,DIM_S::INER_MEAS_DIM,Eigen::RowMajor> inertial_meas = this->init<DIM_S::INER_MEAS_DIM>(this->inertial_meas_min_max_num);
    do
    {
      // Make numeric jacobian
      Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::INER_MEAS_DIM,Eigen::RowMajor> numeric_jacobian;
      Eigen::Index out_dim_it;
      Eigen::Index state_dim_it;

      const std::function<SCALAR(const SCALAR)> dim_func =
        [&nav_state,&inertial_meas,&out_dim_it,&state_dim_it,this] (const SCALAR x) -> SCALAR
        {
          // Set input
          Eigen::Matrix<SCALAR,1,DIM_S::INER_MEAS_DIM,Eigen::RowMajor> input = inertial_meas;
          input[state_dim_it] = x;

          // Make reading
          const Eigen::Matrix<SCALAR,1,DIM_S::NAV_DIM,Eigen::RowMajor> reading = this->test_obj.getNavStateDynamics(this->time, nav_state, Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,Eigen::RowMajor>::Zero(), input);

          SCALAR output;
          if((out_dim_it >= DIM_S::ERROR::EULER_START_IND) and (out_dim_it <= DIM_S::ERROR::EULER_END_IND))
          {
            output = reading[out_dim_it+1];
          }
          else
          {
            if(out_dim_it >= DIM_S::ERROR::EULER_START_IND)
            {
              output = reading[out_dim_it+1];
            }
            else
            {
              output = reading[out_dim_it];
            }
          }

          return output;
        };

      for(state_dim_it = 0; state_dim_it < DIM_S::INER_MEAS_DIM; ++state_dim_it)
      {
        for(out_dim_it = 0; out_dim_it < DIM_S::ERROR_DIM; ++out_dim_it)
        {
          numeric_jacobian(out_dim_it,state_dim_it) = boost::math::differentiation::finite_difference_derivative<const std::function<SCALAR(SCALAR)>,SCALAR,numeric_diff_order>(
            dim_func, inertial_meas[state_dim_it]);
        }
      }
      //numeric_jacobian.template middleRows<3>(DIM_S::ERROR::EULER_START_IND) *= SCALAR(2);
      // Get analytic_jacobian
      Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::INER_MEAS_DIM,Eigen::RowMajor> analytic_jacobian =
        this->test_obj.getNavStateDynamicsPDWRInertialMeasurement(this->time, nav_state, Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,Eigen::RowMajor>::Zero(), inertial_meas).transpose();
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
        std::cout << "inertial_meas:\n" << inertial_meas << std::endl;
      }
      EXPECT_TRUE(is_equal);
    } while(this->inc<DIM_S::INER_MEAS_DIM>(inertial_meas, this->inertial_meas_min_max_num));
  } while(this->inc<DIM_S::ERROR_DIM>(nav_state, this->nav_state_min_max_num));
  std::cout << "Max Error Found: " << max_error_found << std::endl;
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

/* dubins_airplane_model_test.cpp */
