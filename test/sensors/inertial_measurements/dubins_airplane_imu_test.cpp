/**
 * @File: dubins_airplane_imu_test.cpp
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
#include<kalman_filter/sensors/inertial_measurements/dubins_airplane_imu.hpp>
#include<kalman_filter/dynamics/dubins_airplane_model.hpp>

using DIM_S = kf::dynamics::DubinsAirplaneDim;
using SCALAR = double;

class DubinsAirplaneIMUTest
 : public ::testing::Test
{
public:
  DubinsAirplaneIMUTest()
   : test_obj(9.81)
  {
    const SCALAR small_val = 1;
    for(Eigen::Index dim_ind = 0; dim_ind < DIM_S::TRUTH_DIM; ++dim_ind)
    {
      this->truth_state_min_max_num[dim_ind] = std::make_tuple(-small_val, small_val, 1);
    }
    std::get<0>(this->truth_state_min_max_num[DIM_S::TRUTH::AIR_SPEED_IND]) += 80;
    std::get<1>(this->truth_state_min_max_num[DIM_S::TRUTH::AIR_SPEED_IND]) += 80;
//    std::get<0>(this->truth_state_min_max_num[DIM_S::TRUTH::YAW_IND]) += 3.14/4.0;
//    std::get<1>(this->truth_state_min_max_num[DIM_S::TRUTH::YAW_IND]) += 3.14/4.0;
//    std::get<0>(this->truth_state_min_max_num[DIM_S::TRUTH::ROLL_IND]) += 3.14/4.0;
//    std::get<1>(this->truth_state_min_max_num[DIM_S::TRUTH::ROLL_IND]) += 3.14/4.0;
//    std::get<0>(this->truth_state_min_max_num[DIM_S::TRUTH::PITCH_IND]) += 3.14/4.0;
//    std::get<1>(this->truth_state_min_max_num[DIM_S::TRUTH::PITCH_IND]) += 3.14/4.0;
    for(Eigen::Index dim_ind = 0; dim_ind < DIM_S::CONTROL_DIM; ++dim_ind)
    {
      this->control_min_max_num[dim_ind] = std::make_tuple(-small_val, small_val, 1);
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
  inline SCALAR inc(Eigen::Ref<Eigen::Matrix<SCALAR,1,DIM,Eigen::RowMajor>> state,
                    const std::array<std::tuple<SCALAR,SCALAR,SCALAR>,DIM>& min_max_nums) const
  {
    for(Eigen::Index dim_ind = 0; dim_ind < DIM; ++dim_ind)
    {
      state[dim_ind] += this->findStep(min_max_nums[dim_ind]);
      if(std::get<1>(min_max_nums[dim_ind]) >= state[dim_ind])
      {
        return true;
      }
      state[dim_ind] = std::get<0>(min_max_nums[dim_ind]);
    }
    return false;
  }
  template<Eigen::Index DIM>
  inline Eigen::Matrix<SCALAR,1,DIM,Eigen::RowMajor> init(const std::array<std::tuple<SCALAR,SCALAR,SCALAR>,DIM>& min_max_nums) const
  {
    Eigen::Matrix<SCALAR,1,DIM,Eigen::RowMajor> output;
    for(Eigen::Index dim_ind = 0; dim_ind < DIM; ++dim_ind)
    {
      output[dim_ind] = std::get<0>(min_max_nums[dim_ind]);
    }
    return output;
  }

  inline static constexpr const SCALAR output_error_eps   = 1e-6;
  inline static constexpr const size_t numeric_diff_order = 8;

  kf::sensors::DubinsAirplaneIMU<DIM_S,SCALAR,Eigen::RowMajor>            test_obj;
  inline static constexpr const SCALAR                                    time = std::numeric_limits<SCALAR>::quiet_NaN();
  std::array<std::tuple<SCALAR,SCALAR,SCALAR>,DIM_S::TRUTH_DIM>           truth_state_min_max_num;
  std::array<std::tuple<SCALAR,SCALAR,SCALAR>,DIM_S::CONTROL_DIM>         control_min_max_num;
  std::array<std::tuple<SCALAR,SCALAR,SCALAR>,DIM_S::INER_MEAS_NOISE_DIM> inertial_noise_min_max_num;
};

TEST_F(DubinsAirplaneIMUTest, getMeasurement)
{
  Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,Eigen::RowMajor> truth_state = this->init<DIM_S::TRUTH_DIM>(this->truth_state_min_max_num);
  do
  {
    Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,Eigen::RowMajor> control = this->init<DIM_S::CONTROL_DIM>(this->control_min_max_num);
    do
    {
      Eigen::Matrix<SCALAR,1,DIM_S::INER_MEAS_NOISE_DIM,Eigen::RowMajor> noise = this->init<DIM_S::INER_MEAS_NOISE_DIM>(this->inertial_noise_min_max_num);
      do
      {
        const Eigen::Matrix<SCALAR,1,DIM_S::INER_MEAS_DIM,Eigen::RowMajor> output =
          this->test_obj.getMeasurement(this->time, truth_state, control, noise);

        EXPECT_FALSE(output.hasNaN());
        EXPECT_TRUE(output.allFinite());
      } while(this->inc<DIM_S::INER_MEAS_NOISE_DIM>(noise, this->inertial_noise_min_max_num));
    } while(this->inc<DIM_S::CONTROL_DIM>(control, this->control_min_max_num));
  } while(this->inc<DIM_S::TRUTH_DIM>(truth_state, this->truth_state_min_max_num));
}

TEST_F(DubinsAirplaneIMUTest, getMeasurementPDWRDispersionState)
{
  SCALAR max_error_found = -std::numeric_limits<SCALAR>::infinity();
  Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,Eigen::RowMajor> truth_state = this->init<DIM_S::TRUTH_DIM>(this->truth_state_min_max_num);
  do
  {
    Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,Eigen::RowMajor> control = this->init<DIM_S::CONTROL_DIM>(this->control_min_max_num);
    do
    {
      Eigen::Matrix<SCALAR,1,DIM_S::INER_MEAS_NOISE_DIM,Eigen::RowMajor> noise = this->init<DIM_S::INER_MEAS_NOISE_DIM>(this->inertial_noise_min_max_num);
      //do
      //{
        // Make numeric jacobian
        Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,DIM_S::TRUTH_DISP_DIM,Eigen::RowMajor> numeric_jacobian;
        Eigen::Index meas_dim_it;
        Eigen::Index truth_disp_dim_it;

        const std::function<SCALAR(const SCALAR)> dim_func =
          [&truth_state,&control,&noise,&meas_dim_it,&truth_disp_dim_it,this] (const SCALAR x) -> SCALAR
          {
            // Set input
            Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,Eigen::RowMajor> input_truth_state = truth_state;
            input_truth_state[truth_disp_dim_it] = x;

            // Make reading
            const Eigen::Matrix<SCALAR,1,DIM_S::INER_MEAS_DIM,Eigen::RowMajor> reading = this->test_obj.getMeasurement(this->time, input_truth_state, control, noise);

            return reading[meas_dim_it];
          };

        for(truth_disp_dim_it = 0; truth_disp_dim_it < DIM_S::TRUTH_DISP_DIM; ++truth_disp_dim_it)
        {
          for(meas_dim_it = 0; meas_dim_it < DIM_S::INER_MEAS_DIM; ++meas_dim_it)
          {
            numeric_jacobian(meas_dim_it,truth_disp_dim_it) = boost::math::differentiation::finite_difference_derivative<const std::function<SCALAR(SCALAR)>,SCALAR,numeric_diff_order>(
              dim_func, truth_state[truth_disp_dim_it]);
          }
        }
        // Get analytic_jacobian
        Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,DIM_S::TRUTH_DISP_DIM,Eigen::RowMajor> analytic_jacobian =
          this->test_obj.getMeasurementPDWRDispersionState(this->time, truth_state, control);
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
          std::cout << "noise:\n" << noise << std::endl;
        }
        EXPECT_TRUE(is_equal);
      //} while(this->inc<DIM_S::INER_MEAS_NOISE_DIM>(noise, this->inertial_noise_min_max_num));
    } while(this->inc<DIM_S::CONTROL_DIM>(control, this->control_min_max_num));
  } while(this->inc<DIM_S::TRUTH_DIM>(truth_state, this->truth_state_min_max_num));
  std::cout << "Max Error Found: " << max_error_found << std::endl;
}

TEST_F(DubinsAirplaneIMUTest, getMeasurementPDWRControl)
{
  SCALAR max_error_found = -std::numeric_limits<SCALAR>::infinity();
  Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,Eigen::RowMajor> truth_state = this->init<DIM_S::TRUTH_DIM>(this->truth_state_min_max_num);
  do
  {
    Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,Eigen::RowMajor> control = this->init<DIM_S::CONTROL_DIM>(this->control_min_max_num);
    do
    {
      Eigen::Matrix<SCALAR,1,DIM_S::INER_MEAS_NOISE_DIM,Eigen::RowMajor> noise = this->init<DIM_S::INER_MEAS_NOISE_DIM>(this->inertial_noise_min_max_num);
      do
      {
        // Make numeric jacobian
        Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,DIM_S::CONTROL_DIM,Eigen::RowMajor> numeric_jacobian;
        Eigen::Index meas_dim_it;
        Eigen::Index control_dim_it;

        const std::function<SCALAR(const SCALAR)> dim_func =
          [&truth_state,&control,&noise,&meas_dim_it,&control_dim_it,this] (const SCALAR x) -> SCALAR
          {
            // Set input
            Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,Eigen::RowMajor> input_control = control;
            input_control[control_dim_it] = x;

            // Make reading
            const Eigen::Matrix<SCALAR,1,DIM_S::INER_MEAS_DIM,Eigen::RowMajor> reading = this->test_obj.getMeasurement(this->time, truth_state, input_control, noise);

            return reading[meas_dim_it];
          };

        for(control_dim_it = 0; control_dim_it < DIM_S::CONTROL_DIM; ++control_dim_it)
        {
          for(meas_dim_it = 0; meas_dim_it < DIM_S::INER_MEAS_DIM; ++meas_dim_it)
          {
            numeric_jacobian(meas_dim_it,control_dim_it) = boost::math::differentiation::finite_difference_derivative<const std::function<SCALAR(SCALAR)>,SCALAR,numeric_diff_order>(
              dim_func, control[control_dim_it]);
          }
        }
        // Get analytic_jacobian
        Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,DIM_S::CONTROL_DIM,Eigen::RowMajor> analytic_jacobian =
          this->test_obj.getMeasurementPDWRControl(this->time, truth_state, control);
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
          std::cout << "noise:\n" << noise << std::endl;
        }
        EXPECT_TRUE(is_equal);
      } while(this->inc<DIM_S::INER_MEAS_NOISE_DIM>(noise, this->inertial_noise_min_max_num));
    } while(this->inc<DIM_S::CONTROL_DIM>(control, this->control_min_max_num));
  } while(this->inc<DIM_S::TRUTH_DIM>(truth_state, this->truth_state_min_max_num));
  std::cout << "Max Error Found: " << max_error_found << std::endl;
}

TEST_F(DubinsAirplaneIMUTest, getMeasurementPDWRNoise)
{
  SCALAR max_error_found = -std::numeric_limits<SCALAR>::infinity();
  Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DIM,Eigen::RowMajor> truth_state = this->init<DIM_S::TRUTH_DIM>(this->truth_state_min_max_num);
  do
  {
    Eigen::Matrix<SCALAR,1,DIM_S::CONTROL_DIM,Eigen::RowMajor> control = this->init<DIM_S::CONTROL_DIM>(this->control_min_max_num);
    do
    {
      Eigen::Matrix<SCALAR,1,DIM_S::INER_MEAS_NOISE_DIM,Eigen::RowMajor> noise = this->init<DIM_S::INER_MEAS_NOISE_DIM>(this->inertial_noise_min_max_num);
      do
      {
        // Make numeric jacobian
        Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,DIM_S::INER_MEAS_NOISE_DIM,Eigen::RowMajor> numeric_jacobian;
        Eigen::Index meas_dim_it;
        Eigen::Index noise_dim_it;

        const std::function<SCALAR(const SCALAR)> dim_func =
          [&truth_state,&control,&noise,&meas_dim_it,&noise_dim_it,this] (const SCALAR x) -> SCALAR
          {
            // Set input
            Eigen::Matrix<SCALAR,1,DIM_S::INER_MEAS_NOISE_DIM,Eigen::RowMajor> input_noise = noise;
            input_noise[noise_dim_it] = x;

            // Make reading
            const Eigen::Matrix<SCALAR,1,DIM_S::INER_MEAS_DIM,Eigen::RowMajor> reading = this->test_obj.getMeasurement(this->time, truth_state, control, input_noise);

            return reading[meas_dim_it];
          };

        for(noise_dim_it = 0; noise_dim_it < DIM_S::INER_MEAS_NOISE_DIM; ++noise_dim_it)
        {
          for(meas_dim_it = 0; meas_dim_it < DIM_S::INER_MEAS_DIM; ++meas_dim_it)
          {
            numeric_jacobian(meas_dim_it,noise_dim_it) = boost::math::differentiation::finite_difference_derivative<const std::function<SCALAR(SCALAR)>,SCALAR,numeric_diff_order>(
              dim_func, noise[noise_dim_it]);
          }
        }
        // Get analytic_jacobian
        Eigen::Matrix<SCALAR,DIM_S::INER_MEAS_DIM,DIM_S::INER_MEAS_NOISE_DIM,Eigen::RowMajor> analytic_jacobian =
          this->test_obj.getMeasurementPDWRNoise(this->time, truth_state, control);
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
          std::cout << "noise:\n" << noise << std::endl;
        }
        EXPECT_TRUE(is_equal);
      } while(this->inc<DIM_S::INER_MEAS_NOISE_DIM>(noise, this->inertial_noise_min_max_num));
    } while(this->inc<DIM_S::CONTROL_DIM>(control, this->control_min_max_num));
  } while(this->inc<DIM_S::TRUTH_DIM>(truth_state, this->truth_state_min_max_num));
  std::cout << "Max Error Found: " << max_error_found << std::endl;
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

/* dubins_airplane_imu_test.cpp */
