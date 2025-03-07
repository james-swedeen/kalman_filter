/**
 * @File: feature_bearing_test.cpp
 * @Date: October 2022
 * @Author: James Swedeen
 **/

/* Testing Headers */
#include<gtest/gtest.h>

/* Boost Headers */
#include<boost/math/differentiation/finite_difference.hpp>

/* Local Headers */
#include<kalman_filter/helpers/dimension_struct.hpp>
#include<kalman_filter/math/quaternion.hpp>
#include<kalman_filter/sensors/measurements/feature_bearing.hpp>

using FLOAT = double;
//using FLOAT = long double;

// Testing class
class FeatureBearingTest : public ::testing::Test
{
public:
  struct FeatureDim
   : public kf::Dimensions<0,7,7,0,6,6,0,0,0,0,false>
  {
  public:
    struct TRUTH
    {
    public:
      inline static constexpr const Eigen::Index NORTH_IND                      = 0;
      inline static constexpr const Eigen::Index EAST_IND                       = 1;
      inline static constexpr const Eigen::Index DOWN_IND                       = 2;
      inline static constexpr const Eigen::Index QUAT_W_IND                     = 3;
      inline static constexpr const Eigen::Index QUAT_X_IND                     = 4;
      inline static constexpr const Eigen::Index QUAT_Y_IND                     = 5;
      inline static constexpr const Eigen::Index QUAT_Z_IND                     = 6;
      inline static constexpr const Eigen::Index FEATURE_BEARING_ROLL_BIAS_IND  = 7;
      inline static constexpr const Eigen::Index FEATURE_BEARING_PITCH_BIAS_IND = 8;
      inline static constexpr const Eigen::Index FEATURE_BEARING_YAW_BIAS_IND   = 9;

      inline static constexpr const Eigen::Index POS_START_IND                  = NORTH_IND;
      inline static constexpr const Eigen::Index POS_END_IND                    = DOWN_IND;
      inline static constexpr const Eigen::Index QUAT_START_IND                 = QUAT_W_IND;
      inline static constexpr const Eigen::Index QUAT_END_IND                   = QUAT_Z_IND;
      inline static constexpr const Eigen::Index FEATURE_BEARING_BIAS_START_IND = FEATURE_BEARING_ROLL_BIAS_IND;
      inline static constexpr const Eigen::Index FEATURE_BEARING_BIAS_END_IND   = FEATURE_BEARING_YAW_BIAS_IND;
    };
    struct NAV
    {
    public:
      inline static constexpr const Eigen::Index NORTH_IND                      = 0;
      inline static constexpr const Eigen::Index EAST_IND                       = 1;
      inline static constexpr const Eigen::Index DOWN_IND                       = 2;
      inline static constexpr const Eigen::Index QUAT_W_IND                     = 3;
      inline static constexpr const Eigen::Index QUAT_X_IND                     = 4;
      inline static constexpr const Eigen::Index QUAT_Y_IND                     = 5;
      inline static constexpr const Eigen::Index QUAT_Z_IND                     = 6;
      inline static constexpr const Eigen::Index FEATURE_BEARING_ROLL_BIAS_IND  = 7;
      inline static constexpr const Eigen::Index FEATURE_BEARING_PITCH_BIAS_IND = 8;
      inline static constexpr const Eigen::Index FEATURE_BEARING_YAW_BIAS_IND   = 9;

      inline static constexpr const Eigen::Index POS_START_IND                  = NORTH_IND;
      inline static constexpr const Eigen::Index POS_END_IND                    = DOWN_IND;
      inline static constexpr const Eigen::Index QUAT_START_IND                 = QUAT_W_IND;
      inline static constexpr const Eigen::Index QUAT_END_IND                   = QUAT_Z_IND;
      inline static constexpr const Eigen::Index FEATURE_BEARING_BIAS_START_IND = FEATURE_BEARING_ROLL_BIAS_IND;
      inline static constexpr const Eigen::Index FEATURE_BEARING_BIAS_END_IND   = FEATURE_BEARING_YAW_BIAS_IND;
    };
    struct ERROR
    {
    public:
      inline static constexpr const Eigen::Index NORTH_IND                      = 0;
      inline static constexpr const Eigen::Index EAST_IND                       = 1;
      inline static constexpr const Eigen::Index DOWN_IND                       = 2;
      inline static constexpr const Eigen::Index ROLL_IND                       = 3;
      inline static constexpr const Eigen::Index PITCH_IND                      = 4;
      inline static constexpr const Eigen::Index YAW_IND                        = 5;
      inline static constexpr const Eigen::Index FEATURE_BEARING_ROLL_BIAS_IND  = 6;
      inline static constexpr const Eigen::Index FEATURE_BEARING_PITCH_BIAS_IND = 7;
      inline static constexpr const Eigen::Index FEATURE_BEARING_YAW_BIAS_IND   = 8;

      inline static constexpr const Eigen::Index POS_START_IND                  = NORTH_IND;
      inline static constexpr const Eigen::Index POS_END_IND                    = DOWN_IND;
      inline static constexpr const Eigen::Index EULER_START_IND                = ROLL_IND;
      inline static constexpr const Eigen::Index EULER_END_IND                  = YAW_IND;
      inline static constexpr const Eigen::Index FEATURE_BEARING_BIAS_START_IND = FEATURE_BEARING_ROLL_BIAS_IND;
      inline static constexpr const Eigen::Index FEATURE_BEARING_BIAS_END_IND   = FEATURE_BEARING_YAW_BIAS_IND;
    };
    struct TRUTH_DISP
    {
    public:
      inline static constexpr const Eigen::Index NORTH_IND                      = 0;
      inline static constexpr const Eigen::Index EAST_IND                       = 1;
      inline static constexpr const Eigen::Index DOWN_IND                       = 2;
      inline static constexpr const Eigen::Index ROLL_IND                       = 3;
      inline static constexpr const Eigen::Index PITCH_IND                      = 4;
      inline static constexpr const Eigen::Index YAW_IND                        = 5;
      inline static constexpr const Eigen::Index FEATURE_BEARING_ROLL_BIAS_IND  = 6;
      inline static constexpr const Eigen::Index FEATURE_BEARING_PITCH_BIAS_IND = 7;
      inline static constexpr const Eigen::Index FEATURE_BEARING_YAW_BIAS_IND   = 8;

      inline static constexpr const Eigen::Index POS_START_IND                  = NORTH_IND;
      inline static constexpr const Eigen::Index POS_END_IND                    = DOWN_IND;
      inline static constexpr const Eigen::Index EULER_START_IND                = ROLL_IND;
      inline static constexpr const Eigen::Index EULER_END_IND                  = YAW_IND;
      inline static constexpr const Eigen::Index FEATURE_BEARING_BIAS_START_IND = FEATURE_BEARING_ROLL_BIAS_IND;
      inline static constexpr const Eigen::Index FEATURE_BEARING_BIAS_END_IND   = FEATURE_BEARING_YAW_BIAS_IND;
    };
  };

  inline static constexpr const FLOAT  output_error_eps   = 0.005;
  inline static constexpr const size_t numeric_diff_order = 4;

  inline static constexpr const FLOAT step_size_linear        = 10;
  inline static constexpr const FLOAT step_size_angular       = kf::math::pi<FLOAT>()/FLOAT(32);
  inline static constexpr const FLOAT uav_position_bounds     = 10;
  inline static constexpr const FLOAT feature_position_bounds = 1;
  inline static constexpr const FLOAT bias_bounds             = 0.01; // kf::math::pi<FLOAT>()/FLOAT(16);

  Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> camera_offset;
  Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> camera_viewing_angles;

  FeatureBearingTest()
  {
    this->camera_offset[0] = 0;
    this->camera_offset[1] = 0;
    this->camera_offset[2] = 0;

    this->camera_viewing_angles[0] = 0;
    this->camera_viewing_angles[1] = kf::math::pi<FLOAT>()/FLOAT(4);
    this->camera_viewing_angles[2] = 0;
  }
};

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

TEST_F(FeatureBearingTest, LOS_VEC_DIFF)
{
  Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> feature_loc;

  for(feature_loc[0] = -this->feature_position_bounds; feature_loc[0] < this->feature_position_bounds; feature_loc[0] += this->step_size_linear)
  {
  for(feature_loc[1] = -this->feature_position_bounds; feature_loc[1] < this->feature_position_bounds; feature_loc[1] += this->step_size_linear)
  {
  //for(feature_loc[2] = -this->feature_position_bounds; feature_loc[2] < this->feature_position_bounds; feature_loc[2] += this->step_size_linear)
  feature_loc[2] = 0;
  {
    const Eigen::Matrix<FLOAT,3,3,Eigen::RowMajor> camera_frame_rotation =
      kf::math::quat::rollPitchYawToDirectionCosineMatrix(this->camera_viewing_angles[0],
                                                          this->camera_viewing_angles[1],
                                                          this->camera_viewing_angles[2]);

    Eigen::Matrix<FLOAT,1,10,Eigen::RowMajor> nav_state;

    for(nav_state[FeatureDim::NAV::NORTH_IND] = -this->uav_position_bounds; nav_state[FeatureDim::NAV::NORTH_IND] < this->uav_position_bounds; nav_state[FeatureDim::NAV::NORTH_IND] += this->step_size_linear)
    {
    for(nav_state[FeatureDim::NAV::EAST_IND] = -this->uav_position_bounds; nav_state[FeatureDim::NAV::EAST_IND] < this->uav_position_bounds; nav_state[FeatureDim::NAV::EAST_IND] += this->step_size_linear)
    {
//    for(nav_state[FeatureDim::NAV::DOWN_IND] = -this->uav_position_bounds; nav_state[FeatureDim::NAV::DOWN_IND] < this->uav_position_bounds; nav_state[FeatureDim::NAV::DOWN_IND] += this->step_size_linear)
    nav_state[FeatureDim::NAV::DOWN_IND] = -3500;
    {
    for(FLOAT roll = -this->bias_bounds; roll < this->bias_bounds; roll += this->step_size_angular)
    {
    for(FLOAT pitch = -this->bias_bounds; pitch < this->bias_bounds; pitch += this->step_size_angular)
    {
    for(FLOAT yaw = -this->bias_bounds; yaw < this->bias_bounds; yaw += this->step_size_angular)
    {
      nav_state.middleCols<4>(FeatureDim::NAV::QUAT_START_IND) = kf::math::quat::rollPitchYawToQuaternion(Eigen::Matrix<FLOAT,1,1,Eigen::RowMajor>(roll),
                                                                                                          Eigen::Matrix<FLOAT,1,1,Eigen::RowMajor>(pitch),
                                                                                                          Eigen::Matrix<FLOAT,1,1,Eigen::RowMajor>(yaw));
//    for(nav_state[FeatureDim::NAV::FEATURE_BEARING_ROLL_BIAS_IND] = -this->bias_bounds; nav_state[FeatureDim::NAV::FEATURE_BEARING_ROLL_BIAS_IND] < this->bias_bounds; nav_state[FeatureDim::NAV::FEATURE_BEARING_ROLL_BIAS_IND] += this->step_size_angular)
    {
//    for(nav_state[FeatureDim::NAV::FEATURE_BEARING_PITCH_BIAS_IND] = -this->bias_bounds; nav_state[FeatureDim::NAV::FEATURE_BEARING_PITCH_BIAS_IND] < this->bias_bounds; nav_state[FeatureDim::NAV::FEATURE_BEARING_PITCH_BIAS_IND] += this->step_size_angular)
    {
//    for(nav_state[FeatureDim::NAV::FEATURE_BEARING_YAW_BIAS_IND] = -this->bias_bounds; nav_state[FeatureDim::NAV::FEATURE_BEARING_YAW_BIAS_IND] < this->bias_bounds; nav_state[FeatureDim::NAV::FEATURE_BEARING_YAW_BIAS_IND] += this->step_size_angular)
    {
      nav_state.rightCols<3>().setZero();
      Eigen::Matrix<FLOAT,3,9,Eigen::RowMajor> numeric_jacobian;
      Eigen::Index nav_state_dim_it;
      Eigen::Index reading_dim_it;

      const std::function<FLOAT(const FLOAT)> dim_func =
        [&nav_state,&nav_state_dim_it,&reading_dim_it,&camera_frame_rotation,&feature_loc,this] (const FLOAT x) -> FLOAT
        {
          // Set input
          Eigen::Matrix<FLOAT,1,10,Eigen::RowMajor> input_nav_state = nav_state;
          if((nav_state_dim_it >= FeatureDim::ERROR::EULER_START_IND) and (nav_state_dim_it <= FeatureDim::ERROR::EULER_END_IND))
          {
            Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> rpy = kf::math::quat::quaternionToRollPitchYaw(input_nav_state.middleCols<4>(FeatureDim::NAV::QUAT_START_IND));
            rpy[nav_state_dim_it-FeatureDim::ERROR::EULER_START_IND] = x;
            input_nav_state.middleCols<4>(FeatureDim::NAV::QUAT_START_IND) =
              kf::math::quat::normalize(kf::math::quat::rollPitchYawToQuaternion(rpy));
          }
          else
          {
            if(nav_state_dim_it >= FeatureDim::ERROR::EULER_START_IND)
            {
              input_nav_state[nav_state_dim_it+1] = x;
            }
            else
            {
              input_nav_state[nav_state_dim_it] = x;
            }
          }
          // Make reading
          const Eigen::Matrix<FLOAT,3,3,Eigen::RowMajor> rot_ned_body  = kf::math::quat::quaternionToDirectionCosineMatrix(input_nav_state.template middleCols<4>(FeatureDim::NAV::QUAT_START_IND)).transpose();
          const Eigen::Matrix<FLOAT,3,3,Eigen::RowMajor> rot_cam_bias  = kf::math::quat::rollPitchYawToDirectionCosineMatrix(input_nav_state[FeatureDim::NAV::FEATURE_BEARING_ROLL_BIAS_IND],
                                                                                                                             input_nav_state[FeatureDim::NAV::FEATURE_BEARING_PITCH_BIAS_IND],
                                                                                                                             input_nav_state[FeatureDim::NAV::FEATURE_BEARING_YAW_BIAS_IND]);
          const Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> diff_vec_ned  = feature_loc - input_nav_state.template middleCols<3>(FeatureDim::NAV::POS_START_IND);
          const Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> diff_vec_body = rot_ned_body * diff_vec_ned.transpose();
          const Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> rel_los_vec   = rot_cam_bias * camera_frame_rotation * (diff_vec_body - this->camera_offset).transpose();

          return rel_los_vec[reading_dim_it];
        };

      for(nav_state_dim_it = 0; nav_state_dim_it < 9; ++nav_state_dim_it)
      {
      for(reading_dim_it = 0; reading_dim_it < 3; ++reading_dim_it)
      {
        FLOAT input_val;
        if((nav_state_dim_it >= FeatureDim::ERROR::EULER_START_IND) and (nav_state_dim_it <= FeatureDim::ERROR::EULER_END_IND))
        {
          const Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> rpy = kf::math::quat::quaternionToRollPitchYaw(nav_state.middleCols<4>(FeatureDim::NAV::QUAT_START_IND));
          input_val = rpy[nav_state_dim_it-FeatureDim::ERROR::EULER_START_IND];
        }
        else
        {
          if(nav_state_dim_it >= FeatureDim::ERROR::EULER_START_IND)
          {
            input_val = nav_state[nav_state_dim_it+1];
          }
          else
          {
            input_val = nav_state[nav_state_dim_it];
          }
        }
        numeric_jacobian(reading_dim_it,nav_state_dim_it) = boost::math::differentiation::finite_difference_derivative<const std::function<FLOAT(FLOAT)>,FLOAT,numeric_diff_order>(
          dim_func, input_val);
      }
      }
      // Make analytic_jacobian
      const Eigen::Matrix<FLOAT,3,3,Eigen::RowMajor> rot_ned_body  = kf::math::quat::quaternionToDirectionCosineMatrix(nav_state.template middleCols<4>(FeatureDim::NAV::QUAT_START_IND)).transpose();
      const Eigen::Matrix<FLOAT,3,3,Eigen::RowMajor> rot_cam_bias  = kf::math::quat::rollPitchYawToDirectionCosineMatrix(    nav_state[FeatureDim::NAV::FEATURE_BEARING_ROLL_BIAS_IND],
                                                                                                                             nav_state[FeatureDim::NAV::FEATURE_BEARING_PITCH_BIAS_IND],
                                                                                                                             nav_state[FeatureDim::NAV::FEATURE_BEARING_YAW_BIAS_IND]);
      const Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> diff_vec_ned  = feature_loc - nav_state.template middleCols<3>(FeatureDim::NAV::POS_START_IND);
      const Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> diff_vec_body = rot_ned_body * diff_vec_ned.transpose();
      const Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> rel_los_vec   = rot_cam_bias * camera_frame_rotation * (diff_vec_body - this->camera_offset).transpose();
      Eigen::Matrix<FLOAT,3,9,Eigen::RowMajor> analytic_jacobian;
      analytic_jacobian.setZero();
      analytic_jacobian.template middleCols<3>(FeatureDim::ERROR::POS_START_IND) = -rot_cam_bias * camera_frame_rotation * rot_ned_body;
      analytic_jacobian.template middleCols<3>(FeatureDim::ERROR::EULER_START_IND) = rot_cam_bias * camera_frame_rotation * rot_ned_body * kf::math::crossProductMatrix(diff_vec_ned);
      analytic_jacobian.template middleCols<3>(FeatureDim::ERROR::FEATURE_BEARING_BIAS_START_IND) = kf::math::crossProductMatrix(rot_cam_bias * camera_frame_rotation * (diff_vec_body - this->camera_offset).transpose()).transpose();

      const bool is_equal = ((numeric_jacobian - analytic_jacobian).array().abs() < this->output_error_eps).all();
      EXPECT_TRUE(is_equal);
      if(not is_equal)
      {
        std::cout << "numeric_jacobian:\n" << numeric_jacobian << std::endl;
        std::cout << "analytic_jacobian:\n" << analytic_jacobian << std::endl;
        std::cout << "diff:\n" << numeric_jacobian - analytic_jacobian << std::endl;
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
    }
    }
  }
  }
  }
}

TEST_F(FeatureBearingTest, getMeasurementEstimatePDWRErrorState)
{
  return;
  Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> feature_loc;

  for(feature_loc[0] = -this->feature_position_bounds; feature_loc[0] < this->feature_position_bounds; feature_loc[0] += this->step_size_linear)
  {
  for(feature_loc[1] = -this->feature_position_bounds; feature_loc[1] < this->feature_position_bounds; feature_loc[1] += this->step_size_linear)
  {
  //for(feature_loc[2] = -this->feature_position_bounds; feature_loc[2] < this->feature_position_bounds; feature_loc[2] += this->step_size_linear)
  feature_loc[2] = 0;
  {
    kf::sensors::FeatureBearing<FeatureDim,true,false,FLOAT,Eigen::RowMajor> test_model(
      std::numeric_limits<FLOAT>::quiet_NaN(),
      this->camera_offset,
      this->camera_viewing_angles,
      std::numeric_limits<FLOAT>::quiet_NaN(),
      feature_loc);

    Eigen::Matrix<FLOAT,1,7,Eigen::RowMajor> nav_state;

    for(nav_state[FeatureDim::NAV::NORTH_IND] = -this->uav_position_bounds; nav_state[FeatureDim::NAV::NORTH_IND] < this->uav_position_bounds; nav_state[FeatureDim::NAV::NORTH_IND] += this->step_size_linear)
    {
    for(nav_state[FeatureDim::NAV::EAST_IND] = -this->uav_position_bounds; nav_state[FeatureDim::NAV::EAST_IND] < this->uav_position_bounds; nav_state[FeatureDim::NAV::EAST_IND] += this->step_size_linear)
    {
//    for(nav_state[FeatureDim::NAV::DOWN_IND] = -this->uav_position_bounds; nav_state[FeatureDim::NAV::DOWN_IND] < this->uav_position_bounds; nav_state[FeatureDim::NAV::DOWN_IND] += this->step_size_linear)
    nav_state[FeatureDim::NAV::DOWN_IND] = -500;
    {
    for(FLOAT roll = -this->bias_bounds; roll < this->bias_bounds; roll += this->step_size_angular)
    {
    for(FLOAT pitch = -this->bias_bounds; pitch < this->bias_bounds; pitch += this->step_size_angular)
    {
    for(FLOAT yaw = -this->bias_bounds; yaw < this->bias_bounds; yaw += this->step_size_angular)
    {
      nav_state.middleCols<4>(FeatureDim::NAV::QUAT_START_IND) = kf::math::quat::rollPitchYawToQuaternion(Eigen::Matrix<FLOAT,1,1,Eigen::RowMajor>(roll),
                                                                                                          Eigen::Matrix<FLOAT,1,1,Eigen::RowMajor>(pitch),
                                                                                                          Eigen::Matrix<FLOAT,1,1,Eigen::RowMajor>(yaw));
/*    for(nav_state[FeatureDim::NAV::FEATURE_BEARING_ROLL_BIAS_IND] = -this->bias_bounds; nav_state[FeatureDim::NAV::FEATURE_BEARING_ROLL_BIAS_IND] < this->bias_bounds; nav_state[FeatureDim::NAV::FEATURE_BEARING_ROLL_BIAS_IND] += this->step_size_angular)
    {
    for(nav_state[FeatureDim::NAV::FEATURE_BEARING_PITCH_BIAS_IND] = -this->bias_bounds; nav_state[FeatureDim::NAV::FEATURE_BEARING_PITCH_BIAS_IND] < this->bias_bounds; nav_state[FeatureDim::NAV::FEATURE_BEARING_PITCH_BIAS_IND] += this->step_size_angular)
    {
    for(nav_state[FeatureDim::NAV::FEATURE_BEARING_YAW_BIAS_IND] = -this->bias_bounds; nav_state[FeatureDim::NAV::FEATURE_BEARING_YAW_BIAS_IND] < this->bias_bounds; nav_state[FeatureDim::NAV::FEATURE_BEARING_YAW_BIAS_IND] += this->step_size_angular)
    {*/
      Eigen::Matrix<FLOAT,2,6,Eigen::RowMajor> numeric_jacobian;
      Eigen::Index nav_state_dim_it;
      Eigen::Index reading_dim_it;

      const std::function<FLOAT(const FLOAT)> dim_func =
        [&nav_state,&nav_state_dim_it,&reading_dim_it,&test_model] (const FLOAT x) -> FLOAT
        {
          Eigen::Matrix<FLOAT,1,7,Eigen::RowMajor> input_nav_state = nav_state;
          if((nav_state_dim_it >= FeatureDim::ERROR::EULER_START_IND) and (nav_state_dim_it <= FeatureDim::ERROR::EULER_END_IND))
          {
            Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> rpy = kf::math::quat::quaternionToRollPitchYaw(input_nav_state.middleCols<4>(FeatureDim::NAV::QUAT_START_IND));
            rpy[nav_state_dim_it-FeatureDim::ERROR::EULER_START_IND] = x;
            input_nav_state.middleCols<4>(FeatureDim::NAV::QUAT_START_IND) =
              kf::math::quat::normalize(kf::math::quat::rollPitchYawToQuaternion(rpy));
          }
          else
          {
            if(nav_state_dim_it >= FeatureDim::ERROR::EULER_START_IND)
            {
              input_nav_state[nav_state_dim_it+1] = x;
            }
            else
            {
              input_nav_state[nav_state_dim_it] = x;
            }
          }
          return test_model.estimateMeasurement(0, input_nav_state)[reading_dim_it];
        };

      for(nav_state_dim_it = 0; nav_state_dim_it < 6; ++nav_state_dim_it)
      {
      for(reading_dim_it = 0; reading_dim_it < 2; ++reading_dim_it)
      {
        FLOAT input_val;
        if((nav_state_dim_it >= FeatureDim::ERROR::EULER_START_IND) and (nav_state_dim_it <= FeatureDim::ERROR::EULER_END_IND))
        {
          const Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> rpy = kf::math::quat::quaternionToRollPitchYaw(nav_state.middleCols<4>(FeatureDim::NAV::QUAT_START_IND));
          input_val = rpy[nav_state_dim_it-FeatureDim::ERROR::EULER_START_IND];
        }
        else
        {
          if(nav_state_dim_it >= FeatureDim::ERROR::EULER_START_IND)
          {
            input_val = nav_state[nav_state_dim_it+1];
          }
          else
          {
            input_val = nav_state[nav_state_dim_it];
          }
        }
        numeric_jacobian(reading_dim_it,nav_state_dim_it) = boost::math::differentiation::finite_difference_derivative<const std::function<FLOAT(FLOAT)>,FLOAT,numeric_diff_order>(
          dim_func, input_val);
      }
      }

      const Eigen::Matrix<FLOAT,2,6,Eigen::RowMajor> analytic_jacobian = test_model.getMeasurementEstimatePDWRErrorState(0, nav_state);

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
//    }
//    }
//    }
    }
    }
    }
    }
    }
    }
  }
  }
  }
}

/* feature_bearing_test.cpp */
