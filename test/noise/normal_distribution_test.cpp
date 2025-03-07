/**
 * @File: normal_distribution_test.cpp
 * @Date: April 2022
 * @Author: James Swedeen
 **/

/* Testing Headers */
#include<gtest/gtest.h>

/* Local Headers */
#include<kalman_filter/noise/normal_distribution.hpp>

// Testing class
class NormalDistributionTest : public ::testing::Test
{
public:
  NormalDistributionTest()
  {
    srand((unsigned int) time(0));

    this->covariance.setZero();
    for(Eigen::Index it = 0; it < NUM_DIM; ++it)
    {
      this->mean[it]          =          std::fmod((double)rand(), this->bounds);
      this->covariance(it,it) = std::abs(std::fmod((double)rand(), this->bounds)) + 0.0001;
    }
  }

  inline static constexpr const Eigen::Index NUM_DIM          = 3;
  inline static constexpr const Eigen::Index bounds           = 3;
  inline static constexpr const Eigen::Index number_of_runs   = 100000000;
  inline static constexpr const double       output_error_eps = 1e-3;

  Eigen::Matrix<double,1,      NUM_DIM,Eigen::RowMajor> mean;
  Eigen::Matrix<double,NUM_DIM,NUM_DIM,Eigen::RowMajor> covariance;
};


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

TEST_F(NormalDistributionTest, Constructor)
{
  kf::noise::NormalDistribution<NUM_DIM,true, true, true, double,Eigen::RowMajor> zero_mean_unit_var_n;
  kf::noise::NormalDistribution<1,      true, true, true, double,Eigen::RowMajor> zero_mean_unit_var_1;
  kf::noise::NormalDistribution<NUM_DIM,false,false,false,double,Eigen::RowMajor> disabled_n(this->mean, this->covariance);
  kf::noise::NormalDistribution<1,      false,false,false,double,Eigen::RowMajor> disabled_1(this->mean.col(0), this->covariance.col(0).row(0));
  kf::noise::NormalDistribution<NUM_DIM,true, false,true, double,Eigen::RowMajor> unit_var_n(this->mean, this->covariance);
  kf::noise::NormalDistribution<1,      true, false,true, double,Eigen::RowMajor> unit_var_1(this->mean.col(0), this->covariance.col(0).row(0));
  kf::noise::NormalDistribution<NUM_DIM,true, true, false,double,Eigen::RowMajor> zero_mean_n(this->mean, this->covariance);
  kf::noise::NormalDistribution<1,      true, true, false,double,Eigen::RowMajor> zero_mean_1(this->mean.col(0), this->covariance.col(0).row(0));
  kf::noise::NormalDistribution<NUM_DIM,true, false,false,double,Eigen::RowMajor> full_n(this->mean, this->covariance);
  kf::noise::NormalDistribution<1,      true, false,false,double,Eigen::RowMajor> full_1(this->mean.col(0), this->covariance.col(0).row(0));
}

TEST_F(NormalDistributionTest, getMean)
{
  kf::noise::NormalDistribution<NUM_DIM,true, true, true, double,Eigen::RowMajor> zero_mean_unit_var_n;
  kf::noise::NormalDistribution<1,      true, true, true, double,Eigen::RowMajor> zero_mean_unit_var_1;
  kf::noise::NormalDistribution<NUM_DIM,false,false,false,double,Eigen::RowMajor> disabled_n(this->mean, this->covariance);
  kf::noise::NormalDistribution<1,      false,false,false,double,Eigen::RowMajor> disabled_1(this->mean.col(0), this->covariance.col(0).row(0));
  kf::noise::NormalDistribution<NUM_DIM,true, false,true, double,Eigen::RowMajor> unit_var_n(this->mean, this->covariance);
  kf::noise::NormalDistribution<1,      true, false,true, double,Eigen::RowMajor> unit_var_1(this->mean.col(0), this->covariance.col(0).row(0));
  kf::noise::NormalDistribution<NUM_DIM,true, true, false,double,Eigen::RowMajor> zero_mean_n(this->mean, this->covariance);
  kf::noise::NormalDistribution<1,      true, true, false,double,Eigen::RowMajor> zero_mean_1(this->mean.col(0), this->covariance.col(0).row(0));
  kf::noise::NormalDistribution<NUM_DIM,true, false,false,double,Eigen::RowMajor> full_n(this->mean, this->covariance);
  kf::noise::NormalDistribution<1,      true, false,false,double,Eigen::RowMajor> full_1(this->mean.col(0), this->covariance.col(0).row(0));

  EXPECT_TRUE((zero_mean_unit_var_n.getMean().array().abs()                     < this->output_error_eps).all());
  EXPECT_TRUE((zero_mean_unit_var_1.getMean().array().abs()                     < this->output_error_eps).all());
  EXPECT_TRUE(((this->mean.array()        - disabled_n.getMean().array()).abs() < this->output_error_eps).all());
  EXPECT_TRUE(((this->mean.col(0).array() - disabled_1.getMean().array()).abs() < this->output_error_eps).all());
  EXPECT_TRUE(((this->mean.array()        - unit_var_n.getMean().array()).abs() < this->output_error_eps).all());
  EXPECT_TRUE(((this->mean.col(0).array() - unit_var_1.getMean().array()).abs() < this->output_error_eps).all());
  EXPECT_TRUE((zero_mean_n.getMean().array().abs()                              < this->output_error_eps).all());
  EXPECT_TRUE((zero_mean_1.getMean().array().abs()                              < this->output_error_eps).all());
  EXPECT_TRUE(((this->mean.array()        - full_n.getMean().array()).abs()     < this->output_error_eps).all());
  EXPECT_TRUE(((this->mean.col(0).array() - full_1.getMean().array()).abs()     < this->output_error_eps).all());
}

TEST_F(NormalDistributionTest, getCovariance)
{
  kf::noise::NormalDistribution<NUM_DIM,true, true, true, double,Eigen::RowMajor> zero_mean_unit_var_n;
  kf::noise::NormalDistribution<1,      true, true, true, double,Eigen::RowMajor> zero_mean_unit_var_1;
  kf::noise::NormalDistribution<NUM_DIM,false,false,false,double,Eigen::RowMajor> disabled_n(this->mean, this->covariance);
  kf::noise::NormalDistribution<1,      false,false,false,double,Eigen::RowMajor> disabled_1(this->mean.col(0), this->covariance.col(0).row(0));
  kf::noise::NormalDistribution<NUM_DIM,true, false,true, double,Eigen::RowMajor> unit_var_n(this->mean, this->covariance);
  kf::noise::NormalDistribution<1,      true, false,true, double,Eigen::RowMajor> unit_var_1(this->mean.col(0), this->covariance.col(0).row(0));
  kf::noise::NormalDistribution<NUM_DIM,true, true, false,double,Eigen::RowMajor> zero_mean_n(this->mean, this->covariance);
  kf::noise::NormalDistribution<1,      true, true, false,double,Eigen::RowMajor> zero_mean_1(this->mean.col(0), this->covariance.col(0).row(0));
  kf::noise::NormalDistribution<NUM_DIM,true, false,false,double,Eigen::RowMajor> full_n(this->mean, this->covariance);
  kf::noise::NormalDistribution<1,      true, false,false,double,Eigen::RowMajor> full_1(this->mean.col(0), this->covariance.col(0).row(0));

  EXPECT_TRUE(((Eigen::Matrix<double,NUM_DIM,NUM_DIM,Eigen::RowMajor>::Identity().array() - zero_mean_unit_var_n.getCovariance().array()).abs() < this->output_error_eps).all());
  EXPECT_TRUE(((Eigen::Matrix<double,1,      1,      Eigen::RowMajor>::Identity().array() - zero_mean_unit_var_1.getCovariance().array()).abs() < this->output_error_eps).all());

  EXPECT_TRUE(((this->covariance.array()               - disabled_n.getCovariance().array()).abs() < this->output_error_eps).all());
  EXPECT_TRUE(((this->covariance.col(0).row(0).array() - disabled_1.getCovariance().array()).abs() < this->output_error_eps).all());

  EXPECT_TRUE(((Eigen::Matrix<double,NUM_DIM,NUM_DIM,Eigen::RowMajor>::Identity().array() - unit_var_n.getCovariance().array()).abs() < this->output_error_eps).all());
  EXPECT_TRUE(((Eigen::Matrix<double,1,      1,      Eigen::RowMajor>::Identity().array() - unit_var_1.getCovariance().array()).abs() < this->output_error_eps).all());

  EXPECT_TRUE(((this->covariance.array()               - zero_mean_n.getCovariance().array()).abs() < this->output_error_eps).all());
  EXPECT_TRUE(((this->covariance.col(0).row(0).array() - zero_mean_1.getCovariance().array()).abs() < this->output_error_eps).all());

  EXPECT_TRUE(((this->covariance.array()               - full_n.getCovariance().array()).abs() < this->output_error_eps).all());
  EXPECT_TRUE(((this->covariance.col(0).row(0).array() - full_1.getCovariance().array()).abs() < this->output_error_eps).all());
}

TEST_F(NormalDistributionTest, getNoise)
{
  std::vector<kf::noise::NoiseBasePtr<NUM_DIM,double,Eigen::RowMajor>>      noise_gens_n;
  std::vector<kf::noise::NoiseBasePtr<1,      double,Eigen::RowMajor>>      noise_gens_1;

  kf::noise::NormalDistribution<NUM_DIM,false,false,false,double,Eigen::RowMajor> disabled_n(this->mean, this->covariance);
  kf::noise::NormalDistribution<1,      false,false,false,double,Eigen::RowMajor> disabled_1(this->mean.col(0), this->covariance.col(0).row(0));

  noise_gens_n.reserve(4);
  noise_gens_1.reserve(4);

  noise_gens_n.emplace_back(std::make_shared<kf::noise::NormalDistribution<NUM_DIM,true,true, true, double,Eigen::RowMajor>>());
  noise_gens_n.emplace_back(std::make_shared<kf::noise::NormalDistribution<NUM_DIM,true,false,true, double,Eigen::RowMajor>>(this->mean, this->covariance));
  noise_gens_n.emplace_back(std::make_shared<kf::noise::NormalDistribution<NUM_DIM,true,true, false,double,Eigen::RowMajor>>(this->mean, this->covariance));
  noise_gens_n.emplace_back(std::make_shared<kf::noise::NormalDistribution<NUM_DIM,true,false,false,double,Eigen::RowMajor>>(this->mean, this->covariance));

  noise_gens_1.emplace_back(std::make_shared<kf::noise::NormalDistribution<1,true, true, true, double,Eigen::RowMajor>>());
  noise_gens_1.emplace_back(std::make_shared<kf::noise::NormalDistribution<1,true, false,true, double,Eigen::RowMajor>>(this->mean.col(0), this->covariance.col(0).row(0)));
  noise_gens_1.emplace_back(std::make_shared<kf::noise::NormalDistribution<1,true, true, false,double,Eigen::RowMajor>>(this->mean.col(0), this->covariance.col(0).row(0)));
  noise_gens_1.emplace_back(std::make_shared<kf::noise::NormalDistribution<1,true, false,false,double,Eigen::RowMajor>>(this->mean.col(0), this->covariance.col(0).row(0)));

  for(Eigen::Index x_it = 0; x_it < this->number_of_runs; ++x_it)
  {
    EXPECT_TRUE((disabled_n.getNoise().array().abs() < this->output_error_eps).all());
    EXPECT_TRUE((disabled_1.getNoise().array().abs() < this->output_error_eps).all());
  }
  for(size_t gen_it = 0; gen_it < noise_gens_1.size(); ++gen_it)
  {
    Eigen::Matrix<double,Eigen::Dynamic,NUM_DIM,       Eigen::RowMajor> x_n(this->number_of_runs, NUM_DIM);
    Eigen::Matrix<double,1,             Eigen::Dynamic,Eigen::RowMajor> x_1(this->number_of_runs);
    for(Eigen::Index x_it = 0; x_it < this->number_of_runs; ++x_it)
    {
      x_n.row(x_it) = noise_gens_n[gen_it]->getNoise();
      x_1[x_it]     = noise_gens_1[gen_it]->getNoise()[0];
    }

    const Eigen::Matrix<double,1,NUM_DIM,Eigen::RowMajor> mean_n = x_n.colwise().sum().array()/double(this->number_of_runs);
    const double                                          mean_1 = x_1.rowwise().sum()[0]/double(this->number_of_runs);

    Eigen::Matrix<double,NUM_DIM,NUM_DIM,Eigen::RowMajor> covariance_n = Eigen::Matrix<double,NUM_DIM,NUM_DIM,Eigen::RowMajor>::Zero();
    double                                                covariance_1 = 0;
    for(Eigen::Index x_it = 0; x_it < this->number_of_runs; ++x_it)
    {
      const Eigen::Matrix<double,1,NUM_DIM,Eigen::RowMajor> temp = x_n.row(x_it) - mean_n;
      covariance_n += temp.transpose()*temp;

      const double temp_1 = x_1[x_it] - mean_1;
      covariance_1 += temp_1*temp_1;
    }
    covariance_n.array() /= double(this->number_of_runs - 1);
    covariance_1 /= double(this->number_of_runs - 1);

    EXPECT_TRUE(((noise_gens_n[gen_it]->getMean().array()       - mean_n.array()).      abs() < this->output_error_eps).all());
    EXPECT_TRUE(((noise_gens_1[gen_it]->getMean().array()       - mean_1).              abs() < this->output_error_eps).all());
    EXPECT_TRUE(((noise_gens_n[gen_it]->getCovariance().array() - covariance_n.array()).abs() < this->output_error_eps).all());
    EXPECT_TRUE(((noise_gens_1[gen_it]->getCovariance().array() - covariance_1).        abs() < this->output_error_eps).all());
  }
}

/* normal_distribution_test.cpp */
