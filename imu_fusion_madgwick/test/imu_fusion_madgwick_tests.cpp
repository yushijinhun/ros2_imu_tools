// Copyright 2019 Bold Hearts
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "imu_fusion_madgwick/imu_fusion_madgwick.hpp"

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Geometry>

#include <gtest/gtest.h>
#include <memory>

inline ::testing::AssertionResult MatricesEqual(
  Eigen::MatrixXd const & expected,
  Eigen::MatrixXd const & actual,
  double delta = 0.000001)
{
  double d = (expected - actual).array().abs().sum();
  if (d < delta) {
    return ::testing::AssertionSuccess();
  } else {
    return ::testing::AssertionFailure() << "Actual: " << std::endl << actual << std::endl <<
           "Expected: " << std::endl << expected << std::endl << " d = " << d;
  }
}

TEST(OrientationTests, construction)
{
  imu_fusion_madgwick::IMUFusionMadgwick imu;
  Eigen::Quaterniond quaternion = imu.getEigenQuaternion();

  EXPECT_EQ(1.0, quaternion.w() );
  EXPECT_EQ(Eigen::Vector3d::Zero(), quaternion.vec() );
}

TEST(OrientationTests, gyro_halfPiXHighFreq)
{
  imu_fusion_madgwick::IMUFusionMadgwick imu_fusion;

  for (unsigned i = 0; i < 512; ++i) {
    imu_fusion.integrate(Eigen::Vector3d{0.5 * M_PI, 0, 0}, 1.0 / 512);
  }

  Eigen::Quaterniond quaternion = imu_fusion.getEigenQuaternion();

  EXPECT_TRUE(MatricesEqual(Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitX()).matrix(),
    quaternion.matrix()));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
