#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Geometry>

#include "imu_fusion_madgwick/imu_fusion_madgwick.hpp"

using namespace std;
using namespace Eigen;
using namespace imu_fusion_madgwick;

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
