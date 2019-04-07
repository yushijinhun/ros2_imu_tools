#include "imu_filter_madgwick/imu_filter_madgwick.hpp"

namespace imu_filter_madgwick {

// sampling period in seconds
#define deltat 0.008

// gyroscope measurement error in rad/s (shown as 5 deg/s)
#define gyroMeasError 3.14159265358979f * (5.0f / 180.0f)

// compute beta
#define beta sqrt(3.0f / 4.0f) * gyroMeasError

IMUFilterMadgwickPublisher::IMUFilterMadgwickPublisher()
  : rclcpp::Node{"imu_filter_madgwick_publisher"},
  lastUpdateTime_(now())
{

  RCLCPP_INFO(get_logger(), "Initialise robot orientation");
  d_orientation.reset();

  imuPub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data");

  RCLCPP_INFO(get_logger(), "Subscribe for /imu/data_raw");
  imuRawSub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data_raw",
    [ = ](sensor_msgs::msg::Imu::SharedPtr imuRawMsg) {

      // TODO: auto time = imuRawMsg.get()->header.stamp;
      float dt = static_cast<float>((now() - lastUpdateTime_).seconds());
      lastUpdateTime_ = now();

      Eigen::Matrix<double, 3, 1> const& referenceDirMes = Eigen::Matrix<double, 3, 1>{
        imuRawMsg.get()->linear_acceleration.x,
        imuRawMsg.get()->linear_acceleration.y,
        imuRawMsg.get()->linear_acceleration.z
      };

      Eigen::Matrix<double, 3, 1> const& angularRate = Eigen::Matrix<double, 3, 1>{
        imuRawMsg.get()->angular_velocity.x,
        imuRawMsg.get()->angular_velocity.y,
        imuRawMsg.get()->angular_velocity.z
      };

      d_orientation.integrate(angularRate, dt, referenceDirMes, gyroMeasError);

      imuRawMsg->orientation.w = d_orientation.getQuaternion().w();
      imuRawMsg->orientation.x = d_orientation.getQuaternion().x();
      imuRawMsg->orientation.y = d_orientation.getQuaternion().y();
      imuRawMsg->orientation.z = d_orientation.getQuaternion().z();

      RCLCPP_DEBUG(get_logger(), "Received raw IMU message and publish IMU message");
      imuPub_->publish(imuRawMsg.get());
    });

}

IMUFilterMadgwickPublisher::~IMUFilterMadgwickPublisher() {}

} // namespace
