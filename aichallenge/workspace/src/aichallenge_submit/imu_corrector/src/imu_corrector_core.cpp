// Copyright 2020 Tier IV, Inc.
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

#include "imu_corrector/imu_corrector_core.hpp"

std::array<double, 9> transformCovariance(const std::array<double, 9> & cov)
{
  using COV_IDX = tier4_autoware_utils::xyz_covariance_index::XYZ_COV_IDX;

  double max_cov = 0.0;
  max_cov = std::max(max_cov, cov[COV_IDX::X_X]);
  max_cov = std::max(max_cov, cov[COV_IDX::Y_Y]);
  max_cov = std::max(max_cov, cov[COV_IDX::Z_Z]);

  std::array<double, 9> cov_transformed;
  cov_transformed.fill(0.);
  cov_transformed[COV_IDX::X_X] = max_cov;
  cov_transformed[COV_IDX::Y_Y] = max_cov;
  cov_transformed[COV_IDX::Z_Z] = max_cov;
  return cov_transformed;
}

geometry_msgs::msg::Vector3 transformVector3(
  const geometry_msgs::msg::Vector3 & vec, const geometry_msgs::msg::TransformStamped & transform)
{
  geometry_msgs::msg::Vector3Stamped vec_stamped;
  vec_stamped.vector = vec;

  geometry_msgs::msg::Vector3Stamped vec_stamped_transformed;
  tf2::doTransform(vec_stamped, vec_stamped_transformed, transform);
  return vec_stamped_transformed.vector;
}

geometry_msgs::msg::Quaternion transformQuaternion(
  const geometry_msgs::msg::Quaternion & quat, const geometry_msgs::msg::TransformStamped & transform)
{
  geometry_msgs::msg::QuaternionStamped quat_stamped;
  quat_stamped.quaternion = quat;

  geometry_msgs::msg::QuaternionStamped quat_stamped_transformed;
  tf2::doTransform(quat_stamped, quat_stamped_transformed, transform);
  return quat_stamped_transformed.quaternion;
}

namespace imu_corrector
{
ImuCorrector::ImuCorrector(const rclcpp::NodeOptions & node_options)
: Node("imu_corrector", node_options),
  output_frame_(declare_parameter<std::string>("base_link", "base_link"))
{
  transform_listener_ = std::make_shared<tier4_autoware_utils::TransformListener>(this);

  angular_velocity_offset_x_imu_link_ = declare_parameter<double>("angular_velocity_offset_x", 0.0);
  angular_velocity_offset_y_imu_link_ = declare_parameter<double>("angular_velocity_offset_y", 0.0);
  angular_velocity_offset_z_imu_link_ = declare_parameter<double>("angular_velocity_offset_z", 0.0);

  angular_velocity_stddev_xx_imu_link_ =
    declare_parameter<double>("angular_velocity_stddev_xx", 0.03);
  angular_velocity_stddev_yy_imu_link_ =
    declare_parameter<double>("angular_velocity_stddev_yy", 0.03);
  angular_velocity_stddev_zz_imu_link_ =
    declare_parameter<double>("angular_velocity_stddev_zz", 0.03);

  accel_stddev_imu_link_ = declare_parameter<double>("acceleration_stddev", 10000.0);

  // EMA filter parameters
  use_ema_filter_ = declare_parameter<bool>("use_ema_filter", true);
  ema_alpha_ = declare_parameter<double>("ema_alpha", 0.1); // Tune this: 0.0 to 1.0 (lower is smoother)

  // Kalman filter parameters
  use_kalman_filter_ = declare_parameter<bool>("use_kalman_filter", true);
  kalman_q_ = declare_parameter<double>("kalman_q", 0.01); // Tune this: Process noise covariance
  kalman_r_ = declare_parameter<double>("kalman_r", 0.1);  // Tune this: Measurement noise covariance

  auto rv_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "input", rv_qos, std::bind(&ImuCorrector::callbackImu, this, std::placeholders::_1));

  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("output", rv_qos);
}

void ImuCorrector::callbackImu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg_ptr)
{
  sensor_msgs::msg::Imu imu_msg;
  imu_msg = *imu_msg_ptr;

  imu_msg.angular_velocity.x -= angular_velocity_offset_x_imu_link_;
  imu_msg.angular_velocity.y -= angular_velocity_offset_y_imu_link_;
  imu_msg.angular_velocity.z -= angular_velocity_offset_z_imu_link_;

  // EMA Filter for Gyroscope
  if (use_ema_filter_) {
    if (!ema_initialized_) {
      ema_x_ = imu_msg.angular_velocity.x;
      ema_y_ = imu_msg.angular_velocity.y;
      ema_z_ = imu_msg.angular_velocity.z;
      ema_initialized_ = true;
    } else {
      ema_x_ = ema_alpha_ * imu_msg.angular_velocity.x + (1.0 - ema_alpha_) * ema_x_;
      ema_y_ = ema_alpha_ * imu_msg.angular_velocity.y + (1.0 - ema_alpha_) * ema_y_;
      ema_z_ = ema_alpha_ * imu_msg.angular_velocity.z + (1.0 - ema_alpha_) * ema_z_;
    }
    imu_msg.angular_velocity.x = ema_x_;
    imu_msg.angular_velocity.y = ema_y_;
    imu_msg.angular_velocity.z = ema_z_;
  }

  // Kalman Filter for Gyroscope
  if (use_kalman_filter_) {
    if (!kf_initialized_) {
      kf_x_ = imu_msg.angular_velocity.x;
      kf_y_ = imu_msg.angular_velocity.y;
      kf_z_ = imu_msg.angular_velocity.z;
      kf_p_x_ = 1.0;
      kf_p_y_ = 1.0;
      kf_p_z_ = 1.0;
      kf_initialized_ = true;
    } else {
      // Predict
      kf_p_x_ += kalman_q_;
      kf_p_y_ += kalman_q_;
      kf_p_z_ += kalman_q_;

      // Update
      double k_x = kf_p_x_ / (kf_p_x_ + kalman_r_);
      double k_y = kf_p_y_ / (kf_p_y_ + kalman_r_);
      double k_z = kf_p_z_ / (kf_p_z_ + kalman_r_);

      kf_x_ = kf_x_ + k_x * (imu_msg.angular_velocity.x - kf_x_);
      kf_y_ = kf_y_ + k_y * (imu_msg.angular_velocity.y - kf_y_);
      kf_z_ = kf_z_ + k_z * (imu_msg.angular_velocity.z - kf_z_);

      kf_p_x_ = (1.0 - k_x) * kf_p_x_;
      kf_p_y_ = (1.0 - k_y) * kf_p_y_;
      kf_p_z_ = (1.0 - k_z) * kf_p_z_;
    }
    imu_msg.angular_velocity.x = kf_x_;
    imu_msg.angular_velocity.y = kf_y_;
    imu_msg.angular_velocity.z = kf_z_;
  }

  imu_msg.angular_velocity_covariance[COV_IDX::X_X] =
    angular_velocity_stddev_xx_imu_link_ * angular_velocity_stddev_xx_imu_link_;
  imu_msg.angular_velocity_covariance[COV_IDX::Y_Y] =
    angular_velocity_stddev_yy_imu_link_ * angular_velocity_stddev_yy_imu_link_;
  imu_msg.angular_velocity_covariance[COV_IDX::Z_Z] =
    angular_velocity_stddev_zz_imu_link_ * angular_velocity_stddev_zz_imu_link_;
  imu_msg.linear_acceleration_covariance[COV_IDX::X_X] =
    accel_stddev_imu_link_ * accel_stddev_imu_link_;
  imu_msg.linear_acceleration_covariance[COV_IDX::Y_Y] =
    accel_stddev_imu_link_ * accel_stddev_imu_link_;
  imu_msg.linear_acceleration_covariance[COV_IDX::Z_Z] =
    accel_stddev_imu_link_ * accel_stddev_imu_link_;

  geometry_msgs::msg::TransformStamped::ConstSharedPtr tf_imu2base_ptr =
    transform_listener_->getLatestTransform(imu_msg.header.frame_id, output_frame_);
  if (!tf_imu2base_ptr) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "Please publish TF %s to %s", output_frame_.c_str(),
    (imu_msg.header.frame_id).c_str());
    return;
  }

  sensor_msgs::msg::Imu imu_msg_base_link;
  imu_msg_base_link.header.stamp = imu_msg_ptr->header.stamp;
  imu_msg_base_link.header.frame_id = output_frame_;
  imu_msg_base_link.orientation = transformQuaternion(imu_msg.orientation, *tf_imu2base_ptr); 
  imu_msg_base_link.linear_acceleration =
    transformVector3(imu_msg.linear_acceleration, *tf_imu2base_ptr);
  imu_msg_base_link.linear_acceleration_covariance =
    transformCovariance(imu_msg.linear_acceleration_covariance);
  imu_msg_base_link.angular_velocity = transformVector3(imu_msg.angular_velocity, *tf_imu2base_ptr);
  imu_msg_base_link.angular_velocity_covariance =
    transformCovariance(imu_msg.angular_velocity_covariance);

  imu_pub_->publish(imu_msg_base_link);
}

}  // namespace imu_corrector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(imu_corrector::ImuCorrector)
