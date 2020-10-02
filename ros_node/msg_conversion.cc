// MIT License

// Copyright (c) 2019 Edward Liu

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "ros_node/msg_conversion.h"

namespace static_map_ros {
namespace {

static_map::data::Header ToLocalHeader(const std_msgs::Header& header) {
  static_map::data::Header local_header;

  local_header.seq = header.seq;
  local_header.frame_id = header.frame_id;
  local_header.stamp = ToLocalTime(header.stamp);

  return local_header;
}

}  // namespace

static_map::SimpleTime ToLocalTime(const ros::Time& time) {
  static_map::SimpleTime local_time;
  local_time.secs = time.sec;
  local_time.nsecs = time.nsec;

  return local_time;
}

static_map::data::ImuMsg ToLocalImu(const sensor_msgs::Imu& msg) {
  static_map::data::ImuMsg local_imu;

  local_imu.header = ToLocalHeader(msg.header);

  local_imu.angular_velocity[0] = msg.angular_velocity.x;
  local_imu.angular_velocity[1] = msg.angular_velocity.y;
  local_imu.angular_velocity[2] = msg.angular_velocity.z;

  local_imu.linear_acceleration[0] = msg.linear_acceleration.x;
  local_imu.linear_acceleration[1] = msg.linear_acceleration.y;
  local_imu.linear_acceleration[2] = msg.linear_acceleration.z;

  local_imu.orientation.x() = msg.orientation.x;
  local_imu.orientation.y() = msg.orientation.y;
  local_imu.orientation.z() = msg.orientation.z;
  local_imu.orientation.w() = msg.orientation.w;

  for (int i = 0; i < 9; ++i) {
    local_imu.angular_velocity_covariance[i] =
        msg.angular_velocity_covariance[i];
    local_imu.linear_acceleration_covariance[i] =
        msg.linear_acceleration_covariance[i];
    local_imu.orientation_covariance[i] = msg.orientation_covariance[i];
  }

  return std::move(local_imu);
}

static_map::data::OdomMsg ToLocalOdom(const nav_msgs::Odometry& msg) {
  static_map::data::OdomMsg local_odom;
  local_odom.header = ToLocalHeader(msg.header);

  local_odom.pose.pose.position.x() = msg.pose.pose.position.x;
  local_odom.pose.pose.position.y() = msg.pose.pose.position.y;
  local_odom.pose.pose.position.z() = msg.pose.pose.position.z;
  local_odom.pose.pose.orientation.w() = msg.pose.pose.orientation.w;
  local_odom.pose.pose.orientation.x() = msg.pose.pose.orientation.x;
  local_odom.pose.pose.orientation.y() = msg.pose.pose.orientation.y;
  local_odom.pose.pose.orientation.z() = msg.pose.pose.orientation.z;

  local_odom.twist.twist.linear << msg.twist.twist.linear.x,
      msg.twist.twist.linear.y, msg.twist.twist.linear.z;
  local_odom.twist.twist.angular << msg.twist.twist.angular.x,
      msg.twist.twist.angular.y, msg.twist.twist.angular.z;
  for (int i = 0; i < 36; ++i) {
    local_odom.pose.covariance[i] = msg.pose.covariance[i];
    local_odom.twist.covariance[i] = msg.twist.covariance[i];
  }

  return std::move(local_odom);
}

static_map::data::NavSatFixMsg ToLocalNavSatMsg(
    const sensor_msgs::NavSatFix& msg) {
  static_map::data::NavSatFixMsg local_navsat;

  local_navsat.header = ToLocalHeader(msg.header);
  local_navsat.status.status = msg.status.status;
  local_navsat.status.service = msg.status.service;

  local_navsat.latitude = msg.latitude;
  local_navsat.longtitude = msg.longitude;
  local_navsat.altitude = msg.altitude;
  for (int i = 0; i < 9; ++i) {
    local_navsat.position_covariance[i] = msg.position_covariance[i];
  }
  local_navsat.position_covariance_type = msg.position_covariance_type;
  return std::move(local_navsat);
}

}  // namespace static_map_ros
