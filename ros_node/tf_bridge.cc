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

#include "ros_node/tf_bridge.h"
#include "common/macro_defines.h"
#include "glog/logging.h"

namespace static_map_ros {
namespace {
inline Eigen::Matrix4d GeometryTransformToEigen(
    const geometry_msgs::TransformStamped& transform) {
  Eigen::Matrix4d eigen_transform = Eigen::Matrix4d::Identity();
  eigen_transform.block(0, 3, 3, 1) << transform.transform.translation.x,
      transform.transform.translation.y, transform.transform.translation.z;
  eigen_transform.block(0, 0, 3, 3) =
      Eigen::Quaterniond(
          transform.transform.rotation.w, transform.transform.rotation.x,
          transform.transform.rotation.y, transform.transform.rotation.z)
          .toRotationMatrix();
  return eigen_transform;
}
}  // namespace

Eigen::Matrix4d LoopUpTransfrom(const std::string& target_frame,
                                const std::string& source_frame,
                                const tf::TransformListener& listener) {
  std::string tf_error_msg;
  int wait_count = 30;
  while (!listener.waitForTransform(target_frame, source_frame, ros::Time(0),
                                    ros::Duration(2.), ros::Duration(0.001),
                                    &tf_error_msg)) {
    PRINT_INFO_FMT("Wating for tf from %s to %s", target_frame.c_str(),
                   source_frame.c_str());
    wait_count--;
    CHECK_GT(wait_count, 0);
  }

  tf::StampedTransform transform;
  listener.lookupTransform(target_frame, source_frame, ros::Time(0), transform);
  Eigen::Isometry3d eigen_transform = Eigen::Isometry3d::Identity();
  tf::transformTFToEigen(transform, eigen_transform);

  return eigen_transform.matrix();
}

Eigen::Matrix4d LoopUpTransfrom(const std::string& target_frame,
                                const std::string& source_frame,
                                const tf2_ros::Buffer& tf_buffer) {
  std::string tf_error_msg;
  CHECK(tf_buffer.canTransform(target_frame, source_frame, ros::Time(0),
                               &tf_error_msg));
  const auto transform =
      tf_buffer.lookupTransform(target_frame, source_frame, ros::Time(0));
  return GeometryTransformToEigen(transform);
}

}  // namespace static_map_ros
