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

#ifndef BUILDER_SIMPLE_POSE_EXTRAPOLATOR_H_
#define BUILDER_SIMPLE_POSE_EXTRAPOLATOR_H_

#include <deque>
#include <memory>

#include "Eigen/Eigen"
#include "builder/sensors.h"
#include "common/math.h"
#include "common/mutex.h"
#include "common/simple_time.h"

namespace static_map {

///
/// @brief SimplePoseExtrapolator: Simply use a sliding window to get average
/// velocity and angle velocity, then do extrapolation in the near future.
///
class SimplePoseExtrapolator {
 public:
  using RigidPose3d = Eigen::Matrix4d;

  explicit SimplePoseExtrapolator(SimpleTime pose_queue_duration);

  SimplePoseExtrapolator(const SimplePoseExtrapolator&) = delete;
  SimplePoseExtrapolator& operator=(const SimplePoseExtrapolator&) = delete;

  void AddPose(SimpleTime time, const RigidPose3d& pose);
  RigidPose3d ExtrapolatePose(SimpleTime time);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  void UpdateVelocitiesFromPoses();
  Eigen::Quaterniond ExtrapolateRotation(SimpleTime time) const;
  Eigen::Vector3d ExtrapolateTranslation(SimpleTime time) const;

  common::Mutex mutex_;
  const SimpleTime pose_queue_duration_;

  std::deque<sensors::TimedPose> timed_pose_queue_;
  Eigen::Vector3d linear_velocity_from_poses_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_from_poses_ = Eigen::Vector3d::Zero();
  sensors::TimedPose cached_extrapolated_pose_;
};
}  // namespace static_map

#endif  // BUILDER_SIMPLE_POSE_EXTRAPOLATOR_H_
