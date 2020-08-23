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

#include "builder/simple_pose_extrapolator.h"

namespace static_map {

namespace {
inline Eigen::Quaterniond Rotation(
    const SimplePoseExtrapolator::RigidPose3d& pose) {
  return Eigen::Quaterniond(Eigen::Matrix3d(pose.block(0, 0, 3, 3)));
}
}  // namespace

SimplePoseExtrapolator::SimplePoseExtrapolator(SimpleTime pose_queue_duration)
    : pose_queue_duration_(pose_queue_duration) {}

void SimplePoseExtrapolator::AddPose(SimpleTime time, const RigidPose3d& pose) {
  common::MutexLocker locker(&mutex_);
  timed_pose_queue_.emplace_back(time, pose);

  if (timed_pose_queue_.size() == 1u) {
    // It's the first pose and the only pose.
    cached_extrapolated_pose_.time = time;
    cached_extrapolated_pose_.pose = pose;
    return;
  }

  while (timed_pose_queue_.size() > 2 &&
         timed_pose_queue_[1].time <= time - pose_queue_duration_) {
    timed_pose_queue_.pop_front();
  }
  UpdateVelocitiesFromPoses();
}

SimplePoseExtrapolator::RigidPose3d SimplePoseExtrapolator::ExtrapolatePose(
    SimpleTime time) {
  common::MutexLocker locker(&mutex_);
  const sensors::TimedPose& newest_timed_pose = timed_pose_queue_.back();
  CHECK_GE(time, newest_timed_pose.time);
  if (cached_extrapolated_pose_.time != time) {
    const Eigen::Vector3d translation =
        ExtrapolateTranslation(time) + newest_timed_pose.pose.block(0, 3, 3, 1);
    const Eigen::Quaterniond rotation =
        Eigen::Quaterniond(
            Eigen::Matrix3d(newest_timed_pose.pose.block(0, 0, 3, 3))) *
        ExtrapolateRotation(time);

    RigidPose3d new_pose = RigidPose3d::Identity();
    new_pose.block(0, 0, 3, 3) = rotation.toRotationMatrix();
    new_pose.block(0, 3, 3, 1) = translation;
    cached_extrapolated_pose_.time = time;
    cached_extrapolated_pose_.pose = new_pose;
  }
  return cached_extrapolated_pose_.pose;
}

void SimplePoseExtrapolator::UpdateVelocitiesFromPoses() {
  if (timed_pose_queue_.size() < 2) {
    // We need two poses to estimate velocities.
    return;
  }
  const sensors::TimedPose& newest_timed_pose = timed_pose_queue_.back();
  const auto newest_time = newest_timed_pose.time;
  const sensors::TimedPose& oldest_timed_pose = timed_pose_queue_.front();
  const auto oldest_time = oldest_timed_pose.time;
  const double queue_delta = (newest_time - oldest_time).toSec();
  if (queue_delta < pose_queue_duration_.toSec()) {
    LOG(WARNING) << "Queue too short for velocity estimation. Queue duration: "
                 << queue_delta << " s";
    return;
  }
  const RigidPose3d& newest_pose = newest_timed_pose.pose;
  const RigidPose3d& oldest_pose = oldest_timed_pose.pose;
  linear_velocity_from_poses_ =
      (newest_pose.block(0, 3, 3, 1) - oldest_pose.block(0, 3, 3, 1)) /
      queue_delta;
  angular_velocity_from_poses_ =
      common::QuaternionToEulers(Rotation(oldest_pose).inverse() *
                                 Rotation(newest_pose)) /
      queue_delta;
}

Eigen::Quaterniond SimplePoseExtrapolator::ExtrapolateRotation(
    const SimpleTime time) const {
  const sensors::TimedPose& newest_timed_pose = timed_pose_queue_.back();
  const double extrapolation_delta = (time - newest_timed_pose.time).toSec();

  const Eigen::Vector3d delta_anguler =
      angular_velocity_from_poses_ * extrapolation_delta;
  return common::EulerAnglesToQuaternion(delta_anguler);
}

Eigen::Vector3d SimplePoseExtrapolator::ExtrapolateTranslation(
    SimpleTime time) const {
  const sensors::TimedPose& newest_timed_pose = timed_pose_queue_.back();
  const double extrapolation_delta = (time - newest_timed_pose.time).toSec();
  return extrapolation_delta * linear_velocity_from_poses_;
}

}  // namespace static_map
