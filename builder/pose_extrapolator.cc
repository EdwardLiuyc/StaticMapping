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

#include "builder/pose_extrapolator.h"

#include <algorithm>
#include <memory>

namespace static_map {

namespace {

constexpr double kDefaultWithImuGravity = 9.8;

inline Eigen::Quaterniond Rotation(const PoseExtrapolator::RigidPose3d& pose) {
  return Eigen::Quaterniond(Eigen::Matrix3d(pose.block(0, 0, 3, 3)));
}
}  // namespace

PoseExtrapolator::PoseExtrapolator(const SimpleTime pose_queue_duration,
                                   double imu_gravity_time_constant,
                                   PoseExtrapolator::Mode mode)
    : mode_(mode),
      pose_queue_duration_(pose_queue_duration),
      gravity_time_constant_(imu_gravity_time_constant),
      cached_extrapolated_pose_{SimpleTime(), RigidPose3d::Identity()} {}

std::unique_ptr<PoseExtrapolator> PoseExtrapolator::InitializeWithImu(
    const SimpleTime pose_queue_duration,
    const double imu_gravity_time_constant, const data::ImuMsg& imu_data) {
  auto extrapolator = std::make_unique<PoseExtrapolator>(
      pose_queue_duration, imu_gravity_time_constant);
  extrapolator->AddImuData(imu_data);
  extrapolator->imu_tracker_ = std::make_unique<ImuTracker>(
      imu_gravity_time_constant, imu_data.header.stamp);
  extrapolator->imu_tracker_->AddImuLinearAccelerationObservation(
      imu_data.linear_acceleration);
  extrapolator->imu_tracker_->AddImuAngularVelocityObservation(
      imu_data.angular_velocity);
  extrapolator->imu_tracker_->Advance(imu_data.header.stamp);
  RigidPose3d init_pose = RigidPose3d::Identity();

  init_pose.block(0, 0, 3, 3) =
      extrapolator->imu_tracker_->orientation().toRotationMatrix();
  extrapolator->AddPose(imu_data.header.stamp, init_pose);
  return extrapolator;
}

std::unique_ptr<PoseExtrapolator> PoseExtrapolator::InitialSimpleCTRV(
    const SimpleTime& pose_queue_duration) {
  return std::make_unique<PoseExtrapolator>(
      pose_queue_duration, kDefaultWithImuGravity, Mode::kSimpleCTRV);
}

SimpleTime PoseExtrapolator::GetLastPoseTime() {
  common::MutexLocker locker(&mutex_);
  if (timed_pose_queue_.empty()) {
    return SimpleTime();
  }
  return timed_pose_queue_.back().time;
}

SimpleTime PoseExtrapolator::GetLastExtrapolatedTime() {
  common::MutexLocker locker(&mutex_);
  if (!extrapolation_imu_tracker_) {
    return SimpleTime();
  }
  return extrapolation_imu_tracker_->time();
}

void PoseExtrapolator::AddPose(const SimpleTime time, const RigidPose3d& pose) {
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
  if (mode_ == Mode::kSimpleCTRV) {
    return;
  }

  if (imu_tracker_ == nullptr) {
    SimpleTime tracker_start = time;
    if (!imu_data_.empty()) {
      tracker_start = std::min(tracker_start, imu_data_.front().header.stamp);
    }
    imu_tracker_ =
        std::make_unique<ImuTracker>(gravity_time_constant_, tracker_start);
  }
  AdvanceImuTracker(time, imu_tracker_.get());
  TrimImuData();
  TrimOdometryData();
  odometry_imu_tracker_ = std::make_unique<ImuTracker>(*imu_tracker_);
  extrapolation_imu_tracker_ = std::make_unique<ImuTracker>(*imu_tracker_);
}

void PoseExtrapolator::AddImuData(const data::ImuMsg& imu_data) {
  if (mode_ == Mode::kSimpleCTRV) {
    return;
  }
  common::MutexLocker locker(&mutex_);
  CHECK(timed_pose_queue_.empty() ||
        imu_data.header.stamp >= timed_pose_queue_.back().time);
  imu_data_.push_back(imu_data);
  TrimImuData();
}

void PoseExtrapolator::AddOdometryData(const data::OdomMsg& odometry_data) {
  if (mode_ == Mode::kSimpleCTRV) {
    return;
  }
  common::MutexLocker locker(&mutex_);
  CHECK(timed_pose_queue_.empty() ||
        odometry_data.header.stamp >= timed_pose_queue_.back().time);
  odometry_data_.push_back(odometry_data);
  TrimOdometryData();
  if (odometry_data_.size() < 2) {
    return;
  }
  // TODO(whess): Improve by using more than just the last two odometry poses.
  // Compute extrapolation in the tracking frame.
  const data::OdomMsg& odometry_data_oldest = odometry_data_.front();
  const data::OdomMsg& odometry_data_newest = odometry_data_.back();
  const double odometry_time_delta =
      (odometry_data_newest.header.stamp - odometry_data_oldest.header.stamp)
          .toSec();
  const RigidPose3d odometry_pose_delta =
      odometry_data_oldest.PoseInMatrix().inverse() *
      odometry_data_newest.PoseInMatrix();
  angular_velocity_from_odometry_ =
      common::QuaternionToEulers(Rotation(odometry_pose_delta)) /
      odometry_time_delta;
  if (timed_pose_queue_.empty()) {
    return;
  }
  const Eigen::Vector3d
      linear_velocity_in_tracking_frame_at_newest_odometry_time =
          common::Translation(odometry_pose_delta) / odometry_time_delta;
  const Eigen::Quaterniond orientation_at_newest_odometry_time =
      Eigen::Quaterniond(
          Eigen::Matrix3d(timed_pose_queue_.back().pose.block(0, 3, 3, 1))) *
      ExtrapolateRotation(odometry_data_newest.header.stamp,
                          odometry_imu_tracker_.get());
  linear_velocity_from_odometry_ =
      orientation_at_newest_odometry_time *
      linear_velocity_in_tracking_frame_at_newest_odometry_time;
  std::cout << linear_velocity_from_odometry_.transpose() << std::endl;
}

PoseExtrapolator::RigidPose3d PoseExtrapolator::ExtrapolatePose(
    const SimpleTime time) {
  common::MutexLocker locker(&mutex_);
  const data::TimedPose& newest_timed_pose = timed_pose_queue_.back();
  CHECK_GE(time, newest_timed_pose.time);
  if (cached_extrapolated_pose_.time != time) {
    const Eigen::Vector3d translation =
        ExtrapolateTranslation(time) + newest_timed_pose.pose.block(0, 3, 3, 1);
    const Eigen::Quaterniond rotation =
        Eigen::Quaterniond(
            Eigen::Matrix3d(newest_timed_pose.pose.block(0, 0, 3, 3))) *
        ExtrapolateRotation(time, extrapolation_imu_tracker_.get());

    RigidPose3d new_pose = RigidPose3d::Identity();
    new_pose.block(0, 0, 3, 3) = rotation.toRotationMatrix();
    new_pose.block(0, 3, 3, 1) = translation;
    cached_extrapolated_pose_.time = time;
    cached_extrapolated_pose_.pose = new_pose;
  }
  return cached_extrapolated_pose_.pose;
}

Eigen::Quaterniond PoseExtrapolator::EstimateGravityOrientation(
    const SimpleTime time) {
  if (mode_ == Mode::kSimpleCTRV) {
    return Eigen::Quaterniond::Identity();
  }
  common::MutexLocker locker(&mutex_);
  ImuTracker imu_tracker = *imu_tracker_;
  AdvanceImuTracker(time, &imu_tracker);
  return imu_tracker.orientation();
}

void PoseExtrapolator::InitRoughLinearVelocity(
    const Eigen::Vector3d& velocity) {
  common::MutexLocker locker(&mutex_);
  linear_velocity_from_poses_ = velocity;
}

void PoseExtrapolator::UpdateVelocitiesFromPoses() {
  if (timed_pose_queue_.size() < 2) {
    // We need two poses to estimate velocities.
    return;
  }
  const data::TimedPose& newest_timed_pose = timed_pose_queue_.back();
  const auto newest_time = newest_timed_pose.time;
  const data::TimedPose& oldest_timed_pose = timed_pose_queue_.front();
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

void PoseExtrapolator::TrimImuData() {
  while (imu_data_.size() > 1 && !timed_pose_queue_.empty() &&
         imu_data_[1].header.stamp <= timed_pose_queue_.back().time) {
    imu_data_.pop_front();
  }
}

void PoseExtrapolator::TrimOdometryData() {
  while (odometry_data_.size() > 2 && !timed_pose_queue_.empty() &&
         odometry_data_[1].header.stamp <= timed_pose_queue_.back().time) {
    odometry_data_.pop_front();
  }
}

void PoseExtrapolator::AdvanceImuTracker(const SimpleTime time,
                                         ImuTracker* const imu_tracker) const {
  CHECK_GE(time, imu_tracker->time());
  if (imu_data_.empty() || time < imu_data_.front().header.stamp) {
    // There is no IMU data until 'time', so we advance the ImuTracker and use
    // the angular velocities from poses and fake gravity to help 2D stability.
    imu_tracker->Advance(time);
    imu_tracker->AddImuLinearAccelerationObservation(Eigen::Vector3d::UnitZ());
    imu_tracker->AddImuAngularVelocityObservation(
        odometry_data_.size() < 2 ? angular_velocity_from_poses_
                                  : angular_velocity_from_odometry_);
    return;
  }
  if (imu_tracker->time() < imu_data_.front().header.stamp) {
    // Advance to the beginning of 'imu_data_'.
    imu_tracker->Advance(imu_data_.front().header.stamp);
  }
  auto it = std::lower_bound(
      imu_data_.begin(), imu_data_.end(), imu_tracker->time(),
      [](const data::ImuMsg& imu_data, const SimpleTime& time) {
        return imu_data.header.stamp < time;
      });
  while (it != imu_data_.end() && it->header.stamp < time) {
    imu_tracker->Advance(it->header.stamp);
    imu_tracker->AddImuLinearAccelerationObservation(it->linear_acceleration);
    imu_tracker->AddImuAngularVelocityObservation(it->angular_velocity);
    ++it;
  }
  imu_tracker->Advance(time);
}

Eigen::Quaterniond PoseExtrapolator::ExtrapolateRotation(
    const SimpleTime time, ImuTracker* const imu_tracker) const {
  switch (mode_) {
    case Mode::kDefaultWithImu: {
      CHECK_GE(time, imu_tracker->time());
      AdvanceImuTracker(time, imu_tracker);
      const Eigen::Quaterniond last_orientation = imu_tracker_->orientation();
      return last_orientation.inverse() * imu_tracker->orientation();
    }
    case Mode::kSimpleCTRV: {
      const data::TimedPose& newest_timed_pose = timed_pose_queue_.back();
      const double extrapolation_delta =
          (time - newest_timed_pose.time).toSec();

      const Eigen::Vector3d delta_anguler =
          angular_velocity_from_poses_ * extrapolation_delta;
      return common::EulerAnglesToQuaternion(delta_anguler);
    }
    default:
      break;
  }
  CHECK(false);
  return Eigen::Quaterniond::Identity();
}

Eigen::Vector3d PoseExtrapolator::ExtrapolateTranslation(SimpleTime time) {
  const data::TimedPose& newest_timed_pose = timed_pose_queue_.back();
  const double extrapolation_delta = (time - newest_timed_pose.time).toSec();
  if (mode_ == Mode::kSimpleCTRV || odometry_data_.size() < 2) {
    return extrapolation_delta * linear_velocity_from_poses_;
  }
  return extrapolation_delta * linear_velocity_from_odometry_;
}
}  // namespace static_map
