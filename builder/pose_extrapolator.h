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

#ifndef BUILDER_POSE_EXTRAPOLATOR_H_
#define BUILDER_POSE_EXTRAPOLATOR_H_

#include <deque>
#include <memory>

#include "Eigen/Eigen"
#include "builder/data_types.h"
#include "builder/imu_tracker.h"
#include "common/math.h"
#include "common/mutex.h"
#include "common/simple_time.h"

namespace static_map {

class PoseExtrapolator {
 public:
  using RigidPose3d = Eigen::Matrix4d;

  enum class Mode : uint8_t {
    // Using IMU (refer to google cartographer)
    kDefaultWithImu,
    kSimpleCTRV
  };

  PoseExtrapolator(SimpleTime pose_queue_duration,
                   double imu_gravity_time_constant,
                   Mode mode = Mode::kDefaultWithImu);

  static std::unique_ptr<PoseExtrapolator> InitializeWithImu(
      SimpleTime pose_queue_duration, double imu_gravity_time_constant,
      const sensors::ImuMsg& imu_data);

  static std::unique_ptr<PoseExtrapolator> InitialSimpleCTRV(
      const SimpleTime& pose_queue_duration);

  // Returns the time of the last added pose or Time::min() if no pose was added
  // yet.
  SimpleTime GetLastPoseTime();
  SimpleTime GetLastExtrapolatedTime();

  void AddPose(SimpleTime time, const RigidPose3d& pose);
  void AddImuData(const sensors::ImuMsg& imu_data);
  void AddOdometryData(const sensors::OdomMsg& odometry_data);
  RigidPose3d ExtrapolatePose(SimpleTime time);

  // Returns the current gravity alignment estimate as a rotation from
  // the tracking frame into a gravity aligned frame.
  Eigen::Quaterniond EstimateGravityOrientation(SimpleTime time);

  // formerly, we always assume that the init velocity should be zero
  // but, it is not suitable for any situation, so sometimes we should
  // give it a init linear velocity, and it's not necessary for the velocity
  // to be very accuracy, just a rough estimation will be fine
  void InitRoughLinearVelocity(const Eigen::Vector3d& velocity);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  PoseExtrapolator(const PoseExtrapolator&) = delete;
  PoseExtrapolator& operator=(const PoseExtrapolator&) = delete;

  void UpdateVelocitiesFromPoses();
  void TrimImuData();
  void TrimOdometryData();
  void AdvanceImuTracker(SimpleTime time, ImuTracker* imu_tracker) const;
  Eigen::Quaterniond ExtrapolateRotation(SimpleTime time,
                                         ImuTracker* imu_tracker) const;
  Eigen::Vector3d ExtrapolateTranslation(SimpleTime time);

 private:
  common::Mutex mutex_;
  const Mode mode_;
  const SimpleTime pose_queue_duration_;

  std::deque<sensors::TimedPose> timed_pose_queue_;
  Eigen::Vector3d linear_velocity_from_poses_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_from_poses_ = Eigen::Vector3d::Zero();

  const double gravity_time_constant_;
  std::deque<sensors::ImuMsg> imu_data_;
  std::unique_ptr<ImuTracker> imu_tracker_;
  std::unique_ptr<ImuTracker> odometry_imu_tracker_;
  std::unique_ptr<ImuTracker> extrapolation_imu_tracker_;
  sensors::TimedPose cached_extrapolated_pose_;

  std::deque<sensors::OdomMsg> odometry_data_;
  Eigen::Vector3d linear_velocity_from_odometry_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_from_odometry_ = Eigen::Vector3d::Zero();
};

}  // namespace static_map

#endif  // BUILDER_POSE_EXTRAPOLATOR_H_
