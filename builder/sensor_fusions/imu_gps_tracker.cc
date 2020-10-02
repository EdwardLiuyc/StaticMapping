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

#include "builder/sensor_fusions/imu_gps_tracker.h"

#include <memory>

#include "builder/data_types.h"

namespace static_map {
namespace sensor_fusions {

using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::P;  // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)

ImuGpsTracker::ImuGpsTracker(const double imu_gravity_time_constant,
                             const double imu_frequency, SimpleTime time)
    : Interface(),
      imu_gravity_time_constant_(imu_gravity_time_constant),
      imu_period_(1. / imu_frequency),
      time_(time),
      last_imu_time_(),
      factor_graph_(new gtsam::NonlinearFactorGraph),
      bias_noise_model_(gtsam::noiseModel::Isotropic::Sigma(6, 1e-3)),
      velocity_noise_model_(gtsam::noiseModel::Isotropic::Sigma(3, 0.1)),
      pose_noise_model_(gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished())) {
  const gtsam::Pose3 prior_pose;
  const gtsam::Vector3 prior_velocity;
  // assume zero initial bias
  const gtsam::imuBias::ConstantBias prior_imu_bias;
  initial_values_.insert(P(gps_count_), prior_pose);
  initial_values_.insert(V(gps_count_), prior_velocity);
  initial_values_.insert(B(gps_count_), prior_imu_bias);

  factor_graph_->add(gtsam::PriorFactor<gtsam::Pose3>(P(gps_count_), prior_pose,
                                                      pose_noise_model_));
  factor_graph_->add(gtsam::PriorFactor<gtsam::Vector3>(
      V(gps_count_), prior_velocity, velocity_noise_model_));
  factor_graph_->add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
      B(gps_count_), prior_imu_bias, bias_noise_model_));

  auto param = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(
      imu_gravity_time_constant);
  imu_preintegrated_ = std::make_unique<gtsam::PreintegratedImuMeasurements>(
      param, prior_imu_bias);
}

void ImuGpsTracker::AddImuData(const data::ImuMsg& imu_msg) {
  // common::MutexLocker locker(&mutex_);
  double delta_time = (imu_msg.header.stamp - last_imu_time_).toSec();
  if (last_imu_time_ != SimpleTime()) {
    CHECK_GT(imu_msg.header.stamp, last_imu_time_);
    if (std::fabs(imu_period_ - delta_time) >= imu_period_ * 0.1) {
      PRINT_WARNING_FMT(
          "Maybe missing imu message. delta_time: %lf, imu_period: %lf",
          delta_time, imu_period_);
    }
  } else {
    delta_time = imu_period_;
  }
  imu_preintegrated_->integrateMeasurement(
      imu_msg.linear_acceleration, imu_msg.angular_velocity, delta_time);
  last_imu_time_ = imu_msg.header.stamp;
}

void ImuGpsTracker::AddGpsData(const data::GpsEnuMsg& enu_msg) {
  gps_count_++;

  // step1. add imu factors
  gtsam::PreintegratedImuMeasurements* preint_imu =
      dynamic_cast<gtsam::PreintegratedImuMeasurements*>(
          imu_preintegrated_.get());
  gtsam::ImuFactor imu_factor(P(gps_count_ - 1), V(gps_count_ - 1),
                              P(gps_count_), V(gps_count_), B(gps_count_ - 1),
                              *preint_imu);
  factor_graph_->add(imu_factor);
  factor_graph_->add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
      B(gps_count_ - 1), B(gps_count_),
      gtsam::imuBias::ConstantBias() /* zero bias */, bias_noise_model_));

  // step2. add gps factor
  gtsam::GPSFactor gps_factor(P(gps_count_),
                              gtsam::Point3(enu_msg.x, enu_msg.y, enu_msg.z),
                              gtsam::noiseModel::Isotropic::Sigma(3, 2.0));
  factor_graph_->add(gps_factor);
  const auto prop_state = imu_preintegrated_->predict(prev_state_, prev_bias_);
  initial_values_.insert(P(gps_count_), prop_state.pose());
  initial_values_.insert(V(gps_count_), prop_state.v());
  initial_values_.insert(B(gps_count_), prev_bias_);

  // step3. run optimization
  gtsam::LevenbergMarquardtOptimizer optimizer(*factor_graph_, initial_values_);
  const auto result = optimizer.optimize();

  // step4. overwrite the beginning of the preintegration for the next step.
  prev_state_ = gtsam::NavState(result.at<gtsam::Pose3>(P(gps_count_)),
                                result.at<gtsam::Vector3>(V(gps_count_)));
  prev_bias_ = result.at<gtsam::imuBias::ConstantBias>(B(gps_count_));

  // step5. reset the preintegration object.
  imu_preintegrated_->resetIntegrationAndSetBias(prev_bias_);
  prev_state_.position().print();
}

ImuGpsTracker::~ImuGpsTracker() {}

}  // namespace sensor_fusions
}  // namespace static_map
