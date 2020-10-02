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

#ifndef BUILDER_SENSOR_FUSIONS_IMU_GPS_TRACKER_H_
#define BUILDER_SENSOR_FUSIONS_IMU_GPS_TRACKER_H_

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>
#include <memory>

#include "builder/sensor_fusions/interface.h"
#include "common/mutex.h"
#include "common/simple_time.h"

namespace static_map {

namespace sensors {
struct ImuMsg;
struct GpsEnuMsg;
struct OdomMsg;
};  // namespace sensors

namespace sensor_fusions {

// use gtsam LM to do the state estimation with imu factors and gps factors
// can not do estimation in real-time
// @todo(edward) maybe enhance this method later
class ImuGpsTracker : public Interface {
 public:
  explicit ImuGpsTracker(const double imu_gravity_time_constant,
                         const double imu_frequency, SimpleTime time);
  ~ImuGpsTracker();

  void AddImuData(const data::ImuMsg& imu_msg) final;
  void AddGpsData(const data::GpsEnuMsg& gps_msg) final;
  void AddOdomData(const data::OdomMsg&) final {
    LOG(FATAL) << "Not supported in this tracker.";
  }

 private:
  common::Mutex mutex_;

  const double imu_gravity_time_constant_;
  const double imu_period_;
  SimpleTime time_;

  SimpleTime last_imu_time_;
  SimpleTime last_gps_time_;

  std::unique_ptr<gtsam::PreintegratedImuMeasurements> imu_preintegrated_;
  std::unique_ptr<gtsam::NonlinearFactorGraph> factor_graph_;
  gtsam::Values initial_values_;
  gtsam::NavState prev_state_;
  gtsam::imuBias::ConstantBias prev_bias_;
  gtsam::noiseModel::Diagonal::shared_ptr bias_noise_model_;
  gtsam::noiseModel::Diagonal::shared_ptr velocity_noise_model_;
  gtsam::noiseModel::Diagonal::shared_ptr pose_noise_model_;

  int gps_count_ = 0;
};

}  // namespace sensor_fusions
}  // namespace static_map

#endif  // BUILDER_SENSOR_FUSIONS_IMU_GPS_TRACKER_H_
