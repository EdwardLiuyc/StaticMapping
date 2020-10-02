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

#ifndef BUILDER_SENSOR_FUSIONS_IMU_GPS_EKF_H_
#define BUILDER_SENSOR_FUSIONS_IMU_GPS_EKF_H_

#include "builder/sensor_fusions/interface.h"
#include "common/simple_time.h"

namespace static_map {

namespace sensors {
struct ImuMsg;
struct GpsEnuMsg;
struct OdomMsg;
};  // namespace sensors

namespace sensor_fusions {

class ImuGpsEkf : public Interface {
 public:
  explicit ImuGpsEkf(const double imu_gravity_time_constant,
                     const double imu_frequency, SimpleTime time);
  ~ImuGpsEkf();

  void AddImuData(const data::ImuMsg& imu_msg) final;
  void AddGpsData(const data::GpsEnuMsg& gps_msg) final;
  void AddOdomData(const data::OdomMsg&) final {
    LOG(FATAL) << "Not supported in this tracker.";
  }
};

}  // namespace sensor_fusions
}  // namespace static_map

#endif  // BUILDER_SENSOR_FUSIONS_IMU_GPS_EKF_H_
