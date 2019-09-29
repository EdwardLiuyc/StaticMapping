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

#ifndef BUILDER_DATA_COLLECTOR_H_
#define BUILDER_DATA_COLLECTOR_H_

#include <functional>
#include <memory>
#include <vector>

#include "builder/sensors.h"
#include "common/mutex.h"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include <boost/optional.hpp>

namespace static_map {

struct DataCollectorOptions {
  int accumulate_cloud_num = 1;
};

enum SensorDataType { kImuData, kPointCloudData, kGpsData, kDataTypeCount };

template <typename PointT>
class DataCollector {
 public:
  using PointCloudType = typename pcl::PointCloud<PointT>;
  using PointCloudPtr = typename PointCloudType::Ptr;
  using PointCloudConstPtr = typename PointCloudType::ConstPtr;

  explicit DataCollector(const DataCollectorOptions& options);
  ~DataCollector() {}

  DataCollector(const DataCollector&) = delete;
  DataCollector& operator=(const DataCollector&) = delete;

  struct PointCloudData {
    SimpleTime time;
    float delta_time_in_cloud;
    PointCloudPtr cloud;
  };

  struct ImuData {
    SimpleTime time;
    Eigen::Vector3d acceleration;
    Eigen::Vector3d angular_velocity;
  };

  struct GpsData {
    SimpleTime time;
    bool status_fixed;
    Eigen::Vector3d utm_postion;
    Eigen::Vector3d lat_lon_alt;
  };

  /// @brief collect imu data
  void AddSensorData(const sensors::ImuMsg& imu_msg);
  /// @brief collect gps data
  void AddSensorData(const sensors::NavSatFixMsg& navsat_msg);
  /// @brief collect point cloud data
  void AddSensorData(const PointCloudPtr& cloud);
  /// @brief get utm at 'time' using linear interpolation
  std::unique_ptr<Eigen::Vector3d> InterpolateUtm(const SimpleTime& time,
                                                  double time_threshold = 0.005,
                                                  bool trim_data = false);
  /// @brief delete specific type of data before time
  void TrimSensorData(const SensorDataType type, const SimpleTime& time);
  /// @brief get init utm offset for all utm coords
  Eigen::Vector3d GetUtmOffset() const;

 protected:
  void TrimGpsData(const SimpleTime& time);
  void TrimImuData(const SimpleTime& time);

 private:
  common::Mutex mutex_;
  const DataCollectorOptions options_;

  std::vector<PointCloudData> cloud_data_;
  std::vector<ImuData> imu_data_;

  std::vector<GpsData> gps_data_;
  boost::optional<Eigen::Vector3d> utm_init_offset_;
};

}  // namespace static_map

#endif  // BUILDER_DATA_COLLECTOR_H_
