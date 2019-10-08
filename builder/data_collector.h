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

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "builder/sensors.h"
#include "common/mutex.h"
#include "pre_processors/filter_factory.h"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include <boost/optional.hpp>

namespace static_map {

struct DataCollectorOptions {
  int accumulate_cloud_num = 1;
};

enum SensorDataType {
  kImuData,
  kPointCloudData,
  kGpsData,
  kOdometryData,
  kDataTypeCount
};

template <typename PointT>
class DataCollector {
 public:
  using PointCloudType = typename pcl::PointCloud<PointT>;
  using PointCloudPtr = typename PointCloudType::Ptr;
  using PointCloudConstPtr = typename PointCloudType::ConstPtr;
  using Locker = std::lock_guard<std::mutex>;

  DataCollector(const DataCollectorOptions& options,
                pre_processers::filter::Factory<PointT>* const filter);
  ~DataCollector();

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

  struct OdometryData {
    SimpleTime time;
    Eigen::Matrix4d pose;
    // Eigen::Vector3d linear_velocity;
    // Eigen::Vector3d angular_velocity;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  /// @brief collect imu data
  void AddSensorData(const sensors::ImuMsg& imu_msg);
  /// @brief collect gps data
  void AddSensorData(const sensors::NavSatFixMsg& navsat_msg);
  /// @brief collect point cloud data
  void AddSensorData(const PointCloudPtr& cloud);
  /// @brief collect odom data
  void AddSensorData(const sensors::OdomMsg& odom_msg);
  /// @brief get utm at 'time' using linear interpolation
  std::unique_ptr<Eigen::Vector3d> InterpolateUtm(const SimpleTime& time,
                                                  double time_threshold = 0.005,
                                                  bool trim_data = false);
  std::unique_ptr<Eigen::Matrix4d> InterpolateOdom(
      const SimpleTime& time, double time_threshold = 0.005,
      bool trim_data = false);
  /// @brief delete specific type of data before time
  void TrimSensorData(const SensorDataType type, const SimpleTime& time);
  /// @brief get init utm offset for all utm coords
  Eigen::Vector3d GetUtmOffset() const;
  /// @brief get
  PointCloudPtr GetNewCloud(float* const delta_time);
  /// @brief output utm path to .pcd file for review
  void RawGpsDataToFile(const std::string& filename) const;
  /// @brief output odom path to .pcd file for review
  void RawOdomDataToFile(const std::string& filename) const;

  size_t GetRemainingCloudSize();
  void ClearAllCloud();

 protected:
  void TrimGpsData(const SimpleTime& time);
  void TrimImuData(const SimpleTime& time);

  void CloudPreProcessing();

 private:
  /// every kind of data has its own mutex
  /// avoiding resource competition between different kind of data
  std::mutex mutex_[kDataTypeCount];
  const DataCollectorOptions options_;

  std::vector<PointCloudData> cloud_data_;
  std::atomic_bool accumulate_cloud_available_;
  bool kill_cloud_preprocessing_thread_ = false;
  std::thread cloud_processing_thread_;
  pre_processers::filter::Factory<PointT>* filter_factory_;
  std::vector<ImuData> imu_data_;
  std::vector<GpsData> gps_data_;
  std::vector<OdometryData> odom_data_;

  boost::optional<Eigen::Vector3d> utm_init_offset_;
  boost::optional<Eigen::Matrix4d> odom_init_offset_;

  pcl::PointCloud<pcl::PointXYZI> utm_path_cloud_;
  pcl::PointCloud<pcl::PointXYZI> odom_path_cloud_;

  PointCloudPtr accumulated_point_cloud_ = nullptr;
  PointCloudPtr copied_accumulated_point_cloud_;
  std::atomic_uint accumulated_cloud_count_;
  uint32_t got_clouds_count_ = 0;
  SimpleTime first_time_in_accmulated_cloud_;
  SimpleTime copied_first_time_;
};

}  // namespace static_map

#endif  // BUILDER_DATA_COLLECTOR_H_
