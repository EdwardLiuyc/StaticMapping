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
#include <deque>
#include <functional>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "builder/sensors.h"
#include "common/mutex.h"
#include "pre_processors/filter_factory.h"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include <boost/optional.hpp>
#include "GeographicLib/GeoCoords.hpp"
#include "GeographicLib/Geocentric.hpp"
#include "GeographicLib/Geoid.hpp"
#include "GeographicLib/LocalCartesian.hpp"
#include "GeographicLib/MagneticModel.hpp"

namespace static_map {

struct DataCollectorOptions {
  int accumulate_cloud_num = 1;
};

// 声明有哪些数据
enum SensorDataType {
  kImuData,
  kPointCloudData,
  kGpsData,
  kOdometryData,
  kDataTypeCount->这里也是为了计数
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

  /*******************************************************
   * !@brief 这里能看出数据的数据结构设计也没自己想的那么复杂:
   *    都是时间戳+具体数据内容->点云是空间位置, IMU是加速度和角速度
   *    GPS是ENU坐标系下空间位置+等等
   * ******************************************************/

  // 点云数据结构
  struct PointCloudData {
    SimpleTime time;
    float delta_time_in_cloud;
    PointCloudPtr cloud;
  };

  // IMU数据
  struct ImuData {
    SimpleTime time;
    Eigen::Vector3d acceleration;
    Eigen::Vector3d angular_velocity;
  };

  // GPS数据
  struct GpsData {
    SimpleTime time;
    bool status_fixed;
    Eigen::Vector3d enu_position;
    Eigen::Vector3d lat_lon_alt;
  };

  // 里程计数据
  struct OdometryData {
    SimpleTime time;
    Eigen::Matrix4d pose;
    // Eigen::Vector3d linear_velocity;
    // Eigen::Vector3d angular_velocity;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  /*********************************************
   * !@brief 这里是收集数据（用了比较经典的重载算法）
   * *******************************************/
  /// @brief collect imu data
  // !@brief 收集IMU数据
  void AddSensorData(const sensors::ImuMsg& imu_msg);
  /// @brief collect gps data
  // !@brief 收集GPS数据
  void AddSensorData(const sensors::NavSatFixMsg& navsat_msg);
  /// @brief collect point cloud data
  // !@brief 收集点云数据
  void AddSensorData(const PointCloudPtr& cloud);
  /// @brief collect odom data
  // !@brief 收集里程计数据
  void AddSensorData(const sensors::OdomMsg& odom_msg);
  /// @brief get gps(enu) at 'time' using linear interpolation
  // !@brief 收集GPS数据
  std::unique_ptr<Eigen::Vector3d> InterpolateGps(const SimpleTime& time,
                                                  double time_threshold = 0.005,
                                                  bool trim_data = false);
  std::unique_ptr<Eigen::Matrix4d> InterpolateOdom(
      const SimpleTime& time, double time_threshold = 0.005,
      bool trim_data = false);
  /// @brief delete specific type of data before time
  void TrimSensorData(const SensorDataType type, const SimpleTime& time);
  /// @brief get init gps reference (lat,lon,alt)
  /// it is a boost::optional object so it can be empty
  boost::optional<GeographicLib::LocalCartesian> GetGpsReference() const;
  /// @brief 获得新点云
  PointCloudPtr GetNewCloud(float* const delta_time);
  /// @brief output gps(enu) path to .pcd file for review
  // !@brief 将ENU坐标系下的点云数据写入文件
  void RawGpsDataToFile(const std::string& filename) const;
  /// @brief output odom path to .pcd file for review
  // !@brief 将里程计数据写入文件
  void RawOdomDataToFile(const std::string& filename) const;

  size_t GetRemainingCloudSize();
  void ClearAllCloud();

 protected:
  void TrimGpsData(const SimpleTime& time);
  void TrimImuData(const SimpleTime& time);

  // !@brief 点云预处理算法
  void CloudPreProcessing();

 private:
  /// every kind of data has its own mutex
  /// avoiding resource competition between different kind of data
  // !@brief 之类比较出彩,声明一个mutex数组,每个里面都保存一个对应数据的mutex对象.
  //         根据数据处理的对象,依据数据处理来做
  std::mutex mutex_[kDataTypeCount];
  // 数据收集选项
  const DataCollectorOptions options_;

  // 容器保存点云, 注意这里一个是原始数据,一个是处理后
  /************************************************************
   * !@brief 这里是点云数据先添加到->cloud_data_before_preprocessing_->再到cloud_data_->用于后面的处理
   * *********************************************************/
  std::deque<PointCloudData> cloud_data_;
  std::deque<PointCloudData> cloud_data_before_preprocessing_;
  bool kill_cloud_preprocessing_thread_ = false;
  // !@brief 点云预处理线程
  std::thread cloud_processing_thread_;
  // !@brief 点云预处理单元->这里工厂模式（相当出彩）
  pre_processers::filter::Factory<PointT>* filter_factory_;
  // !@brief 用容器保存IMU和GPS数据
  std::vector<ImuData> imu_data_;
  std::vector<GpsData> gps_data_;
  std::vector<OdometryData> odom_data_;

  // !@brief 这里boost::optional怎么样
  boost::optional<Eigen::Matrix4d> odom_init_offset_;
  boost::optional<GeographicLib::LocalCartesian> reference_gps_point_;

  pcl::PointCloud<pcl::PointXYZI> enu_path_cloud_;
  pcl::PointCloud<pcl::PointXYZI> odom_path_cloud_;

  // 积累点云数据->这里感觉数据结构对不上
  PointCloudPtr accumulated_point_cloud_ = nullptr;
  std::atomic_uint accumulated_cloud_count_;
  uint32_t got_clouds_count_ = 0;
  // 累积数据时候的第一次点云时间
  SimpleTime first_time_in_accmulated_cloud_;
  // 整张点云数据后的时间
  SimpleTime last_whole_frame_time_;
};

}  // namespace static_map

#endif  // BUILDER_DATA_COLLECTOR_H_
