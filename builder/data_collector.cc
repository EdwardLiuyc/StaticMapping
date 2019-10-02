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

#include <utility>

#include "builder/data_collector.h"
#include "builder/msg_conversion.h"
#include "builder/utm.h"
#include "common/macro_defines.h"
#include "common/make_unique.h"
#include "common/math.h"
#include "glog/logging.h"

#include "pcl/io/pcd_io.h"

namespace static_map {

// usually, our longtitude is about 121E, in UTM "51R" zone
constexpr int kUtmZone = 51;

template <typename DataTypeWithTime>
std::pair<int, int> TimeStampBinarySearch(
    const std::vector<DataTypeWithTime>& data_vector, const SimpleTime& time) {
  int mid;
  int start = 0;
  int end = data_vector.size() - 1;
  while (end - start > 1) {
    mid = start + (end - start) / 2;
    if (time < data_vector.at(mid).time) {
      end = mid;
    } else {
      start = mid;
    }
  }
  CHECK_EQ(end - start, 1);
  return std::make_pair(start, end);
}

template <typename PointT>
DataCollector<PointT>::DataCollector(
    const DataCollectorOptions& options,
    pre_processers::filter::Factory<PointT>* const filter)
    : options_(options), filter_factory_(filter) {
  // reserve the vectors for less memory copy when push_back
  constexpr size_t reserve_size = 2000;
  imu_data_.reserve(reserve_size);
  gps_data_.reserve(reserve_size);
  cloud_data_.reserve(reserve_size);
  utm_path_cloud_.points.reserve(reserve_size);
  odom_path_cloud_.points.reserve(reserve_size);
}

template <typename PointT>
DataCollector<PointT>::~DataCollector() {}

template <typename PointT>
void DataCollector<PointT>::RawGpsDataToFile(
    const std::string& filename) const {
  if (!utm_path_cloud_.empty()) {
    PRINT_INFO_FMT("output raw gps data into file: %s", filename.c_str());
    pcl::io::savePCDFileBinaryCompressed(filename, utm_path_cloud_);
  }
}

template <typename PointT>
void DataCollector<PointT>::RawOdomDataToFile(
    const std::string& filename) const {
  if (!odom_path_cloud_.empty()) {
    PRINT_INFO_FMT("output raw odom data into file: %s", filename.c_str());
    pcl::io::savePCDFileBinaryCompressed(filename, odom_path_cloud_);
  }
}

template <typename PointT>
void DataCollector<PointT>::AddSensorData(const sensors::ImuMsg& imu_msg) {
  ImuData imu_data;
  imu_data.time = imu_msg.header.stamp;
  imu_data.acceleration = imu_msg.linear_acceleration.ToEigenVector();
  imu_data.angular_velocity = imu_msg.angular_velocity.ToEigenVector();

  Locker locker(mutex_[kImuData]);
  imu_data_.push_back(imu_data);
}

template <typename PointT>
void DataCollector<PointT>::AddSensorData(
    const sensors::NavSatFixMsg& navsat_msg) {
  Locker locker(mutex_[kGpsData]);
  GpsData data;
  // save all data and its status
  data.status_fixed = (navsat_msg.status.status == sensors::STATUS_FIX);
  data.time = navsat_msg.header.stamp;

  // first, save raw data including latitude, longtitude, altitude
  data.lat_lon_alt << navsat_msg.latitude, navsat_msg.longtitude,
      navsat_msg.altitude;
  // transform to utm
  utm::LatLonToUTMXY(navsat_msg.latitude, navsat_msg.longtitude, kUtmZone,
                     data.utm_postion[0], data.utm_postion[1]);
  data.utm_postion[2] = navsat_msg.altitude;

  if (!utm_init_offset_) {
    utm_init_offset_ = data.utm_postion;
  }
  data.utm_postion -= utm_init_offset_.value();
  if (!gps_data_.empty()) {
    CHECK(data.time > gps_data_.back().time);
  }
  gps_data_.push_back(data);

  pcl::PointXYZI path_point;
  path_point.x = data.utm_postion[0];
  path_point.y = data.utm_postion[1];
  path_point.z = data.utm_postion[2];
  path_point.intensity = sensors::STATUS_FIX;
  utm_path_cloud_.push_back(path_point);
}

template <typename PointT>
void DataCollector<PointT>::AddSensorData(const PointCloudPtr& cloud) {
  PointCloudData data;
  // accumulating clouds into one
  Locker locker(mutex_[kPointCloudData]);
  if (options_.accumulate_cloud_num > 1) {
    // "+=" will update the time stamp of accumulated_point_cloud_
    // so, no need to manually copy the time stamp from pointcloud to
    // accumulated_point_cloud_
    if (accumulated_cloud_count_ == 0) {
      accumulated_point_cloud_.reset(new PointCloudType);
      first_time_in_accmulated_cloud_ =
          sensors::ToLocalTime(cloud->header.stamp);
    }
    *accumulated_point_cloud_ += *cloud;
    accumulated_cloud_count_++;
    if (accumulated_cloud_count_ < options_.accumulate_cloud_num) {
      return;
    }
  } else {
    accumulated_point_cloud_.reset();
    accumulated_point_cloud_ = cloud;
  }

  // filtering cloud
  PointCloudPtr filtered_cloud(new PointCloudType);
  filter_factory_->SetInputCloud(accumulated_point_cloud_);
  filter_factory_->Filter(filtered_cloud);

  // insert new data
  data.cloud = filtered_cloud;
  data.time = sensors::ToLocalTime(filtered_cloud->header.stamp);
  if (first_time_in_accmulated_cloud_ != SimpleTime()) {
    data.delta_time_in_cloud = (sensors::ToLocalTime(cloud->header.stamp) -
                                first_time_in_accmulated_cloud_)
                                   .toSec();
  } else {
    data.delta_time_in_cloud = 0.f;
  }
  cloud_data_.push_back(data);
  first_time_in_accmulated_cloud_ = sensors::ToLocalTime(cloud->header.stamp);

  cloud->points.clear();
  cloud->points.shrink_to_fit();
  accumulated_cloud_count_ = 0;

  // just for debug
  got_clouds_count_++;
  if (got_clouds_count_ % 100u == 0) {
    PRINT_INFO_FMT("Got %u clouds already.", got_clouds_count_);
  }
}

template <typename PointT>
typename DataCollector<PointT>::PointCloudPtr
DataCollector<PointT>::GetNewCloud(float* const delta_time) {
  Locker locker(mutex_[kPointCloudData]);
  if (cloud_data_.empty()) {
    return nullptr;
  }

  PointCloudPtr cloud = cloud_data_[0].cloud;
  *delta_time = cloud_data_[0].delta_time_in_cloud;
  cloud_data_.erase(cloud_data_.begin());
  return cloud;
}

template <typename PointT>
size_t DataCollector<PointT>::GetRemainingCloudSize() {
  Locker locker(mutex_[kPointCloudData]);
  return cloud_data_.size();
}

template <typename PointT>
void DataCollector<PointT>::AddSensorData(const sensors::OdomMsg& odom_msg) {
  Locker locker(mutex_[kOdometryData]);
  OdometryData data;
  data.time = odom_msg.header.stamp;
  data.pose = odom_msg.PoseInMatrix();
  if (!odom_init_offset_) {
    odom_init_offset_ = data.pose;
  }
  const Eigen::Matrix4d relative_pose =
      odom_init_offset_.value().inverse() * data.pose;
  data.pose = relative_pose;
  odom_data_.push_back(data);

  const Eigen::Vector3d translation = common::Translation(data.pose);
  pcl::PointXYZI odom_point;
  odom_point.x = translation[0];
  odom_point.y = translation[1];
  odom_point.z = translation[2];
  odom_point.intensity = 0.;
  odom_path_cloud_.push_back(odom_point);
}

template <typename PointT>
std::unique_ptr<Eigen::Vector3d> DataCollector<PointT>::InterpolateUtm(
    const SimpleTime& time, double time_threshold, bool trim_data) {
  CHECK_LE(time_threshold, 0.5);
  GpsData former_data;
  GpsData latter_data;
  {
    Locker locker(mutex_[kGpsData]);
    if (gps_data_.empty()) {
      // size == 0
      PRINT_WARNING("no utm data.");
      return nullptr;
    } else if (gps_data_.size() == 1) {
      // size == 1
      if (std::fabs(time.toSec() - gps_data_[0].time.toSec()) <=
              time_threshold &&
          gps_data_[0].status_fixed) {
        return common::make_unique<Eigen::Vector3d>(gps_data_[0].utm_postion);
      } else {
        PRINT_WARNING("the only data is not good.");
        return nullptr;
      }
    } else if (time < gps_data_.front().time || time > gps_data_.back().time) {
      // size >= 2
      PRINT_WARNING("too old or too new.");
      return nullptr;
    }

    // binary search for the time period for the target time
    const auto indices = TimeStampBinarySearch(gps_data_, time);
    former_data = gps_data_.at(indices.first);
    latter_data = gps_data_.at(indices.second);
    if (trim_data) {
      gps_data_.erase(gps_data_.begin(), gps_data_.begin() + indices.first);
    }
  }

  const double delta_time = (latter_data.time - former_data.time).toSec();
  CHECK_GT(delta_time, 1.e-6);
  if (delta_time > 1.) {
    PRINT_WARNING("some thing wrong with the search");
    return nullptr;
  }

  CHECK(time >= former_data.time && time <= latter_data.time);
  if (!former_data.status_fixed || !latter_data.status_fixed) {
    PRINT_WARNING("no fixed gps data at this time.");
    return nullptr;
  }
  const double factor = (time - former_data.time).toSec() / delta_time;
  CHECK(factor >= 0. && factor <= 1.);
  const Eigen::Vector3d delta =
      factor * (latter_data.utm_postion - former_data.utm_postion);
  return common::make_unique<Eigen::Vector3d>(former_data.utm_postion + delta);
}

template <typename PointT>
std::unique_ptr<Eigen::Matrix4d> DataCollector<PointT>::InterpolateOdom(
    const SimpleTime& time, double time_threshold, bool trim_data) {
  CHECK_LE(time_threshold, 0.5);
  OdometryData former_data;
  OdometryData latter_data;
  {
    Locker locker(mutex_[kOdometryData]);
    if (odom_data_.empty()) {
      // size == 0
      PRINT_WARNING("no odom data.");
      return nullptr;
    } else if (odom_data_.size() == 1) {
      // size == 1
      if (std::fabs(time.toSec() - odom_data_[0].time.toSec()) <=
          time_threshold) {
        return common::make_unique<Eigen::Matrix4d>(odom_data_[0].pose);
      } else {
        PRINT_WARNING("the only data is not good.");
        return nullptr;
      }
    } else if (time < odom_data_.front().time ||
               time > odom_data_.back().time) {
      // size >= 2
      PRINT_WARNING("too old or too new.");
      return nullptr;
    }

    // binary search for the time period for the target time
    auto indices = TimeStampBinarySearch(odom_data_, time);
    former_data = odom_data_[indices.first];
    latter_data = odom_data_[indices.second];
    if (trim_data) {
      odom_data_.erase(odom_data_.begin(), odom_data_.begin() + indices.first);
    }
  }

  const double delta_time = (latter_data.time - former_data.time).toSec();
  CHECK_GT(delta_time, 1.e-6);
  if (delta_time > 1.) {
    PRINT_WARNING("some thing wrong with the search");
    return nullptr;
  }
  CHECK(time >= former_data.time && time <= latter_data.time);
  const double factor = (time - former_data.time).toSec() / delta_time;
  CHECK(factor >= 0. && factor <= 1.);
  // interpolate the data for more accurate odom data
  const Eigen::Matrix4d pose =
      common::InterpolateTransform(former_data.pose, latter_data.pose, factor);
  return common::make_unique<Eigen::Matrix4d>(pose);
}

template <typename PointT>
Eigen::Vector3d DataCollector<PointT>::GetUtmOffset() const {
  return utm_init_offset_.get_value_or(Eigen::Vector3d());
}

template <typename PointT>
void DataCollector<PointT>::TrimSensorData(const SensorDataType type,
                                           const SimpleTime& time) {
  switch (type) {
    case SensorDataType::kImuData:
      TrimImuData(time);
      break;
    case SensorDataType::kGpsData:
      TrimGpsData(time);
      break;
    case SensorDataType::kPointCloudData:
      break;
    default:
      PRINT_ERROR("unknown type of sensor data.");
      break;
  }
}

template <typename PointT>
void DataCollector<PointT>::ClearAllCloud() {
  Locker locker(mutex_[kPointCloudData]);
  // clear all source clouds
  for (auto& frame : cloud_data_) {
    frame.cloud->points.clear();
    frame.cloud->points.shrink_to_fit();
  }
  cloud_data_.clear();
  cloud_data_.shrink_to_fit();
}

#define TRIM_DATA(data_vector)              \
  const int data_size = data_vector.size(); \
  if (data_size <= 2) {                     \
    return;                                 \
  }                                         \
  int i = 0;                                \
  for (; i < data_size - 2; ++i) {          \
    if (data_vector[i].time >= time) {      \
      break;                                \
    }                                       \
  }                                         \
  data_vector.erase(data_vector.begin(), data_vector.begin() + i);

template <typename PointT>
void DataCollector<PointT>::TrimGpsData(const SimpleTime& time) {
  Locker locker(mutex_[kGpsData]);
  TRIM_DATA(gps_data_);
}

template <typename PointT>
void DataCollector<PointT>::TrimImuData(const SimpleTime& time) {
  Locker locker(mutex_[kImuData]);
  TRIM_DATA(imu_data_);
}

#undef TRIM_DATA

template class DataCollector<pcl::PointXYZI>;

}  // namespace static_map
