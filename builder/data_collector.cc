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
#include "glog/logging.h"

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
  return std::make_pair(start, end);
}

template <typename PointT>
DataCollector<PointT>::DataCollector(const DataCollectorOptions& options)
    : options_(options) {
  // reserve the vectors for less memory copy when push_back
  constexpr size_t reserve_size = 2000;
  imu_data_.reserve(reserve_size);
  gps_data_.reserve(reserve_size);
  cloud_data_.reserve(reserve_size);
}

template <typename PointT>
void DataCollector<PointT>::AddSensorData(const sensors::ImuMsg& imu_msg) {
  ImuData imu_data;
  imu_data.time = imu_msg.header.stamp;
  imu_data.acceleration = imu_msg.linear_acceleration.ToEigenVector();
  imu_data.angular_velocity = imu_msg.angular_velocity.ToEigenVector();

  common::MutexLocker locker(&mutex_);
  imu_data_.push_back(imu_data);
}

template <typename PointT>
void DataCollector<PointT>::AddSensorData(
    const sensors::NavSatFixMsg& navsat_msg) {
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

  common::MutexLocker locker(&mutex_);
  if (!gps_data_.empty()) {
    CHECK(data.time > gps_data_.back().time);
  }
  gps_data_.push_back(data);
}

template <typename PointT>
void DataCollector<PointT>::AddSensorData(const PointCloudPtr& cloud) {
  PointCloudData data;
  data.time = sensors::ToLocalTime(cloud->header.stamp);
  data.cloud = cloud;

  common::MutexLocker locker(&mutex_);
  cloud_data_.push_back(data);
}

template <typename PointT>
std::unique_ptr<Eigen::Vector3d> DataCollector<PointT>::InterpolateUtm(
    const SimpleTime& time, double time_threshold, bool trim_data) {
  if (time_threshold < 1.e-6) {
    time_threshold = 1.e-6;
  }

  GpsData former_data;
  GpsData latter_data;
  {
    common::MutexLocker locker(&mutex_);
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

  CHECK(time >= former_data.time && time <= latter_data.time);
  if (!former_data.status_fixed || !latter_data.status_fixed) {
    PRINT_WARNING("no fixed gps data at this time.");
    return nullptr;
  }
  const double factor = (time - former_data.time).toSec() /
                        (latter_data.time - former_data.time).toSec();
  CHECK(factor >= 0. && factor <= 1.);
  const Eigen::Vector3d delta =
      factor * (latter_data.utm_postion - former_data.utm_postion);
  return common::make_unique<Eigen::Vector3d>(former_data.utm_postion + delta);
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

#define TRIM_DATA(data_vector)              \
  common::MutexLocker locker(&mutex_);      \
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
  TRIM_DATA(gps_data_);
}

template <typename PointT>
void DataCollector<PointT>::TrimImuData(const SimpleTime& time) {
  TRIM_DATA(imu_data_);
}

#undef TRIM_DATA

template class DataCollector<pcl::PointXYZI>;

}  // namespace static_map
