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

#ifndef BUILDER_MSG_CONVERSION_H_
#define BUILDER_MSG_CONVERSION_H_

#include <utility>

#include "nav_msgs/Odometry.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pointmatcher/PointMatcher.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"

#include "builder/sensors.h"

namespace static_map {
namespace sensors {

using PclTimeStamp = uint64_t;
SimpleTime ToLocalTime(const ros::Time& time);
SimpleTime ToLocalTime(const PclTimeStamp& time);

Header ToLocalHeader(const std_msgs::Header& header);
ImuMsg ToLocalImu(const sensor_msgs::Imu& msg);

OdomMsg ToLocalOdom(const nav_msgs::Odometry& msg);

NavSatFixMsg ToLocalNavSatMsg(const sensor_msgs::NavSatFix& msg);

using PM = PointMatcher<float>;
template <typename PointT>
PM::DataPoints pclPointCloudToLibPointMatcherPoints(
    const typename pcl::PointCloud<PointT>::Ptr& pcl_point_cloud) {
  if (!pcl_point_cloud || pcl_point_cloud->empty()) {
    return PM::DataPoints();
  }

  PM::DataPoints::Labels labels;
  labels.push_back(PM::DataPoints::Label("x", 1));
  labels.push_back(PM::DataPoints::Label("y", 1));
  labels.push_back(PM::DataPoints::Label("z", 1));
  labels.push_back(PM::DataPoints::Label("pad", 1));

  const size_t point_count(pcl_point_cloud->points.size());
  PM::Matrix inner_cloud(4, point_count);
  int index = 0;
  for (size_t i = 0; i < point_count; ++i) {
    if (!std::isnan(pcl_point_cloud->points[i].x) &&
        !std::isnan(pcl_point_cloud->points[i].y) &&
        !std::isnan(pcl_point_cloud->points[i].z)) {
      inner_cloud(0, index) = pcl_point_cloud->points[i].x;
      inner_cloud(1, index) = pcl_point_cloud->points[i].y;
      inner_cloud(2, index) = pcl_point_cloud->points[i].z;
      inner_cloud(3, index) = 1;
      ++index;
    }
  }

  PM::DataPoints d(inner_cloud.leftCols(index), labels);
  return std::move(d);
}

}  // namespace sensors
}  // namespace static_map

#endif  // BUILDER_MSG_CONVERSION_H_
