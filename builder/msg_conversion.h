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

}  // namespace sensors
}  // namespace static_map

#endif  // BUILDER_MSG_CONVERSION_H_
