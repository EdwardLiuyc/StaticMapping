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

#include "test/test_helper.h"

#include <random>

namespace static_map {
namespace test {

static std::random_device rd;
static std::mt19937 gen(rd());
static std::uniform_real_distribution<> distrib(0., 100.);

pcl::PointCloud<pcl::PointXYZI>::Ptr CreateRandomPclCloud(const int size) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  cloud->header.stamp = distrib(gen);
  for (int i = 0; i < size; ++i) {
    pcl::PointXYZI pcl_point;
    pcl_point.x = distrib(gen);
    pcl_point.y = distrib(gen);
    pcl_point.z = distrib(gen);
    pcl_point.intensity = distrib(gen);
    cloud->push_back(pcl_point);
  }
  return cloud;
}

data::InnerCloudType::Ptr CreateRandomInnerCloud(const int size) {
  data::InnerCloudType::Ptr cloud(new data::InnerCloudType);
  cloud->stamp = SimpleTime::FromNanoSec(1000000000);
  for (int i = 0; i < size; ++i) {
    data::InnerPointType point;
    point.x = distrib(gen);
    point.y = distrib(gen);
    point.z = distrib(gen);
    point.intensity = distrib(gen);
    cloud->points.push_back(point);
  }
  return cloud;
}

}  // namespace test
}  // namespace static_map
