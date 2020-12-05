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

#include <random>

#include "builder/data/cloud_types.h"
#define BOOST_TEST_MODULE CloudTypes

#include <boost/test/unit_test.hpp>

#define BOOST_CHECK_DOUBLE_EQUAL(A, B) \
  BOOST_CHECK_LE(std::fabs((A) - (B)), 1e-6);

namespace static_map {
namespace data {
namespace {

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

InnerCloudType::Ptr CreateRandomInnerCloud(const int size) {
  InnerCloudType::Ptr cloud(new InnerCloudType);
  cloud->stamp = SimpleTime::FromNanoSec(1000000000);
  for (int i = 0; i < size; ++i) {
    InnerPointType point;
    point.x = distrib(gen);
    point.y = distrib(gen);
    point.z = distrib(gen);
    point.intensity = distrib(gen);
    cloud->points.push_back(point);
  }
  return cloud;
}

}  // namespace

BOOST_AUTO_TEST_CASE(PointConversion) {
  {
    pcl::PointXYZ pcl_point(1., 2., 3.);
    const auto inner_point = ToInnerPoint(pcl_point);
    BOOST_CHECK_EQUAL(inner_point.x, 1.);
    BOOST_CHECK_EQUAL(inner_point.y, 2.);
    BOOST_CHECK_EQUAL(inner_point.z, 3.);
    BOOST_CHECK_EQUAL(inner_point.intensity, 0.);
    BOOST_CHECK_EQUAL(inner_point.factor, 0.);

    pcl::PointXYZ pcl_point2;
    ToPclPoint(inner_point, &pcl_point2);
    BOOST_CHECK_EQUAL(pcl_point.x, pcl_point2.x);
    BOOST_CHECK_EQUAL(pcl_point.y, pcl_point2.y);
    BOOST_CHECK_EQUAL(pcl_point.z, pcl_point2.z);
  }
  {
    pcl::PointXYZI pcl_point;
    pcl_point.x = 2.;
    pcl_point.y = 4.;
    pcl_point.z = 1.;
    pcl_point.intensity = 12.;
    const auto inner_point = ToInnerPoint(pcl_point);
    BOOST_CHECK_EQUAL(inner_point.x, 2.);
    BOOST_CHECK_EQUAL(inner_point.y, 4.);
    BOOST_CHECK_EQUAL(inner_point.z, 1.);
    BOOST_CHECK_EQUAL(inner_point.intensity, 12.);
    BOOST_CHECK_EQUAL(inner_point.factor, 0.);

    pcl::PointXYZI pcl_point2;
    ToPclPoint(inner_point, &pcl_point2);
    BOOST_CHECK_EQUAL(pcl_point.x, pcl_point2.x);
    BOOST_CHECK_EQUAL(pcl_point.y, pcl_point2.y);
    BOOST_CHECK_EQUAL(pcl_point.z, pcl_point2.z);
    BOOST_CHECK_EQUAL(pcl_point.intensity, pcl_point2.intensity);
  }
}

BOOST_AUTO_TEST_CASE(CloudConversion) {
  {
    // pcl cloud -> inner cloud
    const int size = 1200;
    const auto pcl_cloud = CreateRandomPclCloud(size);
    BOOST_CHECK_EQUAL(size, pcl_cloud->size());

    auto inner_cloud = ToInnerPoints(*pcl_cloud);
    BOOST_CHECK(inner_cloud);
    BOOST_CHECK_EQUAL(size, inner_cloud->points.size());

    for (int i = 0; i < size; ++i) {
      const auto& pcl_point = pcl_cloud->at(i);
      const auto& inner_point = inner_cloud->points.at(i);
      BOOST_CHECK_EQUAL(pcl_point.x, inner_point.x);
      BOOST_CHECK_EQUAL(pcl_point.y, inner_point.y);
      BOOST_CHECK_EQUAL(pcl_point.z, inner_point.z);
      BOOST_CHECK_EQUAL(pcl_point.intensity, inner_point.intensity);
    }

    BOOST_CHECK_EQUAL(pcl_cloud->header.stamp, ToPclTime(inner_cloud->stamp));
  }
  {
    // inner cloud -> pcl cloud
    const int size = 890;
    const auto inner_cloud = CreateRandomInnerCloud(size);
    BOOST_CHECK_EQUAL(size, inner_cloud->points.size());

    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(
        new pcl::PointCloud<pcl::PointXYZI>);

    ToPclPointCloud(*inner_cloud, pcl_cloud.get());
    BOOST_CHECK_EQUAL(size, pcl_cloud->points.size());

    for (int i = 0; i < size; ++i) {
      const auto& pcl_point = pcl_cloud->at(i);
      const auto& inner_point = inner_cloud->points.at(i);
      BOOST_CHECK_DOUBLE_EQUAL(pcl_point.x, (float)inner_point.x);
      BOOST_CHECK_DOUBLE_EQUAL(pcl_point.y, (float)inner_point.y);
      BOOST_CHECK_DOUBLE_EQUAL(pcl_point.z, (float)inner_point.z);
      BOOST_CHECK_DOUBLE_EQUAL(pcl_point.intensity,
                               (float)inner_point.intensity);
    }

    BOOST_CHECK_EQUAL(inner_cloud->stamp, ToLocalTime(pcl_cloud->header.stamp));
  }
}

BOOST_AUTO_TEST_CASE(EigenCloud) {}

BOOST_AUTO_TEST_CASE(InnerCloudData) {
  const int size = 1000;
  const auto inner_cloud = CreateRandomInnerCloud(size);
  InnerPointCloudData<pcl::PointXYZI> inner_cloud_data(inner_cloud);

  BOOST_CHECK_EQUAL(inner_cloud_data.GetInnerCloud(), inner_cloud);

  auto pcl_cloud = inner_cloud_data.GetPclCloud();
  for (int i = 0; i < size; ++i) {
    const auto& pcl_point = pcl_cloud->at(i);
    const auto& inner_point = inner_cloud->points.at(i);
    BOOST_CHECK_DOUBLE_EQUAL(pcl_point.x, (float)inner_point.x);
    BOOST_CHECK_DOUBLE_EQUAL(pcl_point.y, (float)inner_point.y);
    BOOST_CHECK_DOUBLE_EQUAL(pcl_point.z, (float)inner_point.z);
    BOOST_CHECK_DOUBLE_EQUAL(pcl_point.intensity, (float)inner_point.intensity);
  }

  BOOST_CHECK(!inner_cloud_data.Empty());
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform.block(0, 3, 3, 1) << 23., 22., 10.;
  inner_cloud_data.TransformCloud(transform);
  auto pcl_cloud_transformed = inner_cloud_data.GetPclCloud();
  BOOST_CHECK(pcl_cloud != pcl_cloud_transformed);
  for (int i = 0; i < size; ++i) {
    const auto& pcl_point = pcl_cloud_transformed->at(i);
    const auto& inner_point = inner_cloud_data.GetInnerCloud()->points.at(i);
    BOOST_CHECK_DOUBLE_EQUAL(pcl_point.x, (float)inner_point.x);
    BOOST_CHECK_DOUBLE_EQUAL(pcl_point.y, (float)inner_point.y);
    BOOST_CHECK_DOUBLE_EQUAL(pcl_point.z, (float)inner_point.z);
    BOOST_CHECK_DOUBLE_EQUAL(pcl_point.intensity, (float)inner_point.intensity);
  }

  BOOST_CHECK_EQUAL(inner_cloud->stamp, inner_cloud_data.GetTime());
  BOOST_CHECK_EQUAL(inner_cloud->stamp, ToLocalTime(pcl_cloud->header.stamp));

  inner_cloud_data.Clear();
  BOOST_CHECK(nullptr != inner_cloud_data.GetInnerCloud());
  BOOST_CHECK(inner_cloud_data.Empty());
  BOOST_CHECK_EQUAL(size, inner_cloud->points.size());
}

}  // namespace data
}  // namespace static_map
