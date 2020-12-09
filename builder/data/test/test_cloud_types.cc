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

#include "builder/data/cloud_types.h"
#define BOOST_TEST_MODULE CloudTypes

#include "boost/test/unit_test.hpp"
#include "common/math.h"
#include "test/test_helper.h"

#define BOOST_CHECK_DOUBLE_EQUAL(A, B) \
  BOOST_CHECK_LE(std::fabs((A) - (B)), 1e-6);

namespace static_map {
namespace data {

using test::CreateRandomInnerCloud;
using test::CreateRandomPclCloud;

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

    auto inner_cloud = ToInnerPointCloud(*pcl_cloud);
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

BOOST_AUTO_TEST_CASE(InnerCloudSerialDeserial) {
  const int size = 800;
  const auto inner_cloud = CreateRandomInnerCloud(size);
  {
    std::fstream of("/tmp/test.bin", std::ios::out | std::ios::binary);
    BOOST_CHECK(of.is_open());
    BOOST_CHECK(inner_cloud->Serialize(&of) >= 0);
    of.close();
  }

  InnerCloudType::Ptr loaded_cloud(new InnerCloudType);
  {
    std::fstream bin_fs("/tmp/test.bin", std::ios::in | std::ios::binary);
    BOOST_CHECK(bin_fs.is_open());
    BOOST_CHECK(loaded_cloud->Deserialize(&bin_fs) >= 0);
    bin_fs.close();
  }

  BOOST_CHECK_EQUAL(loaded_cloud->points.size(), size);
  for (int i = 0; i < size; ++i) {
    const auto& inner_point = inner_cloud->points.at(i);
    const auto& loaded_point = loaded_cloud->points.at(i);
    BOOST_CHECK_DOUBLE_EQUAL(loaded_point.x, inner_point.x);
    BOOST_CHECK_DOUBLE_EQUAL(loaded_point.y, inner_point.y);
    BOOST_CHECK_DOUBLE_EQUAL(loaded_point.z, inner_point.z);
    BOOST_CHECK_DOUBLE_EQUAL(loaded_point.intensity, inner_point.intensity);
    BOOST_CHECK_DOUBLE_EQUAL(loaded_point.factor, inner_point.factor);
  }
}

BOOST_AUTO_TEST_CASE(EigenCloud) {}

BOOST_AUTO_TEST_CASE(InnerCloudData) {
  const int size = 1000;
  const auto inner_cloud = CreateRandomInnerCloud(size);
  InnerPointCloudData inner_cloud_data(inner_cloud);
  BOOST_CHECK_EQUAL(inner_cloud_data.GetInnerCloud(), inner_cloud);

  inner_cloud_data.Clear();
  BOOST_CHECK(nullptr != inner_cloud_data.GetInnerCloud());
  BOOST_CHECK(inner_cloud_data.Empty());
  BOOST_CHECK_EQUAL(size, inner_cloud->points.size());
}

BOOST_AUTO_TEST_CASE(TransformPointAndCloud) {
  const InnerPointType point = {
      .x = 10, .y = 0, .z = 30, .intensity = 0., .factor = 1.};
  {
    const auto transformed_point =
        TransformPoint(Eigen::Matrix4d::Identity(), point);
    BOOST_CHECK_DOUBLE_EQUAL(transformed_point.x, point.x);
    BOOST_CHECK_DOUBLE_EQUAL(transformed_point.y, point.y);
    BOOST_CHECK_DOUBLE_EQUAL(transformed_point.z, point.z);
    BOOST_CHECK_DOUBLE_EQUAL(transformed_point.intensity, point.intensity);
    BOOST_CHECK_DOUBLE_EQUAL(transformed_point.factor, point.factor);
  }
  {
    Eigen::Vector6<double> se3;
    se3 << 0., 0., 0., 0., 0., M_PI;
    Eigen::Matrix4d transform = common::Vector6ToTransform(se3);

    const auto transformed_point = TransformPoint(transform, point);
    BOOST_CHECK_DOUBLE_EQUAL(transformed_point.x, -point.x);
    BOOST_CHECK_DOUBLE_EQUAL(transformed_point.y, -point.y);
    BOOST_CHECK_DOUBLE_EQUAL(transformed_point.z, point.z);
    BOOST_CHECK_DOUBLE_EQUAL(transformed_point.intensity, point.intensity);
    BOOST_CHECK_DOUBLE_EQUAL(transformed_point.factor, point.factor);
  }

  const int size = 1000;
  const auto inner_cloud = CreateRandomInnerCloud(size);
  {
    Eigen::Vector6<double> se3;
    se3 << 0., 0., 0., 0., 0., M_PI;
    Eigen::Matrix4d transform = common::Vector6ToTransform(se3);

    auto another_cloud = CreateRandomInnerCloud(200);
    inner_cloud->ApplyTransformToOutput(transform, another_cloud.get());
    BOOST_CHECK_EQUAL(inner_cloud->points.size(), another_cloud->points.size());
    BOOST_CHECK_EQUAL(inner_cloud->stamp, another_cloud->stamp);

    for (int i = 0; i < size; ++i) {
      const auto& transformed_point = another_cloud->points.at(i);
      const auto& inner_point = inner_cloud->points.at(i);
      BOOST_CHECK_DOUBLE_EQUAL(transformed_point.x, -inner_point.x);
      BOOST_CHECK_DOUBLE_EQUAL(transformed_point.y, -inner_point.y);
      BOOST_CHECK_DOUBLE_EQUAL(transformed_point.z, inner_point.z);
      BOOST_CHECK_DOUBLE_EQUAL(transformed_point.intensity,
                               inner_point.intensity);
      BOOST_CHECK_DOUBLE_EQUAL(transformed_point.factor, inner_point.factor);
    }

    inner_cloud->ApplyTransformInplace(transform);
    for (int i = 0; i < size; ++i) {
      const auto& transformed_point = another_cloud->points.at(i);
      const auto& inner_point = inner_cloud->points.at(i);
      BOOST_CHECK_DOUBLE_EQUAL(transformed_point.x, inner_point.x);
      BOOST_CHECK_DOUBLE_EQUAL(transformed_point.y, inner_point.y);
      BOOST_CHECK_DOUBLE_EQUAL(transformed_point.z, inner_point.z);
      BOOST_CHECK_DOUBLE_EQUAL(transformed_point.intensity,
                               inner_point.intensity);
      BOOST_CHECK_DOUBLE_EQUAL(transformed_point.factor, inner_point.factor);
    }
  }
}

}  // namespace data
}  // namespace static_map
