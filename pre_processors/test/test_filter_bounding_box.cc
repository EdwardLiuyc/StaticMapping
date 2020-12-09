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

#include "pre_processors/filter_bounding_box.h"

#define BOOST_TEST_MODULE FilterBoundingBoxRemoval

#include "boost/test/unit_test.hpp"
#include "common/bounding_box.h"
#include "common/math.h"
#include "test/test_helper.h"

namespace static_map {
namespace pre_processers {
namespace filter {

using test::CreateRandomInnerCloud;

BOOST_AUTO_TEST_CASE(FilterConfig) {
  BoundingBoxRemoval filter_bbox;
  {
    const std::string filter_config =
        "<filter name=\"BoundingBoxRemoval\" >"
        "<param type=\"1\" name=\"min_x\"> 90. </param>"
        "<param type=\"1\" name=\"max_x\"> 80. </param>"
        "</filter>";
    BOOST_CHECK(!filter_bbox.InitFromXmlText(filter_config.c_str()));
  }
  {
    const std::string filter_config =
        "<filter name=\"BoundingBoxRemoval\" >"
        "<param type=\"1\" name=\"min_x\"> 10. </param>"
        "<param type=\"1\" name=\"max_x\"> 80. </param>"
        "<param type=\"1\" name=\"min_y\"> 10. </param>"
        "<param type=\"1\" name=\"max_y\"> 0. </param>"
        "</filter>";
    BOOST_CHECK(!filter_bbox.InitFromXmlText(filter_config.c_str()));
  }
  {
    const std::string filter_config =
        "<filter name=\"BoundingBoxRemoval\" >"
        "<param type=\"1\" name=\"min_z\"> 90. </param>"
        "<param type=\"1\" name=\"max_z\"> 80. </param>"
        "</filter>";
    BOOST_CHECK(!filter_bbox.InitFromXmlText(filter_config.c_str()));
  }
}

BOOST_AUTO_TEST_CASE(Filter) {
  BoundingBoxRemoval filter_bbox;
  const auto raw_cloud = CreateRandomInnerCloud(1000);

  filter_bbox.SetInputCloud(raw_cloud);
  data::InnerCloudType::Ptr filtered_cloud(new data::InnerCloudType);
  {
    filter_bbox.Filter(filtered_cloud);
    BOOST_CHECK_EQUAL(filtered_cloud->stamp, raw_cloud->stamp);
    BOOST_CHECK_EQUAL(filtered_cloud->points.size(), 0);
  }

  {
    const std::string filter_config =
        "<filter name=\"BoundingBoxRemoval\" >"
        "<param type=\"1\" name=\"min_x\"> 0. </param>"
        "<param type=\"1\" name=\"max_x\"> 100. </param>"
        "<param type=\"1\" name=\"min_y\"> 0 </param>"
        "<param type=\"1\" name=\"max_y\"> 100. </param>"
        "<param type=\"1\" name=\"min_z\"> 0. </param>"
        "<param type=\"1\" name=\"max_z\"> 100. </param>"
        "</filter>";
    BOOST_CHECK(filter_bbox.InitFromXmlText(filter_config.c_str()));
    filter_bbox.Filter(filtered_cloud);
    BOOST_CHECK_EQUAL(filtered_cloud->stamp, raw_cloud->stamp);
    BOOST_CHECK_EQUAL(filtered_cloud->points.size(), 0);
  }

  {
    const std::string filter_config =
        "<filter name=\"BoundingBoxRemoval\" >"
        "<param type=\"1\" name=\"min_x\"> 10. </param>"
        "<param type=\"1\" name=\"max_x\"> 80. </param>"
        "<param type=\"1\" name=\"min_y\"> 20. </param>"
        "<param type=\"1\" name=\"max_y\"> 70. </param>"
        "<param type=\"1\" name=\"min_z\"> 30. </param>"
        "<param type=\"1\" name=\"max_z\"> 80. </param>"
        "</filter>";
    BOOST_CHECK(filter_bbox.InitFromXmlText(filter_config.c_str()));
    common::BoundingBox bbox(Eigen::Vector3d(10., 20., 30.),
                             Eigen::Vector3d(80., 70., 80.));
    for (int i = 0; i < 100; ++i) {
      filter_bbox.Filter(filtered_cloud);
      for (const auto& point : filtered_cloud->points) {
        Eigen::Vector3d point_eigen(point.x, point.y, point.z);
        BOOST_CHECK(!bbox.ContainsPoint(point_eigen));
      }
      BOOST_CHECK_EQUAL(filtered_cloud->stamp, raw_cloud->stamp);
    }
  }
}

}  // namespace filter
}  // namespace pre_processers
}  // namespace static_map
