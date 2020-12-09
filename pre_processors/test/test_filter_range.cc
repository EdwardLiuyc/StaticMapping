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

#include "pre_processors/filter_range.h"

#define BOOST_TEST_MODULE FilterRange

#include "boost/test/unit_test.hpp"
#include "common/math.h"
#include "test/test_helper.h"

namespace static_map {
namespace pre_processers {
namespace filter {

using test::CreateRandomInnerCloud;

BOOST_AUTO_TEST_CASE(Filter) {
  Range filter_range;
  const int cloud_size = 1000;
  const auto raw_cloud = CreateRandomInnerCloud(cloud_size);

  filter_range.SetInputCloud(raw_cloud);
  data::InnerCloudType::Ptr filtered_cloud(new data::InnerCloudType);
  {
    filter_range.Filter(filtered_cloud);
    BOOST_CHECK_EQUAL(filtered_cloud->stamp, raw_cloud->stamp);
    BOOST_CHECK_EQUAL(filtered_cloud->points.size(), raw_cloud->points.size());
    for (int i = 0; i < cloud_size; ++i) {
      const auto& raw_point = raw_cloud->points.at(i);
      const auto& filtered_point = filtered_cloud->points.at(i);
      BOOST_CHECK_EQUAL(filtered_point.x, raw_point.x);
      BOOST_CHECK_EQUAL(filtered_point.y, raw_point.y);
      BOOST_CHECK_EQUAL(filtered_point.z, raw_point.z);
      BOOST_CHECK_EQUAL(filtered_point.intensity, raw_point.intensity);
      BOOST_CHECK_EQUAL(filtered_point.factor, raw_point.factor);
    }
  }
  {
    const std::string filter_config =
        "<filter name=\"Range\" >"
        "<param type=\"1\" name=\"min_range\"> 20. </param>"
        "<param type=\"1\" name=\"max_range\"> 80. </param>"
        "</filter>";
    BOOST_CHECK(filter_range.InitFromXmlText(filter_config.c_str()));
    for (int i = 0; i < 100; ++i) {
      filter_range.Filter(filtered_cloud);
      for (const auto& point : filtered_cloud->points) {
        const float range = std::sqrt(point.x * point.x + point.y * point.y +
                                      point.z * point.z);
        BOOST_CHECK_LE(range, 80.);
        BOOST_CHECK_GE(range, 20.);
      }
    }
  }
}

}  // namespace filter
}  // namespace pre_processers
}  // namespace static_map
