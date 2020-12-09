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

#include "pre_processors/filter_random_sample.h"

#define BOOST_TEST_MODULE FilterRange

#include "boost/test/unit_test.hpp"
#include "common/math.h"
#include "test/test_helper.h"

namespace static_map {
namespace pre_processers {
namespace filter {

using test::CreateRandomInnerCloud;

BOOST_AUTO_TEST_CASE(FilterConfig) {
  RandomSampler filter_rs;
  {
    const std::string filter_config = "<filter name=\"RandomSampler\" />";
    BOOST_CHECK(filter_rs.InitFromXmlText(filter_config.c_str()));
  }
  {
    const std::string filter_config = "<filter name=\"RandomSamplerSSS\" />";
    BOOST_CHECK(!filter_rs.InitFromXmlText(filter_config.c_str()));
  }
  {
    const std::string filter_config =
        "<filter name=\"RandomSampler\" >"
        "<param type=\"1\" name=\"sampling_rate\"> 0.5 </param>"
        "</filter>";
    BOOST_CHECK(filter_rs.InitFromXmlText(filter_config.c_str()));
  }
  {
    const std::string filter_config =
        "<filter name=\"RandomSampler\" >"
        "<param type=\"1\" name=\"sampling_rate\"> 1.5 </param>"
        "</filter>";
    BOOST_CHECK(!filter_rs.InitFromXmlText(filter_config.c_str()));
  }
}

BOOST_AUTO_TEST_CASE(Filter) {
  RandomSampler filter_rs;
  const std::string filter_config =
      "<filter name=\"RandomSampler\" >"
      "<param type=\"1\" name=\"sampling_rate\"> 0.5 </param>"
      "</filter>";
  BOOST_CHECK(filter_rs.InitFromXmlText(filter_config.c_str()));

  const int raw_point_size = 100000;
  const auto raw_cloud = CreateRandomInnerCloud(raw_point_size);
  filter_rs.SetInputCloud(raw_cloud);
  data::InnerCloudType::Ptr filtered_cloud(new data::InnerCloudType);

  for (int i = 0; i < 100; ++i) {
    filter_rs.Filter(filtered_cloud);
    BOOST_CHECK_EQUAL(filtered_cloud->stamp, raw_cloud->stamp);
    BOOST_CHECK_GT(
        static_cast<double>(filtered_cloud->points.size()) / raw_point_size,
        0.48);
    BOOST_CHECK_LT(
        static_cast<double>(filtered_cloud->points.size()) / raw_point_size,
        0.52);
  }
}

}  // namespace filter
}  // namespace pre_processers
}  // namespace static_map
