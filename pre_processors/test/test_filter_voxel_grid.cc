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

#include "pre_processors/filter_voxel_grid.h"

#define BOOST_TEST_MODULE FilterRange

#include "boost/test/unit_test.hpp"
#include "common/math.h"

namespace static_map {
namespace pre_processers {
namespace filter {

BOOST_AUTO_TEST_CASE(FilterConfig) {
  VoxelGrid filter_voxel_grid;
  {
    const std::string filter_config =
        "<filter name=\"VoxelGrid\" >"
        "<param type=\"1\" name=\"voxel_size\"> 0. </param>"
        "</filter>";
    BOOST_CHECK(!filter_voxel_grid.InitFromXmlText(filter_config.c_str()));
  }
  {
    const std::string filter_config =
        "<filter name=\"VoxelGrid\" >"
        "<param type=\"1\" name=\"voxel_size\"> 10. </param>"
        "</filter>";
    BOOST_CHECK(filter_voxel_grid.InitFromXmlText(filter_config.c_str()));
  }
}

BOOST_AUTO_TEST_CASE(Filter) {
  VoxelGrid filter_voxel_grid;
  data::InnerCloudType::Ptr raw_cloud(new data::InnerCloudType);
  raw_cloud->stamp = SimpleTime::GetCurrentTime();
  for (int x = 0; x < 10; ++x) {
    for (int y = 0; y < 10; ++y) {
      raw_cloud->points.push_back(data::InnerPointType{.x = x * 0.1f + 0.02f,
                                                       .y = y * 0.1f + 0.02f,
                                                       .z = 0.1f,
                                                       .intensity = 0.f});
    }
  }

  filter_voxel_grid.SetInputCloud(raw_cloud);
  data::InnerCloudType::Ptr filtered_cloud(new data::InnerCloudType);
  {
    const std::string filter_config =
        "<filter name=\"VoxelGrid\" >"
        "<param type=\"1\" name=\"voxel_size\"> 0.1 </param>"
        "</filter>";
    BOOST_CHECK(filter_voxel_grid.InitFromXmlText(filter_config.c_str()));

    filter_voxel_grid.Filter(filtered_cloud);
    BOOST_CHECK_EQUAL(filtered_cloud->stamp, raw_cloud->stamp);
    BOOST_CHECK_EQUAL(filtered_cloud->points.size(), raw_cloud->points.size());
  }
  {
    const std::string filter_config =
        "<filter name=\"VoxelGrid\" >"
        "<param type=\"1\" name=\"voxel_size\"> 0.2 </param>"
        "</filter>";
    BOOST_CHECK(filter_voxel_grid.InitFromXmlText(filter_config.c_str()));

    filter_voxel_grid.Filter(filtered_cloud);
    BOOST_CHECK_EQUAL(filtered_cloud->stamp, raw_cloud->stamp);
    BOOST_CHECK_EQUAL(filtered_cloud->points.size(), 36);
  }
  {
    const std::string filter_config =
        "<filter name=\"VoxelGrid\" >"
        "<param type=\"1\" name=\"voxel_size\"> 0.4 </param>"
        "</filter>";
    BOOST_CHECK(filter_voxel_grid.InitFromXmlText(filter_config.c_str()));

    filter_voxel_grid.Filter(filtered_cloud);
    BOOST_CHECK_EQUAL(filtered_cloud->stamp, raw_cloud->stamp);
    BOOST_CHECK_EQUAL(filtered_cloud->points.size(), 9);
  }
}

}  // namespace filter
}  // namespace pre_processers
}  // namespace static_map
