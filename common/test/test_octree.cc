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

#include "common/octree.h"
#define BOOST_TEST_MODULE OctreeTest
#include <boost/test/unit_test.hpp>

namespace static_map {
namespace common {

BOOST_AUTO_TEST_CASE(InitWithPointCloud) {
  OctreeConfig config;
  config.max_depth = 10;
  config.max_num_points_in_voxel = 4;
  config.max_size = 10.;
  config.center << 0, 0, 0;

  Octree octree(config);

  Eigen::MatrixXd points(12, 3);
  // clang-format off
  points <<
    0., 0., 0.,
    1., 1., 1.,
    2., 2., 2.,
    -1., -1., -1.,
    1.5, 1., 0.,
    2., 1., 3.,
    -1., 3., 1.,
    5., 4., 2.,
    2., -5., -4.,
    -4., 2., -2.,
    1., 7., 1.,
    3., 1., -2.;
  // clang-format on
  octree.InitWithPointCloud(points.transpose());
  BOOST_CHECK_EQUAL(2, octree.GetMaxDepth());
  // octree.RecursiveOutput();
}
}  // namespace common
}  // namespace static_map
