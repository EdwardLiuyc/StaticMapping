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

#ifndef COMMON_POINT_UTILS_H_
#define COMMON_POINT_UTILS_H_

#include <glog/logging.h>
#include <cfloat>
#include <cmath>

namespace static_map {
namespace common {

constexpr double kValidLength = 100000000.;

template <typename PointT>
double Length(const PointT& point) {
  return std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
}

template <typename PointT>
bool CheckPoint(const PointT& point) {
  if (std::isinf(point.x) || std::isinf(point.y) || std::isinf(point.z) ||
      std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
    return false;
  }
  const double length = Length(point);
  if (std::isinf(length) || length >= kValidLength) {
    return false;
  }
  return true;
}

}  // namespace common
}  // namespace static_map

#define FATAL_CHECK_POINT(point)                             \
  {                                                          \
    CHECK(std::isfinite(point.x) && !std::isnan(point.x));   \
    CHECK(std::isfinite(point.y) && !std::isnan(point.y));   \
    CHECK(std::isfinite(point.z) && !std::isnan(point.z));   \
    const double length = static_map::common::Length(point); \
    CHECK(std::isfinite(length) && !std::isnan(length));     \
    CHECK_LT(length, static_map::common::kValidLength);      \
  }

#define FATAL_CHECK_CLOUD(cloud)              \
  {                                           \
    for (const auto& point : cloud->points) { \
      FATAL_CHECK_POINT(point);               \
    }                                         \
  }

#endif  // COMMON_POINT_UTILS_H_
