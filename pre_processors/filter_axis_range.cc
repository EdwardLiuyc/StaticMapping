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

#include "pre_processors/filter_axis_range.h"

namespace static_map {
namespace pre_processers {
namespace filter {

AxisRange::AxisRange() : Interface(), axis_index_(2) {
  INIT_FLOAT_PARAM("min", min_);
  INIT_FLOAT_PARAM("max", max_);
  INIT_INT32_PARAM("axis_index", axis_index_);
}

void AxisRange::DisplayAllParams() {
  PARAM_INFO(min_);
  PARAM_INFO(max_);
  PARAM_INFO(axis_index_);
}

bool AxisRange::ConfigsValid() const {
  return (max_ > min_) && (axis_index_ >= 0) && (axis_index_ <= 2);
}

void AxisRange::Filter(const data::InnerCloudType::Ptr& cloud) {
  if (!cloud || !Interface::inner_cloud_) {
    LOG(WARNING) << "nullptr cloud, do nothing!" << std::endl;
    return;
  }
  if (min_ == -std::numeric_limits<float>::max() &&
      max_ == std::numeric_limits<float>::max()) {
    *cloud = *this->inner_cloud_;
    for (int i = 0; i < this->inner_cloud_->points.size(); ++i) {
      this->inliers_.push_back(i);
    }
    this->outliers_.clear();
    return;
  }

  this->FilterPrepare(cloud);

  auto& input = this->inner_cloud_;
  const int size = input->points.size();
  this->inliers_.reserve(size);
  this->outliers_.reserve(size);
  cloud->points.reserve(size);
  // TODO(edward) Filter all axis in one filter (for efficiency)
  for (int i = 0; i < size; ++i) {
    const auto& point = input->points[i];
    bool is_outlier = false;
    switch (static_cast<Axis>(axis_index_)) {
      case Axis::kX:
        if (point.x < min_ || point.x > max_) {
          is_outlier = true;
        }
        break;
      case Axis::kY:
        if (point.y < min_ || point.y > max_) {
          is_outlier = true;
        }
        break;
      case Axis::kZ:
        if (point.z < min_ || point.z > max_) {
          is_outlier = true;
        }
        break;

      default:
        break;
    }

    if (!is_outlier) {
      cloud->points.push_back(input->points[i]);
      this->inliers_.push_back(i);
    } else {
      this->outliers_.push_back(i);
    }
  }
  this->inliers_.shrink_to_fit();
  this->outliers_.shrink_to_fit();
  cloud->points.shrink_to_fit();
}

}  // namespace filter
}  // namespace pre_processers
}  // namespace static_map
