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

#include "common/bounding_box.h"

namespace static_map {
namespace pre_processers {
namespace filter {

BoundingBoxRemoval::BoundingBoxRemoval() : Interface() {
  INIT_FLOAT_PARAM("min_x", min_x_);
  INIT_FLOAT_PARAM("min_y", min_y_);
  INIT_FLOAT_PARAM("min_z", min_z_);
  INIT_FLOAT_PARAM("max_x", max_x_);
  INIT_FLOAT_PARAM("max_y", max_y_);
  INIT_FLOAT_PARAM("max_z", max_z_);
}

void BoundingBoxRemoval::DisplayAllParams() {
  PARAM_INFO(min_x_);
  PARAM_INFO(min_y_);
  PARAM_INFO(min_z_);
  PARAM_INFO(max_x_);
  PARAM_INFO(max_y_);
  PARAM_INFO(max_z_);
}

bool BoundingBoxRemoval::ConfigsValid() const {
  return min_x_ < max_x_ && min_y_ < max_y_ && min_z_ < max_z_;
}

void BoundingBoxRemoval::Filter(const data::InnerCloudType::Ptr& cloud) {
  if (!cloud || !Interface::inner_cloud_) {
    LOG(WARNING) << "nullptr cloud, do nothing!" << std::endl;
    return;
  }
  this->FilterPrepare(cloud);

  const auto bbox =
      common::BoundingBox(Eigen::Vector3d(min_x_, min_y_, min_z_),
                          Eigen::Vector3d(max_x_, max_y_, max_z_));

  auto& input = this->inner_cloud_;
  const int size = input->points.size();
  this->inliers_.reserve(size);
  this->outliers_.reserve(size);
  cloud->points.reserve(size);
  for (int i = 0; i < size; ++i) {
    const auto& point = input->points[i];
    const Eigen::Vector3d point_eigen(point.x, point.y, point.z);
    bool is_outlier = bbox.ContainsPoint(point_eigen);
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
