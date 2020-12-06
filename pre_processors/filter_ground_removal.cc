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

#include "pre_processors/filter_ground_removal.h"

namespace static_map {
namespace pre_processers {
namespace filter {

GroundRemoval::GroundRemoval()
    : Interface(),
      min_point_num_in_voxel_(10),
      leaf_size_(0.8),
      height_threshold_(0.15) {
  // float params
  INIT_FLOAT_PARAM("leaf_size", leaf_size_);
  INIT_FLOAT_PARAM("height_threshold", height_threshold_);
  // int32_t params
  INIT_INT32_PARAM("min_point_num_in_voxel", min_point_num_in_voxel_);
}

void GroundRemoval::DisplayAllParams() {
  PARAM_INFO(min_point_num_in_voxel_);
  PARAM_INFO(leaf_size_);
  PARAM_INFO(height_threshold_);
}

void GroundRemoval::SetInputCloud(const data::InnerCloudType::Ptr& cloud) {
  this->inliers_.clear();
  this->outliers_.clear();
  if (cloud == nullptr || cloud->points.empty()) {
    LOG(WARNING) << "cloud empty, do nothing!" << std::endl;
    this->inner_cloud_ = nullptr;
    return;
  }
  this->inner_cloud_ = cloud;

  voxels_.clear();
  for (int i = 0; i < this->inner_cloud_->points.size(); ++i) {
    auto& point = this->inner_cloud_->points[i];
    Eigen::Vector3i index(static_cast<int>(point.x / leaf_size_),
                          static_cast<int>(point.y / leaf_size_),
                          static_cast<int>(point.z / leaf_size_));
    voxels_[index].push_back(i);
  }
}

void GroundRemoval::Filter(const data::InnerCloudType::Ptr& cloud) {
  if (!cloud || !Interface::inner_cloud_) {
    LOG(WARNING) << "nullptr cloud, do nothing!" << std::endl;
    return;
  }

  this->FilterPrepare(cloud);
  for (auto& index_vector : voxels_) {
    auto& index = index_vector.first;
    auto& vector = index_vector.second;
    if ((int32_t)vector.size() < min_point_num_in_voxel_) {
      continue;
    }
    bool is_ground = false;
    if (index[2] <= 0) {  // z index <= 0  ---> z <= leaf_size
      float max_z_in_voxel = -1.e9;
      float min_z_in_voxel = 1.e9;
      for (auto& i : vector) {
        auto& point = this->inner_cloud_->points[i];
        if (point.z > max_z_in_voxel) {
          max_z_in_voxel = point.z;
        }
        if (point.z < min_z_in_voxel) {
          min_z_in_voxel = point.z;
        }
      }
      auto delta = max_z_in_voxel - min_z_in_voxel;
      if (delta >= 0. && delta <= height_threshold_) {
        is_ground = true;
        for (auto& i : vector) {
          this->outliers_.push_back(i);
        }
      }
    }

    if (!is_ground) {
      for (auto& i : vector) {
        this->inliers_.push_back(i);
        cloud->points.push_back(this->inner_cloud_->points[i]);
      }
    }
  }
  std::sort(this->inliers_.begin(), this->inliers_.end());
  std::sort(this->outliers_.begin(), this->outliers_.end());
}

}  // namespace filter
}  // namespace pre_processers
}  // namespace static_map
