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

#ifndef PRE_PROCESSORS_FILTER_GROUND_REMOVAL_H_
#define PRE_PROCESSORS_FILTER_GROUND_REMOVAL_H_

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "pre_processors/filter_interface.h"

namespace static_map {
namespace pre_processers {
namespace filter {

template <typename PointT>
class GroundRemoval : public Interface<PointT> {
 public:
  USE_POINTCLOUD;

  GroundRemoval()
      : Interface<PointT>(),
        min_point_num_in_voxel_(10),
        leaf_size_(0.8),
        height_threshold_(0.15) {
    // float params
    INIT_INNER_PARAM(Interface<PointT>::kFloatParam, 0, "leaf_size",
                     leaf_size_);
    INIT_INNER_PARAM(Interface<PointT>::kFloatParam, 1, "height_threshold",
                     height_threshold_);
    // int32_t params
    INIT_INNER_PARAM(Interface<PointT>::kInt32Param, 0,
                     "min_point_num_in_voxel", min_point_num_in_voxel_);
  }
  ~GroundRemoval() {}

  GroundRemoval(const GroundRemoval&) = delete;
  GroundRemoval& operator=(const GroundRemoval&) = delete;

  std::shared_ptr<Interface<PointT>> CreateNewInstance() override {
    return std::make_shared<GroundRemoval<PointT>>();
  }

  void SetInputCloud(const PointCloudPtr& cloud) override {
    this->inliers_.clear();
    this->outliers_.clear();
    if (cloud == nullptr || cloud->empty()) {
      LOG(WARNING) << "cloud empty, do nothing!" << std::endl;
      this->inner_cloud_ = nullptr;
      return;
    }
    this->inner_cloud_ = cloud;

    voxels_.clear();
    for (int i = 0; i < this->inner_cloud_->size(); ++i) {
      auto& point = this->inner_cloud_->points[i];
      Eigen::Vector3i index(static_cast<int>(point.x / leaf_size_),
                            static_cast<int>(point.y / leaf_size_),
                            static_cast<int>(point.z / leaf_size_));
      voxels_[index].push_back(i);
    }
  }

  void Filter(const PointCloudPtr& cloud) override {
    if (!cloud || !Interface<PointT>::inner_cloud_) {
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
          cloud->push_back(this->inner_cloud_->points[i]);
        }
      }
    }
    std::sort(this->inliers_.begin(), this->inliers_.end());
    std::sort(this->outliers_.begin(), this->outliers_.end());
  }

  void DisplayAllParams() override {
    PARAM_INFO(min_point_num_in_voxel_);
    PARAM_INFO(leaf_size_);
    PARAM_INFO(height_threshold_);
  }

 private:
  struct VectorCompare {
    bool operator()(const Eigen::Vector3i a, const Eigen::Vector3i b) const {
      return std::forward_as_tuple(a[0], a[1], a[2]) <
             std::forward_as_tuple(b[0], b[1], b[2]);
    }
  };
  std::map<Eigen::Vector3i, std::vector<int>, VectorCompare> voxels_;

  // parameters
  int32_t min_point_num_in_voxel_;
  float leaf_size_;
  float height_threshold_;
};

}  // namespace filter
}  // namespace pre_processers
}  // namespace static_map

#endif  // PRE_PROCESSORS_FILTER_GROUND_REMOVAL_H_
