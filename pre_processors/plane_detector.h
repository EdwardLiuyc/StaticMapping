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

#ifndef PRE_PROCESSORS_PLANE_DETECTOR_H_
#define PRE_PROCESSORS_PLANE_DETECTOR_H_

// third party
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
// stl
#include <map>
#include <vector>
// local
#include "common/macro_defines.h"
#include "glog/logging.h"

namespace static_map {

template <typename PointT>
class PlaneDetector {
 public:
  using PointCloudType = typename pcl::PointCloud<PointT>;
  using PointCloudPtr = typename PointCloudType::Ptr;
  using PointCloudConstPtr = typename PointCloudType::ConstPtr;

  PlaneDetector()
      : min_point_num_in_voxel_(10),
        min_z_(1.e9),
        max_z_(-1.e9),
        leaf_size_(0.5) {}
  ~PlaneDetector() = default;

  void SetMinPointNumInVoxel(size_t num) {
    const size_t default_minimum = 5;
    if (num < default_minimum) {
      LOG(WARNING) << "setting num is too small, reset to default minimum : "
                   << default_minimum << std::endl;
      min_point_num_in_voxel_ = default_minimum;
    } else {
      min_point_num_in_voxel_ = num;
    }
  }

  inline void SetLeafSize(float leaf_size) { leaf_size_ = leaf_size; }

  void SetInputCloud(const PointCloudPtr& cloud) {
    if (!cloud || cloud->empty()) {
      PRINT_ERROR("cloud is empty.");
      input_cloud_ = nullptr;
      return;
    }
    input_cloud_ = cloud;

    for (int i = 0; i < input_cloud_->size(); ++i) {
      auto& point = input_cloud_->points[i];
      Eigen::Vector3i index(static_cast<int>(point.x / leaf_size_),
                            static_cast<int>(point.y / leaf_size_),
                            static_cast<int>(point.z / leaf_size_));
      voxels_[index].push_back(i);
      if (point.z > max_z_) {
        max_z_ = point.z;
      }
      if (point.z < min_z_) {
        min_z_ = point.z;
      }
    }
  }

  void Detect(std::vector<int>& ground_indices,  // NOLINT
              float distance_threshold = 0.2) {
    if (!input_cloud_ || voxels_.empty()) {
      PRINT_ERROR("input cloud is nullptr, nothing to filter.");
      return;
    }
    if (distance_threshold >= leaf_size_) {
      PRINT_ERROR("distance_threshold is too large.");
      return;
    }

    for (auto& index_vector : voxels_) {
      Eigen::Vector3i index = index_vector.first;
      auto& indices = index_vector.second;
      if (indices.size() < min_point_num_in_voxel_) {
        continue;
      }
      if (index[2] /* z index */ <= 0) {
        float max_z_in_voxel = -1.e9;
        float min_z_in_voxel = 1.e9;
        for (auto& i : indices) {
          auto& point = input_cloud_->points[i];
          if (point.z > max_z_in_voxel) {
            max_z_in_voxel = point.z;
          }
          if (point.z < min_z_in_voxel) {
            min_z_in_voxel = point.z;
          }
        }
        auto delta = max_z_in_voxel - min_z_in_voxel;
        if (delta >= 0. && delta <= distance_threshold) {
          for (auto& i : indices) {
            ground_indices.push_back(i);
          }
        }
      } else {
        if (indices.size() < min_point_num_in_voxel_ * 2) {
          continue;
        }
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<PointT> seg;  // Create the segmentation object
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.1);
        PointCloudPtr voxel_cloud(new PointCloudType);
        for (auto& i : indices) {
          voxel_cloud->push_back(input_cloud_->points[i]);
        }
        seg.setInputCloud(voxel_cloud);
        seg.segment(*inliers, *coefficients);
        float inliers_rate =
            static_cast<float>(inliers->indices.size()) / indices.size();
        if (inliers_rate > 0.85) {
          for (auto index : inliers->indices) {
            ground_indices.push_back(indices[index]);
          }
        }
      }
    }
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  PointCloudPtr input_cloud_;

  struct VectorCompare {
    bool operator()(const Eigen::Vector3i a, const Eigen::Vector3i b) const {
      return std::forward_as_tuple(a[0], a[1], a[2]) <
             std::forward_as_tuple(b[0], b[1], b[2]);
    }
  };
  std::map<Eigen::Vector3i, std::vector<int>, VectorCompare> voxels_;
  size_t min_point_num_in_voxel_;

  float min_z_;
  float max_z_;
  float leaf_size_;
};

}  // namespace static_map

#endif  // PRE_PROCESSORS_PLANE_DETECTOR_H_
