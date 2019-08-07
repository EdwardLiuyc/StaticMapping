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

#pragma once

#include <memory>
#include <vector>

#include "registrators/cuda/icp_cuda.h"
#include "registrators/registrator_interface.h"

namespace static_map {
namespace registrator {

constexpr int kDim = 3;

struct InnerPoint {
  Eigen::Vector3f point = Eigen::Vector3f::Zero();
  Eigen::Vector3f normal = Eigen::Vector3f::Zero();
  double intensity;
  bool has_normal = false;

  inline void FromFloatArray(const float* const array) {
    point << array[0], array[1], array[2];
  }

  inline void ToFloatArray(float* const array) {
    array[0] = point[0];
    array[1] = point[1];
    array[2] = point[2];
  }

  inline void FromPoint(const pcl::PointXYZI& p) {
    point[0] = p.x;
    point[1] = p.y;
    point[2] = p.z;
    intensity = p.intensity;
  }
  inline void FromPoint(const pcl::PointXYZ& p) {
    point[0] = p.x;
    point[1] = p.y;
    point[2] = p.z;
    intensity = 0.;
  }
};

struct InnerPointCloud {
  std::vector<InnerPoint> points;

  void ToFloatArray(float* const array) {
    CHECK(array);
    float* start = array;
    for (auto& p : points) {
      p.ToFloatArray(start);
      start += kDim;
    }
  }

  using Ptr = std::unique_ptr<InnerPointCloud>;
  using ConstPtr = std::unique_ptr<const InnerPointCloud>;
};

template <typename PointType>
class IcpFast : public Interface<PointType> {
 public:
  USE_REGISTRATOR_CLOUDS;

  IcpFast() { this->type_ = kFastIcp; }

  void setInputSource(const PointCloudSourcePtr& cloud) override;
  void setInputTarget(const PointCloudTargetPtr& cloud) override;
  bool align(const Eigen::Matrix4f& guess, Eigen::Matrix4f& result) override;

 private:
  typename InnerPointCloud::Ptr source_inner_cloud_;
  typename InnerPointCloud::Ptr target_inner_cloud_;

  Eigen::Matrix<float, Eigen::Dynamic, 6> target_cloud_with_mormal_;
};

}  // namespace registrator
}  // namespace static_map
