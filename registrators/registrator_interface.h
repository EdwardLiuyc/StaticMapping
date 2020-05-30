
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

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "common/macro_defines.h"

namespace static_map {
namespace registrator {

enum Type {
  kNoType,
  kIcpPM,
  kLibicp,  // deprecated
  kNdtWithGicp,
  kLegoLoam,
  kNdt,
  kFastIcp,
  kTypeCount
};

struct InlierPointPairs {
  Eigen::Matrix<float, Eigen::Dynamic, 3> ref_points, read_points;
  size_t pairs_num;
};

template <typename PointType>
class Interface {
 public:
  typedef pcl::PointCloud<PointType> PointCloudSource;
  typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
  typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

  typedef pcl::PointCloud<PointType> PointCloudTarget;
  typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
  typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;

  Interface() = default;
  ~Interface() = default;

  virtual void setInputSource(const PointCloudSourcePtr& cloud) {
    if (!cloud) {
      source_cloud_ = nullptr;
      return;
    }
    if (cloud->empty()) {
      PRINT_WARNING("cloud is empty.");
      return;
    }
    source_cloud_ = cloud;
  }

  virtual void setInputTarget(const PointCloudTargetPtr& cloud) {
    if (!cloud) {
      target_cloud_ = nullptr;
      return;
    }
    if (cloud->empty()) {
      PRINT_WARNING("cloud is empty.");
      return;
    }
    target_cloud_ = cloud;
  }

  virtual double getFitnessScore() { return final_score_; }
  virtual InlierPointPairs getInlierPointPairs() { return point_pairs_; }

  // need to be implemented by child class
  virtual bool align(const Eigen::Matrix4f& guess, Eigen::Matrix4f& result) = 0;

 protected:
  double final_score_;

  PointCloudSourcePtr source_cloud_ = nullptr;
  PointCloudTargetPtr target_cloud_ = nullptr;

  Type type_ = kNoType;
  InlierPointPairs point_pairs_;
};

}  // namespace registrator
}  // namespace static_map

#define USE_REGISTRATOR_CLOUDS                              \
  using typename Interface<PointType>::PointCloudSource;    \
  using typename Interface<PointType>::PointCloudTarget;    \
  using typename Interface<PointType>::PointCloudSourcePtr; \
  using typename Interface<PointType>::PointCloudTargetPtr;
