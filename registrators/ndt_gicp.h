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

#ifndef REGISTRATORS_NDT_GICP_H_
#define REGISTRATORS_NDT_GICP_H_

#include <cmath>
#include <vector>

#include "pcl/filters/approximate_voxel_grid.h"
#include "pcl/registration/gicp.h"
#include "pcl/registration/ndt.h"
#include "pcl/search/impl/search.hpp"

#include "common/macro_defines.h"
#include "registrators/interface.h"

namespace static_map {
namespace registrator {

template <typename PointType>
class NdtWithGicp : public Interface<PointType> {
 public:
  USE_REGISTRATOR_CLOUDS;

  NdtWithGicp();
  ~NdtWithGicp() = default;

  PROHIBIT_COPY_AND_ASSIGN(NdtWithGicp);

  bool Align(const Eigen::Matrix4d& guess, Eigen::Matrix4d& result) override;

  void InitWithOptions() override;

 private:
  pcl::NormalDistributionsTransform<PointType, PointType> ndt_;
  pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp_;

  PointCloudSourcePtr down_sampled_source_cloud_ = nullptr;
  PointCloudTargetPtr down_sampled_target_cloud_ = nullptr;

  pcl::ApproximateVoxelGrid<PointType> approximate_voxel_filter_;

  struct {
    float voxel_resolution = 0.2;
    bool using_voxel_filter = true;
    bool use_ndt = true;
  } options_;
};
}  // namespace registrator
}  // namespace static_map

#endif  // REGISTRATORS_NDT_GICP_H_
