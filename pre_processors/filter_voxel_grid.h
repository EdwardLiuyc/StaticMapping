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

#ifndef PRE_PROCESSORS_FILTER_VOXEL_GRID_H_
#define PRE_PROCESSORS_FILTER_VOXEL_GRID_H_

#include <memory>
#include <unordered_map>
#include <vector>

#include "common/eigen_hash.h"
#include "pre_processors/filter_interface.h"

namespace static_map {
namespace pre_processers {
namespace filter {

template <typename PointT>
class VoxelGrid : public Interface<PointT> {
 public:
  USE_POINTCLOUD;

  VoxelGrid() : Interface<PointT>() {}
  ~VoxelGrid() = default;

  PROHIBIT_COPY_AND_ASSIGN(VoxelGrid);

  std::shared_ptr<Interface<PointT>> CreateNewInstance() override {
    return std::make_shared<VoxelGrid<PointT>>();
  }

  void Filter(const PointCloudPtr& cloud) override {
    if (!cloud || !Interface<PointT>::inner_cloud_) {
      LOG(WARNING) << "nullptr cloud, do nothing!" << std::endl;
      return;
    }

    this->FilterPrepare(cloud);
    std::unordered_map<Eigen::Vector3i, std::vector<PointT>> voxel_grid;
    for (const auto& point : this->inner_cloud_->points) {
      Eigen::Vector3i index(std::lround(point.x / voxel_size_),
                            std::lround(point.y / voxel_size_),
                            std::lround(point.z / voxel_size_));
      voxel_grid[index].push_back(point);
    }

    const auto get_average_point =
        [](const std::vector<PointT>& points) -> PointT {
      CHECK(!points.empty());

      double sum[4] = {0, 0, 0, 0};
      for (const auto& point : points) {
        sum[0] += point.x;
        sum[1] += point.y;
        sum[2] += point.z;
        sum[3] += point.intensity;
      }

      const int size = points.size();
      PointT result;
      result.x = sum[0] / size;
      result.y = sum[1] / size;
      result.z = sum[2] / size;
      result.intensity = sum[3] / size;
      return result;
    };

    cloud->points.reserve(voxel_grid.size());
    for (const auto& grid : voxel_grid) {
      cloud->points.push_back(get_average_point(grid.second));
    }
  }

  void DisplayAllParams() override { PARAM_INFO(voxel_size_); }

 private:
  float voxel_size_ = 0.1f;
};

}  // namespace filter
}  // namespace pre_processers
}  // namespace static_map

#endif  // PRE_PROCESSORS_FILTER_VOXEL_GRID_H_
