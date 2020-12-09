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

#include "pre_processors/filter_voxel_grid.h"

namespace static_map {
namespace pre_processers {
namespace filter {

VoxelGrid::VoxelGrid() : Interface() {
  INIT_FLOAT_PARAM("voxel_size", voxel_size_);
}

void VoxelGrid::DisplayAllParams() { PARAM_INFO(voxel_size_); }

bool VoxelGrid::ConfigsValid() const { return voxel_size_ > 1.e-6; }

void VoxelGrid::Filter(const data::InnerCloudType::Ptr& cloud) {
  if (!cloud || !Interface::inner_cloud_) {
    LOG(WARNING) << "nullptr cloud, do nothing!" << std::endl;
    return;
  }

  this->FilterPrepare(cloud);
  std::unordered_map<Eigen::Vector3i, std::vector<data::InnerPointType>>
      voxel_grid;
  for (const auto& point : this->inner_cloud_->points) {
    Eigen::Vector3i index(std::lround(point.x / voxel_size_),
                          std::lround(point.y / voxel_size_),
                          std::lround(point.z / voxel_size_));
    voxel_grid[index].push_back(point);
  }

  const auto get_average_point =
      [](const std::vector<data::InnerPointType>& points)
      -> data::InnerPointType {
    CHECK(!points.empty());

    double sum[4] = {0, 0, 0, 0};
    for (const auto& point : points) {
      sum[0] += point.x;
      sum[1] += point.y;
      sum[2] += point.z;
      sum[3] += point.intensity;
    }

    const int size = points.size();
    data::InnerPointType result;
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

}  // namespace filter
}  // namespace pre_processers
}  // namespace static_map
