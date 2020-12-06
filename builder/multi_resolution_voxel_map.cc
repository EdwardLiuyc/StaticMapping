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

// local
#include "builder/multi_resolution_voxel_map.h"
#include "common/point_utils.h"

namespace static_map {
namespace {
constexpr float kMinHitProb = 0.501f;
constexpr float kMaxMissProb = 0.499f;
constexpr float kMaxProb = 0.9f;
constexpr float kMinProb = 0.1f;
}  // namespace

void MultiResolutionVoxelMap::Initialise(const MrvmSettings& settings) {
  settings_ = settings;
  CHECK_GT(settings_.max_point_num_in_cell, 0);

  settings_.hit_prob = Clamp(settings_.hit_prob, kMinHitProb, kMaxProb);
  settings_.miss_prob = Clamp(settings_.miss_prob, kMinProb, kMaxMissProb);
}

void MultiResolutionVoxelMap::SetOffsetZ(const float& offset) {
  settings_.z_offset = offset;
}

void MultiResolutionVoxelMap::InsertPointCloud(
    const data::InnerCloudType::Ptr& cloud, const Eigen::Vector3f& origin) {
  if (!cloud || cloud->points.empty()) {
    PRINT_ERROR("cloud is empty.");
    return;
  }

  Eigen::Vector3f offseted_origin = origin;
  offseted_origin[2] += settings_.z_offset;
  const float hit_log_odd = ProbabilityToOdd(settings_.hit_prob);
  const float miss_log_odd = ProbabilityToOdd(settings_.miss_prob);
  auto update_prob = [&](Probability former_prob, bool hit) -> float {
    float odd = odds_table_[former_prob];
    odd += hit ? hit_log_odd : miss_log_odd;
    return Clamp(OddToProbability(odd), kMinProb, kMaxProb);
  };

  size_t point_index = 0;
  const size_t cloud_size = cloud->points.size();
  const float resolution = settings_.high_resolution;
  VoxelMap<bool> end_voxels;
#if defined _OPENMP
#pragma omp parallel for private(point_index) num_threads(LOCAL_OMP_THREADS_NUM)
#endif
  for (point_index = 0; point_index < cloud_size; ++point_index) {
    auto& point = cloud->points[point_index];
    Eigen::Vector3f point_vec;
    point_vec << point.x, point.y, point.z;
    std::vector<KeyInt3> high_res_indices =
        common::VoxelCastingBresenham(offseted_origin, point_vec, resolution);

    // update the end voxel
    if (!high_res_indices.empty()) {
      const KeyInt3 high_end_index = high_res_indices.back();
      high_resolution_voxels_[high_end_index].need_update = kFalse;
      end_voxels[high_end_index] = true;

      auto& voxel = high_resolution_voxels_[high_end_index];
      auto& prob = voxel.probability;
      if (static_cast<int>(point.intensity) >
          static_cast<int>(voxel.max_intensity)) {
        voxel.max_intensity = static_cast<int>(point.intensity);
      }
      prob = (Probability)(update_prob(prob, true) * kTableSize);
      if (voxel.points.size() < settings_.max_point_num_in_cell) {
        FATAL_CHECK_POINT(point);
        voxel.points.push_back(point);
      }

      // update the voxels on the line
      const size_t indices_size = high_res_indices.size();
      for (size_t i = 0; i < indices_size - 1; ++i) {
        auto& high_index = high_res_indices[i];
        if (high_resolution_voxels_.find(high_index) !=
                high_resolution_voxels_.end() &&
            high_resolution_voxels_[high_index].need_update == kTrue) {
          // update the probability
          auto& prob = high_resolution_voxels_[high_index].probability;
          prob = (Probability)(update_prob(prob, false) * kTableSize);
        }
      }
    }
  }

  for (auto& voxel : end_voxels) {
    high_resolution_voxels_[voxel.first].need_update = kTrue;
  }
}

void MultiResolutionVoxelMap::OutputToPointCloud(
    const float threshold, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
  if (!cloud) {
    PRINT_WARNING("cloud is nullptr. do nothing!");
  }
  cloud->clear();
  cloud->points.reserve(high_resolution_voxels_.size());
  const Probability prob_threshold = threshold * kTableSize;
  for (auto& high_res_voxel : high_resolution_voxels_) {
    if (high_res_voxel.second.probability >= prob_threshold) {
      CHECK(!high_res_voxel.second.points.empty());
      if (settings_.output_average) {
        pcl::PointXYZI average_point;
        for (auto& point : high_res_voxel.second.points) {
          average_point.x += point.x;
          average_point.y += point.y;
          average_point.z += point.z;
        }
        float size = high_res_voxel.second.points.size();
        average_point.x /= size;
        average_point.y /= size;
        average_point.z /= size;
        if (settings_.use_max_intensity) {
          average_point.intensity =
              static_cast<int>(high_res_voxel.second.max_intensity);
        }
        cloud->push_back(average_point);

      } else {
        for (auto& point : high_res_voxel.second.points) {
          if (settings_.use_max_intensity) {
            point.intensity =
                static_cast<int>(high_res_voxel.second.max_intensity);
          }
          pcl::PointXYZI pcl_point;
          data::ToPclPoint(point, &pcl_point);
          cloud->points.push_back(pcl_point);
        }
      }
    }
  }
  cloud->points.shrink_to_fit();
}

void MultiResolutionVoxelMap::OutputToPointCloud(
    const float threshold,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
  if (!cloud) {
    PRINT_WARNING("cloud is nullptr. do nothing!");
  }
  cloud->clear();
  cloud->points.reserve(high_resolution_voxels_.size());
  const Probability prob_threshold = threshold * kTableSize;
  for (auto& high_res_voxel : high_resolution_voxels_) {
    if (high_res_voxel.second.probability >= prob_threshold) {
      CHECK(!high_res_voxel.second.points.empty());
      // scale the intensity
      uint32_t intensity = high_res_voxel.second.max_intensity;
      intensity *= 1.4;
      if (intensity > 255) {
        intensity = 255;
      }
      const uint8_t rgb = intensity;
      if (settings_.output_average) {
        pcl::PointXYZRGB average_point;
        for (auto& point : high_res_voxel.second.points) {
          average_point.x += point.x;
          average_point.y += point.y;
          average_point.z += point.z;
        }
        float size = high_res_voxel.second.points.size();
        average_point.x /= size;
        average_point.y /= size;
        average_point.z /= size;
        average_point.r = average_point.g = average_point.b = rgb;
        cloud->push_back(average_point);

      } else {
        for (auto& point : high_res_voxel.second.points) {
          pcl::PointXYZRGB rgb_point;
          rgb_point.x = point.x;
          rgb_point.y = point.y;
          rgb_point.z = point.z;
          rgb_point.r = rgb_point.g = rgb_point.b = rgb;
          cloud->points.push_back(rgb_point);
        }
      }
    }
  }
  cloud->points.shrink_to_fit();
}

#define OUTPUT_TO_FILE(output_cloud)                                   \
  OutputToPointCloud(threshold, output_cloud);                         \
  PRINT_INFO("Finished filtering output cloud, generating pcd file."); \
  if (!output_cloud->empty()) {                                        \
    if (compress) {                                                    \
      pcl::io::savePCDFileBinaryCompressed(filename, *output_cloud);   \
    } else {                                                           \
      pcl::io::savePCDFileBinary(filename, *output_cloud);             \
    }                                                                  \
  } else {                                                             \
    PRINT_WARNING("Cloud is empty. Do not output to file.");           \
  }

void MultiResolutionVoxelMap::OutputToPointCloud(const float threshold,
                                                 const std::string& filename,
                                                 bool compress) {
  if (settings_.output_rgb) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    OUTPUT_TO_FILE(output_cloud);
  } else {
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(
        new pcl::PointCloud<pcl::PointXYZI>);
    OUTPUT_TO_FILE(output_cloud);
  }
}

#undef OUTPUT_TO_FILE

}  // namespace static_map
