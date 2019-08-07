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

#ifndef BUILDER_MULTI_RESOLUTION_VOXEL_MAP_H_
#define BUILDER_MULTI_RESOLUTION_VOXEL_MAP_H_

// stl
#include <map>
#include <string>
#include <vector>
// third party
#include "glog/logging.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
// local
#include "common/eigen_hash.h"
#include "common/macro_defines.h"
#include "common/math.h"

#if defined _OPENMP && defined _USE_TBB_
#include <tbb/atomic.h>
#include <tbb/concurrent_unordered_map.h>
#include <tbb/concurrent_vector.h>
#else
#include <atomic>
#endif

#include <type_traits>

namespace static_map {

typedef uint8_t Probability;

constexpr size_t kTableSize = (1 << (sizeof(Probability) * 8));
constexpr Probability kUnknown = (kTableSize >> 1);
constexpr float min_hit_prob = 0.501;
constexpr float max_miss_prob = 0.499;
constexpr float max_prob = 0.9;
constexpr float min_prob = 0.1;

constexpr int kTrue = 1;
constexpr int kFalse = 0;

struct MrvmSettings {
  bool output_average = false;
  float prob_threshold = 0.6f;
  float low_resolution = 1.f;  // not in use any more
  float high_resolution = 0.1f;
  float hit_prob = 0.55f;
  float miss_prob = 0.48f;
  float z_offset = 0.f;
  int max_point_num_in_cell = 10;
};

using common::Clamp;

template <typename PointT>
class MultiResolutionVoxelMap {
 public:
  using PointCloudType = typename pcl::PointCloud<PointT>;
  using PointCloudPtr = typename PointCloudType::Ptr;
  using PointCloudConstPtr = typename PointCloudType::ConstPtr;

  using KeyInt3 = Eigen::Vector3i;

#ifdef _USE_TBB_
  using PointVector =
      tbb::concurrent_vector<PointT, Eigen::aligned_allocator<PointT>>;
  using IndexVector = tbb::concurrent_vector<KeyInt3>;
  // @notice use int but not bool because atomic<bool> is not supported in tbb
  // so, use kTure(int 1) instead of true
  // as long as using kFalse(int 0) instead of false
  using AtomicBool = tbb::atomic<int>;
  using AtomicInt = tbb::atomic<int>;
  template <typename T>
  using VoxelMap =
      tbb::concurrent_unordered_map<KeyInt3, T, std::hash<KeyInt3>>;
#else
  using PointVector = std::vector<PointT, Eigen::aligned_allocator<PointT>>;
  using IndexVector = std::vector<KeyInt3>;
  using AtomicBool = std::atomic<int>;
  using AtomicInt = std::atomic<int>;

  struct VectorCompare {
    bool operator()(const KeyInt3 a, const KeyInt3 b) const {
      return std::forward_as_tuple(a[0], a[1], a[2]) <
             std::forward_as_tuple(b[0], b[1], b[2]);
    }
  };
  template <typename T>
  using VoxelMap = std::map<KeyInt3, T, VectorCompare>;
#endif

  MultiResolutionVoxelMap() {
    for (int i = 0; i < kTableSize; ++i) {
      odds_table_[i] = ProbabilityToOdd(static_cast<float>(i) / kTableSize);
    }
  }
  ~MultiResolutionVoxelMap() {}

  MultiResolutionVoxelMap(const MultiResolutionVoxelMap&) = delete;
  MultiResolutionVoxelMap& operator=(const MultiResolutionVoxelMap&) = delete;

  void InsertPointCloud(const PointCloudPtr& cloud,
                        const Eigen::Vector3f& origin);

  void OutputToPointCloud(float threshold, const PointCloudPtr& cloud);

  void OutputToPointCloud(float threshold, const std::string& filename,
                          bool compress = true) {
    PointCloudPtr output_cloud(new PointCloudType);
    OutputToPointCloud(threshold, output_cloud);
    PRINT_INFO("Finished filtering output cloud, generating pcd file.");
    if (!output_cloud->empty()) {
      if (compress) {
        pcl::io::savePCDFileBinaryCompressed(filename, *output_cloud);
      } else {
        pcl::io::savePCDFileBinary(filename, *output_cloud);
      }
    } else {
      PRINT_WARNING("Cloud is empty. Do not output to file.");
    }
  }

  // Setters for the inner parameters
  inline void SetHitProbability(float hit_prob) {
    settings_.hit_prob = Clamp(hit_prob, min_hit_prob, max_prob);
  }
  inline void SetMissProbability(float miss_prob) {
    settings_.miss_prob = Clamp(miss_prob, min_prob, max_miss_prob);
  }
  inline void SetLowResolution(const float& res) {
    settings_.low_resolution = res;
  }
  inline void SetHighResolution(const float& res) {
    settings_.high_resolution = res;
  }
  inline void SetOffsetZ(const float& offset) { settings_.z_offset = offset; }

  // Getter for the inner parameters
  inline float GetHitProbability() const { return settings_.hit_prob; }
  inline float GetMissProbability() const { return settings_.miss_prob; }

  inline float ProbabilityToOdd(float prob) {
    return std::log(prob / (1. - prob));
  }
  inline float OddToProbability(float odd) {
    return 1. - 1. / (1. + std::exp(odd));
  }

  inline void Initialise(const MrvmSettings& settings) {
    settings_ = settings;
    CHECK_GT(settings_.max_point_num_in_cell, 0);
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  struct HighResolutionVoxel {
    HighResolutionVoxel()
        : probability(kUnknown), need_update(kTrue), max_intensity(0) {}
    Probability probability;
    AtomicBool need_update;
    AtomicInt max_intensity;
    PointVector points;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  VoxelMap<HighResolutionVoxel> high_resolution_voxels_;
  MrvmSettings settings_;

  float odds_table_[kTableSize];
};

}  // namespace static_map

#endif  // BUILDER_MULTI_RESOLUTION_VOXEL_MAP_H_
