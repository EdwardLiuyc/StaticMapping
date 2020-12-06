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
#include <atomic>
#include <map>
#include <string>
#include <type_traits>
#include <vector>
// third party
#include "builder/data/cloud_types.h"
#include "common/eigen_hash.h"
#include "common/macro_defines.h"
#include "common/math.h"
#include "glog/logging.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "tbb/atomic.h"
#include "tbb/concurrent_unordered_map.h"
#include "tbb/concurrent_vector.h"

namespace static_map {

typedef uint8_t Probability;

constexpr size_t kTableSize = (1 << (sizeof(Probability) * 8));
constexpr Probability kUnknown = (kTableSize >> 1);

constexpr int kTrue = 1;
constexpr int kFalse = 0;

struct MrvmSettings {
  bool output_average = false;
  bool output_rgb = false;
  bool use_max_intensity = true;
  float prob_threshold = 0.6f;
  float low_resolution = 1.f;  // not in use any more
  float high_resolution = 0.1f;
  float hit_prob = 0.55f;
  float miss_prob = 0.48f;
  float z_offset = 0.f;
  int max_point_num_in_cell = 10;
};

using common::Clamp;

class MultiResolutionVoxelMap {
 public:
  using KeyInt3 = Eigen::Vector3i;

  using PointVector = tbb::concurrent_vector<data::InnerPointType>;
  using IndexVector = tbb::concurrent_vector<KeyInt3>;
  // @notice use int but not bool because atomic<bool> is not supported in tbb
  // so, use kTure(int 1) instead of true
  // as long as using kFalse(int 0) instead of false
  using AtomicBool = tbb::atomic<int>;
  using AtomicInt = tbb::atomic<int>;
  template <typename T>
  using VoxelMap =
      tbb::concurrent_unordered_map<KeyInt3, T, std::hash<KeyInt3>>;

  MultiResolutionVoxelMap() {
    for (int i = 0; i < kTableSize; ++i) {
      odds_table_[i] = ProbabilityToOdd(static_cast<float>(i) / kTableSize);
    }
  }
  ~MultiResolutionVoxelMap() {}

  MultiResolutionVoxelMap(const MultiResolutionVoxelMap&) = delete;
  MultiResolutionVoxelMap& operator=(const MultiResolutionVoxelMap&) = delete;

  void Initialise(const MrvmSettings& settings);

  void InsertPointCloud(const data::InnerCloudType::Ptr& cloud,
                        const Eigen::Vector3f& origin);

  void OutputToPointCloud(const float threshold,
                          const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

  void OutputToPointCloud(const float threshold,
                          const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

  void OutputToPointCloud(const float threshold, const std::string& filename,
                          bool compress = true);

  // Setters for the inner parameters
  void SetOffsetZ(const float& offset);

  float ProbabilityToOdd(float prob);
  float OddToProbability(float odd);

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

inline float MultiResolutionVoxelMap::ProbabilityToOdd(float prob) {
  return std::log(prob / (1. - prob));
}

inline float MultiResolutionVoxelMap::OddToProbability(float odd) {
  return 1. - 1. / (1. + std::exp(odd));
}

}  // namespace static_map

#endif  // BUILDER_MULTI_RESOLUTION_VOXEL_MAP_H_
