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

#ifndef COMMON_OCTREE_H_
#define COMMON_OCTREE_H_

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include "Eigen/Eigen"
#include "common/bounding_box.h"

namespace static_map {
namespace common {

using OctreePoint = Eigen::Vector3d;

struct OctreeConfig {
  OctreePoint center;      // unit: m
  double max_size = 100.;  // unit: m
  int32_t max_depth = 15;
  int32_t max_num_points_in_voxel = 4;
};

class OctreeNode {
  friend class Octree;

 public:
  explicit OctreeNode(const CubeBoundingBox& bbox);

  const std::vector<OctreePoint>& GetPoints() const { return points_; }

 private:
  int32_t depth_;
  CubeBoundingBox bbox_;
  bool is_leaf_;
  std::vector<OctreePoint> points_;
  std::vector<std::shared_ptr<OctreeNode>> children_;
};

/// @class Octree
/// Some detailed information for this Octree class.
/// Input: a point cloud in Eigen::MatrixXd (3d by default)
///
/// Index:
///  In a common Octree, its nodes will always have 8 children which are stored
///  as pointers. In my design, I will designate them as 8 uint64_t indexes and
///  used as key of an unordered_map. Once got the index of a node, you will get
///  its parent and its children if exists.
class Octree {
 public:
  explicit Octree(const OctreeConfig& config);

  void InitWithPointCloud(const Eigen::MatrixXd& points);

  int GetMaxDepth() const { return max_depth_; }

  void RecursiveOutput();

 private:
  std::shared_ptr<OctreeNode> BuildTree(const int32_t depth,
                                        const CubeBoundingBox& bbox,
                                        const std::vector<OctreePoint>& points);

 private:
  const OctreeConfig config_;
  std::shared_ptr<OctreeNode> root_;

  int max_depth_ = 0;
};
}  // namespace common

}  // namespace static_map

#endif  // COMMON_OCTREE_H_
