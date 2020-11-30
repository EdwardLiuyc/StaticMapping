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

#include "common/octree.h"

#include <algorithm>
#include <deque>
#include <iostream>

#include "glog/logging.h"

namespace static_map {
namespace common {

OctreeNode::OctreeNode(const CubeBoundingBox& bbox)
    : bbox_(bbox), is_leaf_(false) {}

Octree::Octree(const OctreeConfig& config) : config_(config), root_(nullptr) {}

std::shared_ptr<OctreeNode> Octree::BuildTree(
    const int32_t depth, const CubeBoundingBox& bbox,
    const std::vector<OctreePoint>& points) {
  auto current_node = std::make_shared<OctreeNode>(bbox);
  current_node->depth_ = depth;

  // No need to go deeper.
  if (depth == config_.max_depth ||
      points.size() <= config_.max_num_points_in_voxel) {
    current_node->is_leaf_ = true;
    current_node->points_ = points;

    max_depth_ = std::max(max_depth_, depth);
    return current_node;
  }

  const auto sub_bboxes = bbox.GenerateSubBoxes();
  std::vector<OctreePoint> sub_points[8];
  for (const auto& point : points) {
    const int index = bbox.GetSubBoxIndexForPoint(point);
    if (index < 0) {
      continue;
    }
    CHECK_LT(index, 8);
    sub_points[index].push_back(point);
  }

  current_node->children_.resize(8);
  for (int i = 0; i < 8; ++i) {
    current_node->children_[i] =
        BuildTree(depth + 1, sub_bboxes[i], sub_points[i]);
  }
  return current_node;
}

void Octree::InitWithPointCloud(const Eigen::MatrixXd& points) {
  CHECK_GE(points.rows(), 3);

  const int size = points.cols();
  std::vector<OctreePoint> points_vec(size);
  for (int i = 0; i < size; ++i) {
    points_vec[i] = points.col(i).topRows(3);
  }
  root_ = BuildTree(0, CubeBoundingBox(config_.center, config_.max_size),
                    points_vec);
}

void Octree::RecursiveOutput() {
  std::deque<std::shared_ptr<OctreeNode>> queue;
  queue.emplace_back(root_);

  while (!queue.empty()) {
    const auto node = queue.front();
    queue.pop_front();

    if (node->is_leaf_) {
      for (const auto& point : node->points_) {
        std::cout << point.transpose() << ": " << node->depth_ << std::endl;
      }
      continue;
    }
    for (auto& node_ptr : node->children_) {
      queue.push_back(node_ptr);
    }
  }
}

}  // namespace common
}  // namespace static_map
