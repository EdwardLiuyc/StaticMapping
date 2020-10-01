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

#ifndef BACK_END_VIEW_GRAPH_H_
#define BACK_END_VIEW_GRAPH_H_
// third party
#include <Eigen/Eigen>
// stl
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "common/macro_defines.h"

namespace static_map {
namespace back_end {

class ViewGraph {
 public:
  ViewGraph();
  ~ViewGraph() = default;

  PROHIBIT_COPY_AND_ASSIGN(ViewGraph);

  // @todo matrix4d pitfalls
  using Connect =
      std::pair<int64_t /* index */, Eigen::Matrix4d /* Transform */>;
  struct GraphItem {
    Eigen::Matrix4d self_pose;
    std::vector<Connect> connections;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  /// @breif add a new vertex if not exist, or update the pose
  void AddVertex(const int64_t index, const Eigen::Matrix4d &pose);
  /// @brief connect a -> b with t(transform in matrix)
  void AddEdge(const int64_t a, const int64_t b, const Eigen::Matrix4d &t);
  /// @brief save the grapg into a text file
  void SaveTextFile(const std::string &filename);
  /// @brief save the graph into a png image file (using OpenCV)
  void SaveImage(const std::string &filename, const double resolution = 0.05);

  // We don't return the reference of `graph_map_` because it maybe changed when
  // we are dealing with it, so, a little sacrifice of time to copy the whole
  // graph. Maybe we will figure out a better way later. TODO(edward)
  std::map<int64_t, GraphItem> GetWholeGraph() const { return graph_map_; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  void CalcualtePixels();

 private:
  std::map<int64_t, GraphItem> graph_map_;

  // TODO(edward) Use bbox instead.
  Eigen::Vector2d min_bounding_;
  Eigen::Vector2d max_bounding_;
};

}  // namespace back_end
}  // namespace static_map

#endif  // BACK_END_VIEW_GRAPH_H_
