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

// stl
#include <fstream>
#include <ostream>
// local
#include "back_end/view_graph.h"
#include "common/macro_defines.h"

#include "CImg/CImg.h"

namespace static_map {
namespace back_end {

ViewGraph::ViewGraph() {
  min_bounding_ << std::numeric_limits<double>::max(),
      std::numeric_limits<double>::max();
  max_bounding_ << std::numeric_limits<double>::min(),
      std::numeric_limits<double>::min();
}

void ViewGraph::AddEdge(const int64_t a, const int64_t b,
                        const Eigen::Matrix4d &t) {
  if (graph_map_.count(a) == 0 || graph_map_.count(b) == 0) {
    PRINT_ERROR("Cannot find the index a or b in the map.");
    return;
  }
  graph_map_[a].connections.push_back(std::make_pair(b, t));
}

void ViewGraph::AddVertex(const int64_t index, const Eigen::Matrix4d &pose) {
  graph_map_[index].self_pose = pose;
  // update bbox
  if (pose(0, 3) < min_bounding_[0]) {
    min_bounding_[0] = pose(0, 3);
  }
  if (pose(1, 3) < min_bounding_[1]) {
    min_bounding_[1] = pose(1, 3);
  }
  if (pose(0, 3) > max_bounding_[0]) {
    max_bounding_[0] = pose(0, 3);
  }
  if (pose(1, 3) > max_bounding_[1]) {
    max_bounding_[1] = pose(1, 3);
  }
}

void ViewGraph::SaveTextFile(const std::string &filename) {
  std::ofstream file(filename);
  if (file.is_open()) {
    std::string content;

    for (auto &item : graph_map_) {
      content += std::to_string(item.first);
      content += " > ";
      for (auto &connect : item.second.connections) {
        content += (std::to_string(connect.first) + ", ");
      }
      content += "\n";
    }

    file << content;
    file.close();
  } else {
    PRINT_ERROR("Failed to open file.");
  }
}

/// @brief save the graph into a png image file
void ViewGraph::SaveImage(const std::string &filename,
                          const double resolution) {
  if (graph_map_.empty()) {
    PRINT_WARNING("Graph is empty, output nothing.");
    return;
  }

  const double edge_width = 1.;
  max_bounding_[0] += edge_width;
  max_bounding_[1] += edge_width;
  min_bounding_[0] -= edge_width;
  min_bounding_[1] -= edge_width;
  Eigen::Vector2d image_bbox = max_bounding_ - min_bounding_;
  image_bbox /= resolution;
  // std::cout << image_bbox << std::endl;

  auto pose_to_image = [&](const Eigen::Matrix4d &pose) -> std::pair<int, int> {
    Eigen::Vector2d image_position(pose(0, 3), pose(1, 3));
    image_position[0] -= min_bounding_[0];
    image_position[1] = max_bounding_[1] - image_position[1];
    image_position /= resolution;
    return std::make_pair(static_cast<int>(image_position[0]),
                          static_cast<int>(image_position[1]));
  };

  cimg_library::CImg<unsigned char> image(static_cast<int>(image_bbox[0]),
                                          static_cast<int>(image_bbox[1]), 1, 3,
                                          255);

  unsigned char red[] = {255, 0, 0};
  unsigned char blue[] = {0, 0, 255};
  for (auto &item : graph_map_) {
    auto from_index = item.first;
    auto from_coord_in_image = pose_to_image(item.second.self_pose);
    for (auto &connect : item.second.connections) {
      auto to_index = connect.first;
      auto to_coord_in_image = pose_to_image(graph_map_[to_index].self_pose);
      if (to_index - from_index == 1) {
        image.draw_line(from_coord_in_image.first, from_coord_in_image.second,
                        to_coord_in_image.first, to_coord_in_image.second,
                        blue);
      } else {
        image.draw_line(from_coord_in_image.first, from_coord_in_image.second,
                        to_coord_in_image.first, to_coord_in_image.second, red);
      }
    }
  }

  const char *c_filename = filename.c_str();
  image.save(c_filename);
}

}  // namespace back_end
}  // namespace static_map
