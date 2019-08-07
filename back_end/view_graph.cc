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

#ifdef _USE_OPENCV_
#include <opencv2/opencv.hpp>
#endif

namespace static_map {
namespace back_end {

void ViewGraph::AddEdge(const int64_t a, const int64_t b,
                        const Eigen::Matrix4f &t) {
  if (graph_map_.find(a) == graph_map_.end() ||
      graph_map_.find(b) == graph_map_.end()) {
    PRINT_ERROR("Cannot find the index a or b in the map.");
    return;
  }
  graph_map_[a].connections.push_back(std::make_pair(b, t));
}

void ViewGraph::AddVertex(const int64_t index, const Eigen::Matrix4f &pose) {
  if (graph_map_.find(index) == graph_map_.end()) {
    graph_map_.emplace(index, GraphItem());
  }
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
#ifndef _USE_OPENCV_
  PRINT_WARNING("Option: Do not use OpenCV, so not able to save to image.");
  return;
#else
  const double edge_width = 1.;
  max_bounding_[0] += edge_width;
  max_bounding_[1] += edge_width;
  min_bounding_[0] -= edge_width;
  min_bounding_[1] -= edge_width;
  Eigen::Vector2d image_bbox = max_bounding_ - min_bounding_;
  image_bbox /= resolution;
  // std::cout << image_bbox << std::endl;

  auto pose_to_image = [&](const Eigen::Matrix4f &pose) -> std::pair<int, int> {
    Eigen::Vector2d image_position(pose(0, 3), pose(1, 3));
    image_position[0] -= min_bounding_[0];
    image_position[1] = max_bounding_[1] - image_position[1];
    image_position /= resolution;
    return std::make_pair(static_cast<int>(image_position[0]),
                          static_cast<int>(image_position[1]));
  };

  cv::Mat image(static_cast<int>(image_bbox[1]),
                static_cast<int>(image_bbox[0]), CV_8UC3,
                cv::Scalar(255, 255, 255));
  if (image.empty()) {
    PRINT_ERROR("image is empty.");
    return;
  }

  for (auto &item : graph_map_) {
    auto from_index = item.first;
    auto from_coord_in_image = pose_to_image(item.second.self_pose);
    for (auto &connect : item.second.connections) {
      auto to_index = connect.first;
      auto to_coord_in_image = pose_to_image(graph_map_[to_index].self_pose);
      if (to_index - from_index == 1) {
        cv::line(
            image,
            cv::Point(from_coord_in_image.first, from_coord_in_image.second),
            cv::Point(to_coord_in_image.first, to_coord_in_image.second),
            cv::Scalar(0, 0, 255), 2, CV_AA);
      } else {
        cv::line(
            image,
            cv::Point(from_coord_in_image.first, from_coord_in_image.second),
            cv::Point(to_coord_in_image.first, to_coord_in_image.second),
            cv::Scalar(255, 0, 0), 2, CV_AA);
      }
    }
  }

  cv::imwrite(filename, image);
#endif
}

}  // namespace back_end
}  // namespace static_map
