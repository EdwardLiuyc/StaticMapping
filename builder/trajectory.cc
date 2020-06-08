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

#include <utility>
// local
#include "builder/submap.h"
#include "builder/trajectory.h"
#include "common/pugixml.hpp"

namespace static_map {

template <typename PointT>
void Trajectory<PointT>::ToXmlNode(pugi::xml_node* map_node) {
  pugi::xml_node t_node = map_node->append_child("Trajectory");
  t_node.append_attribute("id") = id_;
  t_node.append_attribute("enu_x") = enu_offset_x_;
  t_node.append_attribute("enu_y") = enu_offset_y_;
  for (auto& submap : submaps_) {
    pugi::xml_node submap_node = t_node.append_child("Submap");
    submap_node.append_attribute("id") = submap->GetId().submap_index;
    submap_node.append_attribute("file") = submap->SavedFileName().c_str();
    const Eigen::VectorXd pose_6d = submap->GlobalPoseIn6Dof();
    submap_node.append_attribute("tx") = pose_6d[0];
    submap_node.append_attribute("ty") = pose_6d[1];
    submap_node.append_attribute("tz") = pose_6d[2];
    submap_node.append_attribute("rx") = pose_6d[3];
    submap_node.append_attribute("ry") = pose_6d[4];
    submap_node.append_attribute("rz") = pose_6d[5];

    std::string connect_pairs;
    for (auto& connected : submap->GetConnected()) {
      std::string single_pair =
          "[" + std::to_string(connected.trajectory_index) + "," +
          std::to_string(connected.submap_index) + "]";
      connect_pairs += single_pair;
    }
    submap_node.append_attribute("connections") = connect_pairs.c_str();
  }
}

template <typename PointT>
void Trajectory<PointT>::SetEnuOffset(const double x, const double y) {
  enu_offset_x_ = x;
  enu_offset_y_ = y;
}

template <typename PointT>
void Trajectory<PointT>::SetSavePath(const std::string& path) {
  for (auto& submap : submaps_) {
    submap->SetSavePath(path);
  }
}

template class Trajectory<pcl::PointXYZI>;

}  // namespace static_map
