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
#include "common/file_utils.h"
#include "pugixml/pugixml.hpp"

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

template <typename PointT>
void Trajectory<PointT>::OutputPathToPointcloud(const std::string& path) {
  if (!common::FileExist(path)) {
    LOG(WARNING) << "Path does not exist. do not output any thing.";
    return;
  }
  int path_point_index = 0;
  std::ofstream path_text_file(path + "/path.csv");
  std::string path_content;
  bool write_to_text = path_text_file.is_open();
  pcl::PointCloud<pcl::_PointXYZI>::Ptr path_cloud(
      new pcl::PointCloud<pcl::_PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI> enu_path_cloud;
  pcl::PointCloud<pcl::PointXYZI> odom_path_cloud;
  enu_path_cloud.points.reserve(submaps_.size());
  for (const auto& submap : submaps_) {
    // enu path
    if (submap->HasGps()) {
      pcl::PointXYZI enu_point;
      const Eigen::Vector3d enu = submap->GetRelatedGpsInENU();
      enu_point.x = enu[0];
      enu_point.y = enu[1];
      enu_point.z = enu[2];
      enu_point.intensity = 0.;
      enu_path_cloud.push_back(enu_point);
    }
    // odom path
    if (submap->HasOdom()) {
      pcl::PointXYZI odom_point;
      const Eigen::Vector3d odom =
          common::Translation(submap->GetRelatedOdom());
      odom_point.x = odom[0];
      odom_point.y = odom[1];
      odom_point.z = odom[2];
      odom_point.intensity = 0.;
      odom_path_cloud.push_back(odom_point);
    }
    // map path
    for (auto& frame : submap->GetFrames()) {
      pcl::_PointXYZI path_point;
      Eigen::Matrix4d pose = frame->GlobalPose();
      Eigen::Vector6<double> global_pose_6d = common::TransformToVector6(pose);
      path_point.x = global_pose_6d[0];
      path_point.y = global_pose_6d[1];
      path_point.z = global_pose_6d[2];
      path_point.intensity = path_point_index;

      path_cloud->push_back(path_point);
      if (write_to_text) {
        path_content += (std::to_string(path_point_index) + ", ");
        path_content += (std::to_string(global_pose_6d[0]) + ", ");
        path_content += (std::to_string(global_pose_6d[1]) + ", ");
        path_content += (std::to_string(global_pose_6d[2]) + ", ");
        path_content += (std::to_string(global_pose_6d[3]) + ", ");
        path_content += (std::to_string(global_pose_6d[4]) + ", ");
        path_content += (std::to_string(global_pose_6d[5]) + " \n");
      }
      path_point_index++;
    }
  }
  if (!path_cloud->empty()) {
    pcl::io::savePCDFileBinaryCompressed(path + "/path.pcd", *path_cloud);
  }
  if (!enu_path_cloud.points.empty()) {
    pcl::io::savePCDFileBinaryCompressed(path + "/enu_path.pcd",
                                         enu_path_cloud);
  }
  if (!odom_path_cloud.points.empty()) {
    pcl::io::savePCDFileBinaryCompressed(path + "/odom_path.pcd",
                                         odom_path_cloud);
  }
  if (write_to_text) {
    path_text_file << path_content;
    path_text_file.close();
  }
}

template class Trajectory<pcl::PointXYZI>;

}  // namespace static_map
