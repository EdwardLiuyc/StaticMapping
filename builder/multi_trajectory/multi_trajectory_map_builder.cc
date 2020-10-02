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
#include "builder/multi_trajectory_map_builder.h"
#include "builder/multi_resolution_voxel_map.h"
#include "builder/trajectory.h"
#include "common/file_utils.h"
#include "common/math.h"
#include "pugixml/pugixml.hpp"

namespace static_map {

// refer to
// http://www.martinbroadhurst.com/how-to-split-a-string-in-c.html
template <class Container>
void split(const std::string& str, Container* cont, char delim = ' ') {
  std::stringstream ss(str);
  std::string token;
  while (std::getline(ss, token, delim)) {
    cont->push_back(token);
  }
}

MultiTrajectoryMapBuilder::MultiTrajectoryMapBuilder(
    const MultiTrajectoryMapBuilderOptions& options)
    : options_(options), optimizer_(options.loop_dettect_settings) {}

MultiTrajectoryMapBuilder::~MultiTrajectoryMapBuilder() {}

void MultiTrajectoryMapBuilder::LoadBaseMap(const std::string& package_file) {
  // step1 load file to trajectories
  PRINT_INFO("Loading base map file ... ");
  CHECK_GE(LoadTrajectoryFromFile(&base_trajectories_, package_file, true), 0);

  // step2 insert submaps and connections into back-end
  PRINT_DEBUG("Add submaps ... ");
  for (auto& trajectory : base_trajectories_) {
    for (auto& submap : *trajectory) {
      optimizer_.AddSubmap(submap, false);
    }
  }

  // step3 add connections built when mapping
  PRINT_DEBUG("Add connections without loop detection ... ");
  for (auto& trajectory : base_trajectories_) {
    for (auto& submap : *trajectory) {
      auto target_id = submap->GetId();
      auto connections = submap->GetConnected();
      Eigen::Matrix4f target_pose = submap->GlobalPose();
      for (auto& connected_id : connections) {
        auto source_submap =
            base_trajectories_[connected_id.trajectory_index]->at(
                connected_id.submap_index);
        auto source_id = source_submap->GetId();
        Eigen::Matrix4f source_pose = source_submap->GlobalPose();
        optimizer_.AddSubmapConnection(
            target_id, source_id, target_pose.inverse() * source_pose, false);
      }
    }
  }

  optimizer_.UseCurrentStateAsBaseMap();
}

void MultiTrajectoryMapBuilder::LoadIncrementalMap(
    const std::string& package_file) {
  // step1 load file to trajectories
  PRINT_INFO("Loading incremental map file ... ");
  CHECK_GE(
      LoadTrajectoryFromFile(&incremental_trajectories_, package_file, false),
      0);

  // step1.1 change the trajectory_index of incremental trajectories
  const int offset = base_trajectories_.size();

  // step2 insert submaps and connections into back-end
  PRINT_DEBUG("Add submaps with loop detection ... ");
  for (auto& trajectory : incremental_trajectories_) {
    int current_trajectory_id = trajectory->GetId();
    trajectory->SetId(current_trajectory_id + offset);
    for (auto& submap : *trajectory) {
      auto submap_id = submap->GetId();
      submap_id.trajectory_index += offset;
      submap->SetId(submap_id);
      optimizer_.AddSubmap(submap, true);
      // common::PrintTransform(submap->GlobalPose());
    }
  }

  // step3
  PRINT_DEBUG("Add connections in incremental trajectories ... ");
  for (auto& trajectory : incremental_trajectories_) {
    for (auto& submap : *trajectory) {
      auto target_id = submap->GetId();
      auto& connections = submap->GetConnected();
      Eigen::Matrix4f target_pose = submap->GlobalPose();
      for (auto& connected_id : connections) {
        auto source_submap =
            incremental_trajectories_[connected_id.trajectory_index]->at(
                connected_id.submap_index);
        connected_id = source_submap->GetId();
        Eigen::Matrix4f source_pose = source_submap->GlobalPose();
        optimizer_.AddSubmapConnection(target_id, connected_id,
                                       target_pose.inverse() * source_pose,
                                       false);
      }
    }
  }

  // step3.1 optimizing
  optimizer_.RunFinalOptimizing();

  // step4 add all incremental trajectories into basemap
  base_trajectories_.insert(base_trajectories_.end(),
                            incremental_trajectories_.begin(),
                            incremental_trajectories_.end());
  incremental_trajectories_.clear();
}

int MultiTrajectoryMapBuilder::LoadTrajectoryFromFile(
    std::vector<std::shared_ptr<Trajectory<PointType>>>* trajectories,
    const std::string& file, bool update_base_utm) {
  CHECK(trajectories);
  // step0, test for existence of the file
  if (!common::FileExist(file)) {
    PRINT_ERROR_FMT("base map file does not exist ... %s", file.c_str());
    return -1;
  }
  // step1, load the file
  pugi::xml_document doc;
  if (!doc.load_file(file.c_str())) {
    PRINT_ERROR("Failed to load file.");
    return -1;
  }
  pugi::xml_node map_node = doc.child("Map");
  if (!map_node) {
    PRINT_ERROR("Format error.");
    return -1;
  }
  // step1.1 get the path of pkg.xml (for submaps)
  std::string pkg_file_path = common::FilePath(file);
  for (auto t_node = map_node.child("Trajectory"); t_node;
       t_node = t_node.next_sibling("Trajectory")) {
    const int current_trajectory_index = trajectories->size();
    trajectories->emplace_back(new Trajectory<PointType>);
    auto& current_trajectory = trajectories->back();
    // trajectory attr
    if (current_trajectory_index == 0 && update_base_utm) {
      base_utm_x_ = t_node.attribute("utm_x").as_double();
      base_utm_y_ = t_node.attribute("utm_y").as_double();
    }
    double utm_offset_x = t_node.attribute("utm_x").as_double() - base_utm_x_;
    double utm_offset_y = t_node.attribute("utm_y").as_double() - base_utm_y_;
    current_trajectory->SetId(current_trajectory_index);
    // insert submaps
    for (auto s_node = t_node.child("Submap"); s_node;
         s_node = s_node.next_sibling("Submap")) {
      auto submap =
          std::make_shared<Submap<PointType>>(options_.submap_options);
      // id
      SubmapId submap_id;
      submap_id.trajectory_index = current_trajectory_index;
      submap_id.submap_index = s_node.attribute("id").as_int();
      submap->SetId(submap_id);
      // pose
      Eigen::Vector6<double> pose_6d;
      pose_6d[0] = s_node.attribute("tx").as_double() + utm_offset_x;
      pose_6d[1] = s_node.attribute("ty").as_double() + utm_offset_y;
      pose_6d[2] = s_node.attribute("tz").as_double();
      pose_6d[3] = s_node.attribute("rx").as_double();
      pose_6d[4] = s_node.attribute("ry").as_double();
      pose_6d[5] = s_node.attribute("rz").as_double();
      Eigen::Matrix4d pose = common::Vector6ToTransform(pose_6d);
      submap->SetGlobalPose(pose);
      // connections
      std::string connections = s_node.attribute("connections").as_string();
      auto connected_ids = ConnectionsStrToIds(connections);
      for (auto& id : connected_ids) {
        submap->AddConnectedSubmap(id);
      }
      // file
      std::string file = s_node.attribute("file").as_string();
      CHECK(common::FileExist(pkg_file_path + file));
      submap->SetSavePath(pkg_file_path);
      submap->SetSavedFileName(file);
      // finished
      current_trajectory->push_back(submap);
    }
  }

  return 0;
}

void MultiTrajectoryMapBuilder::SaveWholeMap(const std::string& package_file) {
  std::string pkg_path = common::FilePath(package_file);
  PRINT_INFO("Save whole map package ... ");
  CHECK(incremental_trajectories_.empty());

  pugi::xml_document doc;
  pugi::xml_node map_node = doc.append_child("Map");
  for (auto& trajectory : base_trajectories_) {
    // trajectory->SetSavePath(pkg_path);
    trajectory->SetEnuOffset(base_utm_x_, base_utm_y_);
    trajectory->ToXmlNode(&map_node);
  }

  doc.save_file(package_file.c_str());
}

void MultiTrajectoryMapBuilder::GenerateWholeMapPcd(
    const std::string& pcd_filename) {
  PRINT_INFO("Save Whole Map to pcd...");
  CHECK(incremental_trajectories_.empty());
  pcl::PointCloud<PointType> whole_map;
  for (auto& trajectory : base_trajectories_) {
    for (auto& submap : *trajectory) {
      Eigen::Matrix4f pose = submap->GlobalPose();
      pcl::PointCloud<PointType> transformed_cloud;
      pcl::transformPointCloud(*submap->Cloud(), transformed_cloud, pose);
      whole_map += transformed_cloud;
    }
  }

  if (!whole_map.empty()) {
    pcl::io::savePCDFileBinaryCompressed(pcd_filename, whole_map);
  }
}

void MultiTrajectoryMapBuilder::GenerateStaticMap(
    const std::string& pcd_filename) {
  PRINT_INFO("Save Whole Map to static_map(pcd file)...");
  CHECK(incremental_trajectories_.empty());
  MultiResolutionVoxelMap<PointType> map;
  MrvmSettings settings;
  settings.z_offset = 1.2;
  settings.max_point_num_in_cell = 4;
  map.Initialise(settings);
  for (auto& trajectory : base_trajectories_) {
    for (auto& submap : *trajectory) {
      Eigen::Matrix4f pose = submap->GlobalPose();
      pcl::PointCloud<PointType>::Ptr transformed_cloud(
          new pcl::PointCloud<PointType>);
      pcl::transformPointCloud(*submap->Cloud(), *transformed_cloud, pose);
      PRINT_DEBUG_FMT("submap index in trajectory %d : %d / %d",
                      trajectory->GetId(), submap->GetId().submap_index,
                      static_cast<int>(trajectory->size()) - 1);
      start_clock();
      map.InsertPointCloud(transformed_cloud, submap->GlobalTranslation());
      submap->ClearCloud();
      end_clock(__FILE__, __FUNCTION__, __LINE__);
    }
  }
  PRINT_INFO("All trajectories inserted...");
  map.OutputToPointCloud(0.57, pcd_filename, false);
}

std::vector<SubmapId> MultiTrajectoryMapBuilder::ConnectionsStrToIds(
    const std::string& connections) {
  // the string must be like this
  // "[0,63][0,80][0,81][0,82]"
  if (connections.empty()) {
    return std::vector<SubmapId>();
  }
  std::vector<SubmapId> ids;
  std::vector<std::string> splited_strs;
  split(connections, &splited_strs, ']');
  for (auto& str : splited_strs) {
    auto first_int_pos = str.find('[') + 1;
    auto comma_pos = str.find(',');
    CHECK(first_int_pos <= comma_pos);

    SubmapId id;
    id.trajectory_index =
        atoi(str.substr(first_int_pos, comma_pos - first_int_pos).c_str());
    id.submap_index = atoi(str.substr(comma_pos + 1).c_str());
    ids.push_back(id);
  }
  return ids;
}

}  // namespace static_map
