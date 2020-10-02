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

#include "builder/map_package.h"

#include <string>
#include "Eigen/Eigen"

#include "builder/multi_resolution_voxel_map.h"
#include "builder/submap.h"
#include "builder/trajectory.h"
#include "pugixml/pugixml.hpp"

namespace static_map {

template <typename PointT>
struct SeperatedPart {
  SeperatedPart()
      : center(Eigen::Vector2d::Zero()),
        bb_min(Eigen::Vector2d::Zero()),
        bb_max(Eigen::Vector2d::Zero()),
        cloud(new pcl::PointCloud<PointT>) {}
  Eigen::Vector2d center;
  Eigen::Vector2d bb_min;
  Eigen::Vector2d bb_max;
  std::vector<std::shared_ptr<Submap<PointT>>> inside_submaps;
  typename pcl::PointCloud<PointT>::Ptr cloud;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template <typename PointT>
bool SaveTrajectoriesAsMapPackage(
    const std::vector<std::shared_ptr<Trajectory<PointT>>> trajectores,
    const MapPackageOptions& map_package_options,
    const MrvmSettings& mrvm_options, const std::string& export_path) {
  using PointCloudType = pcl::PointCloud<PointT>;
  using PointCloudPtr = typename pcl::PointCloud<PointT>::Ptr;

  // step1. calculate the bbox
  double min_x = 1.e50;
  double max_x = -1.e50;
  double min_y = 1.e50;
  double max_y = -1.e50;
  for (auto& single_trajectory : trajectores) {
    for (auto& submap : *single_trajectory) {
      const Eigen::Vector3d position = submap->GlobalTranslation();
      if (position[0] > max_x) {
        max_x = position[0];
      }
      if (position[0] < min_x) {
        min_x = position[0];
      }

      if (position[1] > max_y) {
        max_y = position[1];
      }
      if (position[1] < min_y) {
        min_y = position[1];
      }
    }
  }

  min_x -= map_package_options.border_offset;
  min_y -= map_package_options.border_offset;
  max_x += map_package_options.border_offset;
  max_y += map_package_options.border_offset;

  // step2. seperate the map
  const double half_width = map_package_options.piece_width * 0.5;
  int x_steps = (max_x - min_x) / half_width;
  int y_steps = (max_y - min_y) / half_width;
  if (x_steps < 0 || y_steps < 0) {
    PRINT_WARNING("No good bounding box, save no map package.");
    return false;
  }

  PRINT_DEBUG_FMT("Map package bbox: \n  (%lf, %lf)\n  (%lf, %lf)", min_x,
                  min_y, max_x, max_y);

  // situation that the map is too small compared to the piece width
  // we should still add the whole map into the only piece
  // so ++ to the steps
  if (x_steps == 0) {
    x_steps++;
  }
  if (y_steps == 0) {
    y_steps++;
  }

  auto inside_bbox = [](const Eigen::Vector3d& point,
                        const Eigen::Vector2d& bbox_min,
                        const Eigen::Vector2d& bbox_max) -> bool {
    return (point[0] >= bbox_min[0] && point[0] <= bbox_max[0] &&
            point[1] >= bbox_min[1] && point[1] <= bbox_max[1]);
  };
  const Eigen::Vector2d part_offset(map_package_options.border_offset,
                                    map_package_options.border_offset);
  SeperatedPart<PointT> parts[x_steps][y_steps];
  for (int x = 0; x < x_steps; ++x) {
    for (int y = 0; y < y_steps; ++y) {
      auto& part = parts[x][y];
      part.center << min_x + (x + 1) * half_width, min_y + (y + 1) * half_width;
      part.bb_min = part.center - Eigen::Vector2d(half_width, half_width);
      part.bb_max = part.center + Eigen::Vector2d(half_width, half_width);
      part.bb_min[0] = common::Clamp(part.bb_min[0], min_x, max_x);
      part.bb_max[0] = common::Clamp(part.bb_max[0], min_x, max_x);
      part.bb_min[1] = common::Clamp(part.bb_min[1], min_y, max_y);
      part.bb_max[1] = common::Clamp(part.bb_max[1], min_y, max_y);

      Eigen::Vector2d offseted_bb_min = part.bb_min - part_offset;
      Eigen::Vector2d offseted_bb_max = part.bb_max + part_offset;
      for (auto& trajectory : trajectores) {
        for (auto& submap : *trajectory) {
          const Eigen::Vector3d position = submap->GlobalTranslation();
          if (inside_bbox(position, offseted_bb_min, offseted_bb_max)) {
            part.inside_submaps.push_back(submap);
          }
        }
      }
    }
  }

  // step3. join the submaps in single part together
  for (int x = 0; x < x_steps; ++x) {
    for (int y = 0; y < y_steps; ++y) {
      auto& part = parts[x][y];
      const int submaps_size = part.inside_submaps.size();
      int i = 0;
      MultiResolutionVoxelMap<PointT> voxel_map;
      voxel_map.Initialise(mrvm_options);
      for (auto& submap : part.inside_submaps) {
        PointCloudPtr transformed_cloud(new PointCloudType);
        const Eigen::Matrix4d pose = submap->GlobalPose();
        const Eigen::Vector3d translation = submap->GlobalTranslation();
        pcl::transformPointCloud(*(submap->Cloud()->GetPclCloud()),
                                 *transformed_cloud, pose);
        PRINT_DEBUG_FMT("submap in piece[%d][%d] : %d / %d", x, y, i,
                        submaps_size - 1);
        if (inside_bbox(translation, part.bb_min, part.bb_max)) {
          start_clock();
          voxel_map.InsertPointCloud(transformed_cloud,
                                     translation.cast<float>());
          end_clock(__FILE__, __FUNCTION__, __LINE__);
        } else {
          start_clock();
          PointCloudPtr transformed_cloud_in_bbox(new PointCloudType);
          for (auto& point : transformed_cloud->points) {
            if (inside_bbox(Eigen::Vector3d(point.x, point.y, point.z),
                            part.bb_min, part.bb_max)) {
              transformed_cloud_in_bbox->points.push_back(point);
            }
          }
          if (!transformed_cloud_in_bbox->empty()) {
            voxel_map.InsertPointCloud(transformed_cloud_in_bbox,
                                       translation.cast<float>());
          }
          end_clock(__FILE__, __FUNCTION__, __LINE__);
        }
        ++i;
      }

      PointCloudPtr whole_part_cloud(new PointCloudType);
      voxel_map.OutputToPointCloud(mrvm_options.prob_threshold,
                                   whole_part_cloud);

      // step4. cut the cloud in right size
      for (auto& point : whole_part_cloud->points) {
        if (inside_bbox(Eigen::Vector3d(point.x, point.y, point.z), part.bb_min,
                        part.bb_max)) {
          point.x -= part.center[0];
          point.y -= part.center[1];
          part.cloud->points.push_back(point);
        }
      }

      // output to pcd file and release the memory
      std::string filename = map_package_options.cloud_file_prefix +
                             std::to_string(x) + "_" + std::to_string(y) +
                             ".pcd";
      pcl::io::savePCDFileBinaryCompressed(export_path + filename, *part.cloud);
      part.cloud->points.clear();
      part.cloud->points.shrink_to_fit();
    }
  }

  // step5. generate description file and save cloud
  pugi::xml_document doc;
  pugi::xml_node map_package_node = doc.append_child("MapPackage");
  for (int x = 0; x < x_steps; ++x) {
    for (int y = 0; y < y_steps; ++y) {
      pugi::xml_node map_piece_node = map_package_node.append_child("Piece");
      auto& part = parts[x][y];
      map_piece_node.append_attribute("x") = part.center[0];
      map_piece_node.append_attribute("y") = part.center[1];

      std::string filename = map_package_options.cloud_file_prefix +
                             std::to_string(x) + "_" + std::to_string(y) +
                             ".pcd";
      map_piece_node.append_attribute("file") = filename.c_str();
    }
  }
  std::string filename = export_path + map_package_options.descript_filename;
  doc.save_file(filename.c_str());

  return true;
}

template bool SaveTrajectoriesAsMapPackage(
    const std::vector<std::shared_ptr<Trajectory<pcl::PointXYZI>>> trajectores,
    const MapPackageOptions& map_package_options,
    const MrvmSettings& mrvm_options, const std::string& export_path);

}  // namespace static_map
