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

#ifndef BUILDER_MULTI_TRAJECTORY_MAP_BUILDER_H_
#define BUILDER_MULTI_TRAJECTORY_MAP_BUILDER_H_

// stl
#include <memory>
#include <string>
#include <vector>
// local
#include "back_end/loop_detector.h"
#include "back_end/multi_trajectory_optimizer.h"

namespace static_map {

template <typename PointT>
class Trajectory;

struct MultiTrajectoryMapBuilderOptions {
  back_end::LoopDetectorSettings loop_dettect_settings;
  SubmapOptions submap_options;
};

class MultiTrajectoryMapBuilder {
 public:
  explicit MultiTrajectoryMapBuilder(
      const MultiTrajectoryMapBuilderOptions& options);
  ~MultiTrajectoryMapBuilder();

  MultiTrajectoryMapBuilder(const MultiTrajectoryMapBuilder&) = delete;
  MultiTrajectoryMapBuilder& operator=(const MultiTrajectoryMapBuilder&) =
      delete;

  // type define
  // point and cloud definitions
  using PointType = pcl::PointXYZI;
  using PointCloudType = pcl::PointCloud<PointType>;
  using PointCloudPtr = PointCloudType::Ptr;
  using PointCloudConstPtr = PointCloudType::ConstPtr;

  /// @brief initialise with a config file (xml)
  int Initialise(const char* config_file_name);

  void LoadBaseMap(const std::string& package_file);
  void LoadIncrementalMap(const std::string& package_file);
  void SaveWholeMap(const std::string& whole_map_file);
  void GenerateWholeMapPcd(const std::string& pcd_filename);
  void GenerateStaticMap(const std::string& pcd_filename);

 protected:
  int LoadTrajectoryFromFile(
      std::vector<std::shared_ptr<Trajectory<PointType>>>* trajectories,
      const std::string& file, bool update_base_utm);

  std::vector<SubmapId> ConnectionsStrToIds(const std::string& connections);

 private:
  MultiTrajectoryMapBuilderOptions options_;

  back_end::MultiTrajectoryOptimizer<PointType> optimizer_;
  std::vector<std::shared_ptr<Trajectory<PointType>>> base_trajectories_;
  std::vector<std::shared_ptr<Trajectory<PointType>>> incremental_trajectories_;

  double base_utm_x_ = 0.;
  double base_utm_y_ = 0.;
};

}  // namespace static_map

#endif  // BUILDER_MULTI_TRAJECTORY_MAP_BUILDER_H_
