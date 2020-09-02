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

#ifndef BUILDER_MAP_BUILDER_H_
#define BUILDER_MAP_BUILDER_H_

// gtsam
// used for imu preintergration
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// stl
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// local headers
#include <boost/optional.hpp>
#include "back_end/options.h"
#include "builder/map_package.h"
#include "builder/multi_resolution_voxel_map.h"
#include "builder/pose_extrapolator.h"
#include "builder/sensor_fusions/imu_gps_tracker.h"
#include "builder/trajectory.h"
#include "pre_processors/filter_factory.h"
#include "registrators/interface.h"

namespace static_map {

// forward declarations
template <typename PointT>
class DataCollector;

namespace back_end {
template <typename PointT>
class IsamOptimizer;
}

namespace front_end {

struct Options {
  static_map::registrator::MatcherOptions scan_matcher_options;

  // for imu
  struct {
    bool enabled = true;
    sensors::ImuType type = sensors::ImuType::kNormalImu;
    float frequency = 0.f;
    float gravity_constant = 9.8f;
  } imu_options;

  struct {
    float translation_range = 0.35;
    float angle_range = 1.5;
    float time_range = 0.;
  } motion_filter;

  int accumulate_cloud_num = 1;

  struct {
    bool enable = true;
    bool use_average = true;
  } motion_compensation_options;
};

}  // namespace front_end

enum OdomCalibrationMode { kNoCalib, kOnlineCalib, kOfflineCalib };

struct MapBuilderOptions {
  struct WholeOptions {
    std::string export_file_path = "./";
    std::string map_package_path = "./";
    OdomCalibrationMode odom_calib_mode = kOnlineCalib;
  } whole_options;

  front_end::Options front_end_options;
  back_end::Options back_end_options;
  MrvmSettings output_mrvm_settings;
  MapPackageOptions map_package_options;
};

/*
 * @class MapBuilder
 * @brief Main class for single map building
 */
class MapBuilder {
 public:
  MapBuilder();
  ~MapBuilder();

  MapBuilder(const MapBuilder&) = delete;
  MapBuilder& operator=(const MapBuilder&) = delete;

  // type define
  // point and cloud definitions
  using PointType = pcl::PointXYZI;
  using PointCloudType = pcl::PointCloud<PointType>;
  using PointCloudPtr = PointCloudType::Ptr;
  using PointCloudConstPtr = PointCloudType::ConstPtr;
  // call back function for ROS
  using ShowMapFunction = std::function<void(const PointCloudPtr&)>;
  using ShowSubmapFunction = ShowMapFunction;
  using ShowPathFunction =
      std::function<void(const std::vector<Eigen::Matrix4d>&)>;

  using Ptr = std::shared_ptr<MapBuilder>;
  using ConstPtr = std::shared_ptr<const MapBuilder>;

  // functions
  /// @brief initialise the mapbuilder with a config file (xml)
  MapBuilderOptions& Initialise(const char* config_file_name);
  /// @brief finish all remaining computations for imu and pointcloud
  void FinishAllComputations();
  /// @brief set a callback function when the map updated
  void SetShowMapFunction(const ShowMapFunction& func);
  /// @brief set a callback function when the map updated
  void SetShowSubmapFunction(const ShowMapFunction& func);
  /// @brief set a callback function whem the pose updated
  void SetShowPathFunction(const ShowPathFunction& func);
  /// @brief get pointcloud and insert it into the inner container
  void InsertPointcloudMsg(const PointCloudPtr& point_cloud);
  /// @brief get imu msg from sensor and insert it into the inner container
  void InsertImuMsg(const sensors::ImuMsg::Ptr& imu_msg);
  /// @brief get odom msg from sensor and insert it into the inner container
  void InsertOdomMsg(const sensors::OdomMsg::Ptr& odom_msg);
  /// @brief get gps msg from sensor and insert it into the inner container
  void InsertGpsMsg(const sensors::NavSatFixMsg::Ptr& gps_msg);
  /// @brief if enable odom, will get the scan matching guess from odom
  void EnableUsingOdom(bool flag);
  /// @brief if enable gps, will calculate the transform from map to gps (enu)
  void EnableUsingGps(bool flag);
  /// @brief set static tf link from tracking frame to imu
  void SetTrackingToImu(const Eigen::Matrix4d& t);
  /// @brief set static tf link from tracking frame to odometry
  void SetTrackingToOdom(const Eigen::Matrix4d& t);
  /// @brief set static tf link from tracking frame to lidar
  void SetTrackingToLidar(const Eigen::Matrix4d& t);
  /// @brief set static tf link from tracking frame to gps
  void SetTrackingToGps(const Eigen::Matrix4d& t);

  /// @todo(edward) remove these two functions, using tracking frame instead
  /// @brief set static tf link from odom to lidar(cloud frame)
  void SetTransformOdomToLidar(const Eigen::Matrix4d& t);
  /// @brief set static tf link from imu to lidar(cloud frame)
  void SetTransformImuToLidar(const Eigen::Matrix4d& t);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  /// @brief inner initialise the mapbuilder with options for both
  /// front and and back end
  int InitialiseInside();
  /// @brief add a new trajectory
  /// when build a new map or load a exsiting map
  void AddNewTrajectory();
  /// @brief thread for scan to scan matching
  void ScanMatchProcessing();
  /// @brief it ia a new frame in current submap, with input cloud and pose
  void InsertFrameForSubmap(const PointCloudPtr& cloud_ptr,
                            const Eigen::Matrix4d& global_pose,
                            const double match_score);
  /// @brief thread for all operations on submaps
  void SubmapProcessing();
  /// @brief lifelong thread for connecting all submap in back-end
  void ConnectAllSubmap();
  /// @brief match 2 specified submaps
  void SubmapPairMatch(const int source_index, const int target_index);
  /// @brief do offline calibration between odom and lidar
  /// after finishing the optimization of whole map
  // void OfflineCalibrationOdomToLidar();
  /// @brief after getting the map path, calcualate
  /// the transform from map to enu, and set the rotation into submap pose
  void CalculateCoordTransformToGps();
  /// @brief output the path into .pcd and .csv file
  /// @notice the path has been rotated into gps (enu) system
  void OutputPath();
  /// @brief save the info about all submaps
  void GenerateMapPackage(const std::string& filename);
  /// @brief save the map into pieces if enabled
  void SaveMapPackage();

 private:
  common::Mutex mutex_;
  // ********************* data & options *********************
  std::unique_ptr<DataCollector<PointType>> data_collector_;
  MapBuilderOptions options_;
  /// @notice if you want to access some xml_node later in the process,
  /// the doc (xml tree) must be still in the memory
  /// so, make it a member
  pugi::xml_document options_xml_doc_;
  // odoms
  std::vector<sensors::OdomMsg::Ptr> odom_msgs_;
  sensors::OdomMsg init_odom_msg_;
  // we assume that there is only one lidar
  // even if we have several lidars, we still use the fused cloud only
  Eigen::Matrix4d transform_odom_lidar_;
  Eigen::Matrix4d transform_imu_lidar_;
  Eigen::Matrix4d tracking_to_imu_;
  Eigen::Matrix4d tracking_to_odom_;
  Eigen::Matrix4d tracking_to_lidar_;
  // can not use the inversion of the transform
  Eigen::Matrix4d tracking_to_gps_;
  bool use_imu_;
  bool use_odom_;
  bool use_gps_;
  std::atomic<bool> end_all_thread_;
  std::atomic<bool> submap_processing_done_;

  // ********************* pre processors *********************
  pre_processers::filter::Factory<PointType> filter_factory_;
  // frond end
  std::unique_ptr<PoseExtrapolator> extrapolator_ = nullptr;
  std::shared_ptr<registrator::Interface<PointType>> scan_matcher_ = nullptr;
  std::unique_ptr<std::thread> scan_match_thread_;
  std::vector<std::shared_ptr<Frame<PointType>>> frames_;
  std::atomic<bool> scan_match_thread_running_;
  bool got_first_point_cloud_ = false;

  // ************************ back end ************************
  // submaps
  std::unique_ptr<std::thread> submap_thread_;
  std::unique_ptr<back_end::IsamOptimizer<PointType>> isam_optimizer_;

  // show the result in RVIZ(ros) or other platform
  ShowMapFunction show_map_function_;
  ShowSubmapFunction show_submap_function_;
  ShowPathFunction show_path_function_;

  // trajectories
  std::vector<Trajectory<PointType>::Ptr> trajectories_;
  Trajectory<PointType>::Ptr current_trajectory_;
};

}  // namespace static_map

#endif  // BUILDER_MAP_BUILDER_H_
