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
// 前向生命
template <typename PointT>
class DataCollector;

namespace back_end {
template <typename PointT>
class IsamOptimizer;
}

namespace front_end {

// 配置
struct Options {
  static_map::registrator::MatcherOptions scan_matcher_options;

  // for imu
  struct {
    bool enabled = true;
    sensors::ImuType type = sensors::ImuType::kNormalImu;
    float frequency = 0.f;
    float gravity_constant = 9.8f;
  } imu_options;

  //!@brief 这里是判断车是否有运动, 如果没有运动的话就暂时不动
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

// MapBuilderOptions配置
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

 /* **********************************
 * @class 建图算法->单独的地图构建
 ** **********************************/

class MapBuilder {
 public:
  // 类初始化
  MapBuilder();
  ~MapBuilder();

  // 禁止拷贝控制
  MapBuilder(const MapBuilder&) = delete;
  MapBuilder& operator=(const MapBuilder&) = delete;

  // type define
  // point and cloud definitions
  // 点和点云的定义
  using PointType = pcl::PointXYZI;
  using PointCloudType = pcl::PointCloud<PointType>;
  using PointCloudPtr = PointCloudType::Ptr;
  using PointCloudConstPtr = PointCloudType::ConstPtr;
  // call back function for ROS
  // 用于ROS的回调函数
  using ShowMapFunction = std::function<void(const PointCloudPtr&)>;
  using ShowSubmapFunction = ShowMapFunction;
  using ShowPathFunction =
      std::function<void(const std::vector<Eigen::Matrix4d>&)>;
  // 类指针定义:Ptr+ConstPtr
  using Ptr = std::shared_ptr<MapBuilder>;
  using ConstPtr = std::shared_ptr<const MapBuilder>;

  // functions
  /// @brief initialise the mapbuilder with a config file (xml)
  //  @brief 用.xml文件来初始化mapBuilder类
  MapBuilderOptions& Initialise(const char* config_file_name);
  /// @brief finish all remaining computations for imu and pointcloud
  //  @brief 对于IMU和点云的结束所有的计算
  void FinishAllComputations();
  /// @brief set a callback function when the map updated
  //  @brief 对地图展示功能设置
  void SetShowMapFunction(const ShowMapFunction& func);
  /// @brief set a callback function when the map updated
  //  @brief 设置回调函数来确保地图更新的时候触发
  void SetShowSubmapFunction(const ShowMapFunction& func);
  /// @brief set a callback function whem the pose updated
  // !@brief 设置回调函数当位姿发生变化的时候触发
  void SetShowPathFunction(const ShowPathFunction& func);
  /// @brief get pointcloud and insert it into the inner container
  // !@brief 获取点云信息然后插入内置容器中
  void InsertPointcloudMsg(const PointCloudPtr& point_cloud);
  /// @brief get imu msg from sensor and insert it into the inner container
  // !@brief 从传感器中获取IMU信息然后插入到内置容器中
  void InsertImuMsg(const sensors::ImuMsg::Ptr& imu_msg);
  /// @brief get odom msg from sensor and insert it into the inner container
  // !@brief 从传感器中获取轮速计数据，然后插入到内置容器中
  void InsertOdomMsg(const sensors::OdomMsg::Ptr& odom_msg);
  /// @brief get gps msg from sensor and insert it into the inner container
  // !@brief 从传感器中获取GPS数据，然后插入到内置容器中
  void InsertGpsMsg(const sensors::NavSatFixMsg::Ptr& gps_msg);
  /// @brief if enable odom, will get the scan matching guess from odom
  // !@brief 如果使用轮速计，将会从轮速计中获取扫描的猜测信息
  void EnableUsingOdom(bool flag);
  /// @brief if enable gps, will calculate the transform from map to gps (enu)
  // !@brief 如果使用GPS功能，会把地图转换到ENU坐标系下
  void EnableUsingGps(bool flag);
  /// @brief set static tf link from tracking frame to imu
  // !@brief 设置静态tf链接（这个看tf状态）
  void SetTrackingToImu(const Eigen::Matrix4d& t);
  /// @brief set static tf link from tracking frame to odometry
  // !@brief 用轮速计来计算每帧图像的更新
  void SetTrackingToOdom(const Eigen::Matrix4d& t);
  /// @brief set static tf link from tracking frame to lidar
  // !@brief 用雷达来追踪点云帧
  void SetTrackingToLidar(const Eigen::Matrix4d& t);
  /// @brief set static tf link from tracking frame to gps
  // !@brief 用GPS来追踪点云帧
  void SetTrackingToGps(const Eigen::Matrix4d& t);

  // TODO: 移除这两个功能
  /// @todo(edward) remove these two functions, using tracking frame instead
  /// @brief set static tf link from odom to lidar(cloud frame)
  void SetTransformOdomToLidar(const Eigen::Matrix4d& t);
  /// @brief set static tf link from imu to lidar(cloud frame)
  void SetTransformImuToLidar(const Eigen::Matrix4d& t);

  // 这个主要是为了Eigen的内存对齐
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  /// @brief inner initialise the mapbuilder with options for both
  /// front and and back end
  // 用配置对MapBuilder的前端和后端进行初始化
  int InitialiseInside();
  /// @brief add a new trajectory
  /// when build a new map or load a exsiting map
  // 创建新的地图或加载已有地图
  void AddNewTrajectory();
  /// @brief thread for scan to scan matching
  // @brief 帧间匹配
  void ScanMatchProcessing();
  /// @brief it ia a new frame in current submap, with input cloud and pose
  // !@brief 如果在当前子图中这是一帧新点云图，则输入点云和位姿
  void InsertFrameForSubmap(const PointCloudPtr& cloud_ptr,
                            const Eigen::Matrix4d& global_pose,
                            const double match_score);
  /// @brief thread for all operations on submaps
  // !@brief 专门用于子图操作的线程
  void SubmapProcessing();
  /// @brief lifelong thread for connecting all submap in back-end
  // !@brief 用于后端的线程进行连接多个子图
  void ConnectAllSubmap();
<<<<<<< HEAD
  /// @brief life long thread for managing submaps between RAM and Disk
  // !@brief 长期存在线程里用RAM和Disk来进行内存交互
  void SubmapMemoryManaging();
=======
>>>>>>> 5698495f31021f834ee53bd4aeec9bc2df66a500
  /// @brief match 2 specified submaps
  // !@brief 匹配任意两个子图
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
  // 创建指针容器用来收集点云数据
  std::unique_ptr<DataCollector<PointType>> data_collector_;
  // MapBuilder配置
  MapBuilderOptions options_;
  /// @notice if you want to access some xml_node later in the process,
  /// the doc (xml tree) must be still in the memory
  /// so, make it a member
  pugi::xml_document options_xml_doc_;
  // odoms
  // 存储轮速计信息
  std::vector<sensors::OdomMsg::Ptr> odom_msgs_;
  sensors::OdomMsg init_odom_msg_;
  // we assume that there is only one lidar
  // even if we have several lidars, we still use the fused cloud only
  // 这里雷达是作为主传感器的
  // 轮速计到雷达的外参
  Eigen::Matrix4d transform_odom_lidar_;
  // IMU到雷达的外参
  Eigen::Matrix4d transform_imu_lidar_;
  // 基于IMU的位姿
  Eigen::Matrix4d tracking_to_imu_;
  // 基于轮速计的位姿
  Eigen::Matrix4d tracking_to_odom_;
  // 基于雷达的位姿
  Eigen::Matrix4d tracking_to_lidar_;
  // can not use the inversion of the transform
  Eigen::Matrix4d tracking_to_gps_;
  // 是否使用IMU
  bool use_imu_;
  // 是否使用Odom
  bool use_odom_;
  // 是否使用GPS
  bool use_gps_;
  // 明天查下atomic是干啥的
  std::atomic<bool> end_all_thread_;
  std::atomic<bool> submap_processing_done_;

  // 预处理
  // ********************* pre processors *********************
  // 预处理数据
  pre_processers::filter::Factory<PointType> filter_factory_;
  // frond end
  // 运动插值
  std::unique_ptr<PoseExtrapolator> extrapolator_ = nullptr;
  // 帧间匹配器
  std::shared_ptr<registrator::Interface<PointType>> scan_matcher_ = nullptr;
  // 创建帧间匹配的线程
  std::unique_ptr<std::thread> scan_match_thread_;
  // 保存帧间信息
  std::vector<std::shared_ptr<Frame<PointType>>> frames_;
  // 原子锁操作多线程->原子表示不可分割的操作,该操作只存在未开始和已完成两种状态,不存在其他状态
  std::atomic<bool> scan_match_thread_running_;
  // 判断是否是第一帧点云
  bool got_first_point_cloud_ = false;

  // 后端
  // ************************ back end ************************
  // submaps
  std::unique_ptr<std::thread> submap_thread_;
  std::unique_ptr<back_end::IsamOptimizer<PointType>> isam_optimizer_;

  // show the result in RVIZ(ros) or other platform
  // 将结果显示在RVIZ中
  ShowMapFunction show_map_function_;
  ShowSubmapFunction show_submap_function_;
  ShowPathFunction show_path_function_;

  // trajectories
  // 运动轨迹
  std::vector<Trajectory<PointType>::Ptr> trajectories_;
  Trajectory<PointType>::Ptr current_trajectory_;
};

}  // namespace static_map

#endif  // BUILDER_MAP_BUILDER_H_
