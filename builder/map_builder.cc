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

#include <atomic>
#include <chrono>
#include <memory>
#include <utility>

// pcl
#include "pcl/filters/statistical_outlier_removal.h"
#include "unsupported/Eigen/MatrixFunctions"

// local headers
#include "back_end/isam_optimizer.h"
#include "back_end/loop_detector.h"
#include "builder/data/data_collector.h"
#include "builder/map_builder.h"
#include "builder/memory_manager.h"
#include "common/macro_defines.h"
#include "common/performance/simple_prof.h"
#include "common/simple_thread_pool.h"
#include "descriptor/m2dp.h"
#include "pre_processors/filter_random_sample.h"
#include "pugixml/pugixml.hpp"

namespace static_map {

namespace {
constexpr int kSubmapResSize = 100;
constexpr double kExpolatorMinDuration = 0.001;  // second
constexpr double kLocalMapRange = 30.;           // m
}  // namespace

MapBuilder::MapBuilder()
    : use_imu_(false),
      use_gps_(false),
      end_all_thread_(false),
      submap_processing_done_(false),
      scan_match_thread_running_(false) {}

MapBuilder::~MapBuilder() {}

int MapBuilder::InitialiseInside() {
  PRINT_INFO("Init scan matchers.");
  scan_matcher_ = registrator::CreateMatcher(
      options_.front_end_options.scan_matcher_options, true);
  CHECK(scan_matcher_) << "scan match failed to init.";

  use_imu_ = options_.front_end_options.imu_options.enabled;
  if (use_imu_) {
    CHECK_GT(options_.front_end_options.imu_options.frequency, 1.e-6);
  } else {
    extrapolator_ = PoseExtrapolator::InitialSimpleCTRV(
        SimpleTime::FromSec(kExpolatorMinDuration));
  }

  PRINT_INFO("Init isam optimizer.");
  isam_optimizer_.reset(new back_end::IsamOptimizer(
      options_.back_end_options.isam_optimizer_options,
      options_.back_end_options.loop_detector_setting));

  isam_optimizer_->SetTransformOdomToLidar(transform_odom_lidar_);
  isam_optimizer_->SetTrackingToGps(tracking_to_gps_);

  data::DataCollectorOptions data_collector_options;
  data_collector_options.accumulate_cloud_num =
      options_.front_end_options.accumulate_cloud_num;
  data_collector_ = std::make_unique<data::DataCollector>(
      data_collector_options, &filter_factory_);

#ifdef _USE_OPENVDB_
  PRINT_INFO("Init openvdb.");
  openvdb::initialize();
#endif

#ifdef _ICP_USE_CUDA_
  PRINT_INFO("Init cuda device.");
  registrator::cuda::init_cuda_device();
#endif

  AddNewTrajectory();

  PRINT_INFO("Init threads.");
  scan_match_thread_ = std::make_unique<std::thread>(
      std::bind(&MapBuilder::ScanMatchProcessing, this));
  while (!scan_match_thread_running_.load()) {
    SimpleTime::FromSec(0.01).Sleep();
  }
  submap_thread_ = std::make_unique<std::thread>(
      std::bind(&MapBuilder::SubmapProcessing, this));

  PRINT_INFO("Init finished.");
  return 0;
}

void MapBuilder::SetTransformOdomToLidar(const Eigen::Matrix4d& t) {
  transform_odom_lidar_ = t;
  PRINT_INFO("Got tf : odom -> lidar ");
  common::PrintTransform(t);
}

void MapBuilder::SetTransformImuToLidar(const Eigen::Matrix4d& t) {
  transform_imu_lidar_ = t;
  PRINT_INFO("Got tf : imu -> lidar ");
  common::PrintTransform(t);
}

void MapBuilder::SetTrackingToImu(const Eigen::Matrix4d& t) {
  tracking_to_imu_ = t;
  // CHECK the transform
  PRINT_INFO("Got tf : tracking -> imu ");
  common::PrintTransform(t);
  // @todo(edward) incude z!
  CHECK(t.block(0, 3, 2, 1).norm() < 1.e-6)
      << "The tracking frame should be just lying on the imu_link, otherwise "
         "the acceleration calculation should be wrong!";
}

void MapBuilder::SetTrackingToGps(const Eigen::Matrix4d& t) {
  tracking_to_gps_ = t;
  PRINT_INFO("Got tf : tracking -> gps ");
  common::PrintTransform(t);
}

void MapBuilder::SetTrackingToOdom(const Eigen::Matrix4d& t) {
  tracking_to_odom_ = t;
  PRINT_INFO("Got tf : tracking -> odometry ");
  common::PrintTransform(t);
}

void MapBuilder::SetTrackingToLidar(const Eigen::Matrix4d& t) {
  tracking_to_lidar_ = t;
  PRINT_INFO("Got tf : tracking -> lidar ");
  common::PrintTransform(t);
}

void MapBuilder::InsertPointcloudMsg(const PointCloudPtr& point_cloud) {
  if (end_all_thread_.load() || (use_imu_ && extrapolator_ == nullptr)) {
    return;
  }
  if (extrapolator_ && ToLocalTime(point_cloud->header.stamp) <
                           extrapolator_->GetLastPoseTime()) {
    PRINT_WARNING("skip cloud.");
    return;
  }
  // LOG(INFO) << point_cloud->header.stamp;
  // transform to tracking frame
  pcl::transformPointCloud(*point_cloud, *point_cloud, tracking_to_lidar_);
  data_collector_->AddSensorData<PointType>(point_cloud);
}

void MapBuilder::InsertImuMsg(const data::ImuMsg::Ptr& imu_msg) {
  if (!use_imu_ || end_all_thread_.load()) {
    return;
  }

  const Eigen::Matrix3d rotation = common::Rotation(tracking_to_imu_);
  Eigen::Vector3d new_acc = rotation * imu_msg->linear_acceleration;
  Eigen::Vector3d new_angular_velocity = rotation * imu_msg->angular_velocity;

  imu_msg->linear_acceleration = new_acc;
  imu_msg->angular_velocity = new_angular_velocity;
  data_collector_->AddSensorData(*imu_msg);

  if (!extrapolator_) {
    extrapolator_ = PoseExtrapolator::InitializeWithImu(
        SimpleTime::FromSec(kExpolatorMinDuration),
        options_.front_end_options.imu_options.gravity_constant, *imu_msg);
  } else {
    extrapolator_->AddImuData(*imu_msg);
  }
}

void MapBuilder::InsertOdomMsg(const data::OdomMsg::Ptr& odom_msg) {
  if (!use_odom_ || end_all_thread_.load()) {
    return;
  }

  LOG(FATAL) << "This function is temporarily for very percise odom such as "
                "INS-RTK, and only the pose will be used. If you are in the "
                "same situation, you can comment this line and use this "
                "function, it will be really helpful.";
  data_collector_->AddSensorData(*odom_msg);
}

void MapBuilder::InsertGpsMsg(const data::NavSatFixMsg::Ptr& gps_msg) {
  if (!use_gps_ || end_all_thread_.load()) {
    return;
  }
  data_collector_->AddSensorData(*gps_msg);
}

void MapBuilder::AddNewTrajectory() {
  current_trajectory_.reset(new Trajectory);
  trajectories_.push_back(current_trajectory_);
  current_trajectory_->SetId(static_cast<int>(trajectories_.size()) - 1);
  CHECK_GE(current_trajectory_->GetId(), 0);
  CHECK(current_trajectory_);
  PRINT_INFO_FMT("Add a new trajectory : %d", current_trajectory_->GetId());
}

void MapBuilder::InsertFrameForSubmap(InnerCloud::Ptr cloud_ptr,
                                      const Eigen::Matrix4d& global_pose,
                                      const double match_score) {
  auto frame = std::make_shared<Frame>();
  frame->SetCloud(cloud_ptr);
  frame->SetTimeStamp(cloud_ptr->GetTime());
  frame->SetGlobalPose(global_pose);
  frames_.push_back(frame);
}

namespace {
void MotionCompensation(const data::InnerCloudType& raw_cloud,
                        const Eigen::Matrix4d& delta_transform,
                        data::InnerCloudType* const output_cloud) {
  CHECK(output_cloud);
  output_cloud->points.clear();
  const size_t cloud_size = raw_cloud.points.size();
  output_cloud->stamp = raw_cloud.stamp;
  output_cloud->points.reserve(cloud_size);
  for (size_t i = 0; i < cloud_size; ++i) {
    const auto& point = raw_cloud.points[i];
    const Eigen::Matrix4d transform = common::InterpolateTransform(
        Eigen::Matrix4d::Identity().eval(), delta_transform, point.factor);
    const Eigen::Vector3d new_point_start =
        common::Rotation(transform) *
            Eigen::Vector3d(point.x, point.y, point.z) +
        common::Translation(transform);

    data::InnerPointType new_point;
    new_point.x = new_point_start[0];
    new_point.y = new_point_start[1];
    new_point.z = new_point_start[2];
    new_point.intensity = point.intensity;
    new_point.factor = point.factor;
    output_cloud->points.push_back(new_point);
  }
}
}  // namespace

void MapBuilder::ScanMatchProcessing() {
  using Pose3d = PoseExtrapolator::RigidPose3d;

  scan_match_thread_running_ = true;

  InnerCloud::Ptr target_cloud;
  InnerCloud::Ptr source_cloud;
  Pose3d pose_target = Pose3d::Identity();
  Pose3d accumulative_transform = Pose3d::Identity();
  while (true) {
    auto new_inner_cloud = data_collector_->GetNewCloud();
    if (new_inner_cloud == nullptr) {
      if (end_all_thread_.load()) {
        break;
      }
      SimpleTime::FromSec(0.005).Sleep();
      continue;
    }

    source_cloud.reset(new InnerCloud(new_inner_cloud));
    const auto source_time = source_cloud->GetTime();
    if (!got_first_point_cloud_) {
      got_first_point_cloud_ = true;
      target_cloud = source_cloud;
      // Only IcpFast need the pre-calculated normals of target cloud.
      if (scan_matcher_->GetType() == registrator::Type::kFastIcp) {
        target_cloud->CalculateNormals();
      }
      InsertFrameForSubmap(source_cloud, Eigen::Matrix4d::Identity(), 1.);
      if (extrapolator_) {
        extrapolator_->AddPose(source_time, Eigen::Matrix4d::Identity());
      }
      continue;
    }

    REGISTER_BLOCK("FrontEndOneFrame");
    if (extrapolator_ && source_time < extrapolator_->GetLastPoseTime()) {
      PRINT_INFO("Extrapolator still initialising...");
      target_cloud = source_cloud;
      continue;
    }

    Pose3d pose_source = pose_target;
    if (extrapolator_) {
      pose_source = extrapolator_->ExtrapolatePose(source_time);
    }

    Pose3d guess = pose_target.inverse() * pose_source;
    common::NormalizeRotation(guess);
    // common::PrintTransform(guess);

    // motion compensation using guess
    // still in test
    Eigen::Matrix4d align_result = Eigen::Matrix4d::Identity();
    auto inner_cloud_without_comp = source_cloud->GetInnerCloud();
    {
      REGISTER_BLOCK("scan match:target");
      scan_matcher_->SetInputTarget(target_cloud);

      // && options_.front_end_options.motion_compensation_options.use_average
      if (options_.front_end_options.motion_compensation_options.enable) {
        data::InnerCloudType::Ptr compensated_source_cloud(
            new data::InnerCloudType);
        const Pose3d motion_in_source_cloud =
            accumulative_transform.inverse() * guess;
        MotionCompensation(*inner_cloud_without_comp, motion_in_source_cloud,
                           compensated_source_cloud.get());
        source_cloud->SetInnerCloud(compensated_source_cloud);
      }
      scan_matcher_->SetInputSource(source_cloud);
    }
    {
      REGISTER_BLOCK("scan match:align");
      scan_matcher_->Align(guess, align_result);
    }

    if (options_.front_end_options.motion_compensation_options.enable) {
      Eigen::Matrix4d average_transform = align_result;
      if (options_.front_end_options.motion_compensation_options.use_average) {
        std::vector<Eigen::Matrix4d> transforms;
        transforms.push_back(align_result);
        transforms.push_back(guess);
        average_transform = common::AverageTransforms(transforms);
        align_result = average_transform;
      }
      // motion compensation using align result
      data::InnerCloudType::Ptr compensated_source_cloud(
          new data::InnerCloudType);
      MotionCompensation(*inner_cloud_without_comp,
                         accumulative_transform.inverse() * average_transform,
                         compensated_source_cloud.get());
      source_cloud->SetInnerCloud(compensated_source_cloud);
    }

    pose_source = pose_target * align_result;
    accumulative_transform = align_result;
    if (extrapolator_) {
      extrapolator_->AddPose(source_time, pose_source);
      // const Eigen::Quaterniond gravity_alignment =
      //     extrapolator_->EstimateGravityOrientation(source_time);

      // Eigen::Matrix4d gravity_alignment_transform =
      // Eigen::Matrix4d::Identity(); gravity_alignment_transform.block(0, 0, 3,
      // 3) =
      //     gravity_alignment.inverse().toRotationMatrix();
      // pose_source = pose_source * gravity_alignment_transform;
      // accumulative_transform =
      //     accumulative_transform * gravity_alignment_transform;
    }

    const float accu_translation =
        common::Translation(accumulative_transform).norm();
    Eigen::Vector3d euler_angles = common::RotationMatrixToEulerAngles(
        common::Rotation(accumulative_transform));
    const float accu_angles =
        (std::fabs(euler_angles[0]) + std::fabs(euler_angles[1]) +
         std::fabs(euler_angles[2])) *
        (180. / M_PI);
    // Meet the condition of inserting new key frame.
    if (accu_translation >=
            options_.front_end_options.motion_filter.translation_range ||
        (options_.front_end_options.motion_filter.angle_range > 1e-3 &&
         accu_angles >= options_.front_end_options.motion_filter.angle_range)) {
      InsertFrameForSubmap(source_cloud, pose_source,
                           scan_matcher_->GetFitnessScore());

      accumulative_transform = Pose3d::Identity();
      target_cloud = source_cloud;
      if (scan_matcher_->GetType() == registrator::Type::kFastIcp) {
        target_cloud->CalculateNormals();
      }
      pose_target = pose_source;
    }
  }

  scan_match_thread_running_ = false;
  PRINT_INFO("point cloud thread exit.");
}

void MapBuilder::SubmapPairMatch(const int source_index,
                                 const int target_index) {
  REGISTER_FUNC;
  std::shared_ptr<Submap> target_submap, source_submap;
  target_submap = current_trajectory_->at(target_index);
  source_submap = current_trajectory_->at(source_index);

  // init back end(submap to submap matcher)
  std::shared_ptr<registrator::Interface> matcher;
  {
    common::MutexLocker locker(&mutex_);
    // Creating a Matcher has some opertation on an instance of pugi::xml_node
    // and we do not known if it's threadsafe, so add a locker
    matcher = registrator::CreateMatcher(
        options_.back_end_options.submap_matcher_options, false);
  }
  CHECK(matcher);

  matcher->SetInputSource(source_submap->Cloud());
  {
    // REGISTER_BLOCK("SubmapPairMatch:target");
    matcher->SetInputTarget(target_submap->Cloud());
  }
  Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
  // `GetFrames()[0]->GlobalPose()` means the first frame's global pose, and it
  // will never be changed after scan matching. So it's actually safe to
  // calculate the init estimation like this.
  Eigen::Matrix4d guess =
      target_submap->GetFrames()[0]->GlobalPose().inverse() *
      source_submap->GetFrames()[0]->GlobalPose();
  {
    // REGISTER_BLOCK("SubmapPairMatch:align");
    matcher->Align(guess, result);
  }
  common::NormalizeRotation(result);
  double submap_match_score = matcher->GetFitnessScore();
  source_submap->match_score_to_previous_submap_ = submap_match_score;
  if (submap_match_score >=
      options_.back_end_options.submap_matcher_options.accepted_min_score) {
    target_submap->SetMatchedTransformedToNext(result);
  } else {
    // do nothing
    // keep the former transform
    target_submap->SetMatchedTransformedToNext(guess);
    PRINT_WARNING("keep the transform got from scan matching.");
  }
  matcher.reset();
}

void MapBuilder::ConnectAllSubmap() {
  // Finish means that its global pose is ready
  int current_finished_index = 0;
  std::vector<std::shared_ptr<Submap>> submaps_to_connect;
  submaps_to_connect.reserve(20);
  bool first_inserted = false;
  while (true) {
    int submap_size = current_trajectory_->size();
    if (submap_size == 0 || submap_size - 1 == current_finished_index) {
      if (submap_processing_done_.load()) {
        break;
      }
      SimpleTime::FromSec(0.1).Sleep();
      continue;
    }

    if (!first_inserted) {
      first_inserted = true;
      // it is the first frame (first submap)
      isam_optimizer_->AddFrame(
          current_trajectory_->front(),
          current_trajectory_->front()->match_score_to_previous_submap_);
    }

    submaps_to_connect.clear();
    for (int i = current_finished_index; i < submap_size; ++i) {
      submaps_to_connect.push_back((*current_trajectory_)[i]);
      if (!(*current_trajectory_)[i]->GotMatchedToNext()) {
        break;
      }
    }

    if (submaps_to_connect.size() <= 1) {
      SimpleTime::FromSec(0.1).Sleep();
      continue;
    }
    for (size_t i = 1; i < submaps_to_connect.size(); ++i) {
      auto& previous_submap = submaps_to_connect[i - 1];
      auto& current_submap = submaps_to_connect[i];
      current_submap->SetGlobalPose(previous_submap->GlobalPose() *
                                    previous_submap->TransformToNext());
      current_submap->SetTransformFromLast(previous_submap->TransformToNext());
      current_finished_index++;

      isam_optimizer_->AddFrame(
          current_submap, current_submap->match_score_to_previous_submap_);

      if (show_submap_function_) {
        PointCloudPtr submap_cloud_pcl(new PointCloudType);
        data::ToPclPointCloud(*(current_submap->Cloud()->GetInnerCloud()),
                              submap_cloud_pcl.get());
        show_submap_function_(submap_cloud_pcl, current_submap->GlobalPose());
      }
    }

    if (show_path_function_) {
      std::vector<Eigen::Matrix4d> poses;
      poses.reserve(current_trajectory_->size());
      for (auto& submap : *current_trajectory_) {
        if (submap->GetId().submap_index <= current_finished_index) {
          poses.push_back(submap->GlobalPose());
        }
      }
      show_path_function_(poses);
    }

    if (show_map_function_) {
      PointCloudPtr local_map(new PointCloudType);
      for (auto& submap : *current_trajectory_) {
        if (submap->GetId().submap_index > current_finished_index) {
          break;
        }

        const Eigen::Vector3d delta_translation =
            submap->GlobalTranslation() -
            current_trajectory_->at(current_finished_index)
                ->GlobalTranslation();
        if (delta_translation.topRows(2).norm() >= kLocalMapRange ||
            std::fabs(delta_translation.z()) >= 2.) {
          continue;
        }

        PointCloudPtr transformed_cloud(new PointCloudType);
        Eigen::Matrix4d pose = submap->GlobalPose();

        PointCloudPtr submap_cloud_pcl(new PointCloudType);
        data::ToPclPointCloud(*(submap->Cloud()->GetInnerCloud()),
                              submap_cloud_pcl.get());
        pcl::transformPointCloud(*submap_cloud_pcl, *transformed_cloud, pose);
        *local_map += *transformed_cloud;
      }

      // TODO(edward) show_map_function_ use inner point type instread of pcl
      // cloud.
      data::InnerCloudType::Ptr inner_filtered_final_cloud(
          new data::InnerCloudType);
      PointCloudPtr filtered_final_cloud(new PointCloudType);
      pre_processers::filter::RandomSampler random_sample;
      auto inner_local_map = data::ToInnerPointCloud(*local_map);
      random_sample.SetInputCloud(inner_local_map);
      random_sample.SetValue("sampling_rate", 0.05);
      random_sample.Filter(inner_filtered_final_cloud);
      data::ToPclPointCloud(*inner_filtered_final_cloud,
                            filtered_final_cloud.get());
      show_map_function_(filtered_final_cloud);
    }

    if (show_edge_function_) {
      show_edge_function_(isam_optimizer_->GetWholeGraph());
    }
  }

  // for last submap
  // make sure that all submap output to file if need
  if (!current_trajectory_->empty()) {
    current_trajectory_->back()->SetMatchedTransformedToNext(
        Eigen::Matrix4d::Identity());
  }
  PRINT_INFO("All submaps have been connected.");

  isam_optimizer_->RunFinalOptimazation();
  if (use_odom_) {
    switch (options_.whole_options.odom_calib_mode) {
      case kNoCalib:
        PRINT_INFO("Do no calibration.");
        break;
      case kOnlineCalib:
        PRINT_INFO("Update tf(odom->lidar) from online calibration.");
        transform_odom_lidar_ = isam_optimizer_->GetTransformOdomToLidar();
        break;
      case kOfflineCalib: {
        PRINT_ERROR(
            "It is a deprecated function. If you got a RTK-INS for odom, we "
            "will do the online calibration of the extrinsic matrices in the "
            "back-end. No offline mode supported.");
        // Eigen::Matrix4d result;
        // if (OfflineCalibrationOdomToLidar<PointType>(
        //         current_trajectory_, transform_odom_lidar_, &result)) {
        //   PRINT_INFO("Update tf(odom->lidar) from offline calibration.");
        //   transform_odom_lidar_ = result;
        // }
      } break;
      default:
        PRINT_ERROR("Unknown Calibration Mode.");
        break;
    }
  }

  // all frame connected
  // generate results (clouds to pcd files)
  // output the path into pointcloud as a pcd file
  for (auto& submap : *current_trajectory_) {
    submap->UpdateInnerFramePose();
  }

  // calculate the coord transform from the map gps (enu)
  CalculateCoordTransformToGps();
  // path to pcd file after rotation into enu corrdinate
  OutputPath();
  // @todo add a paramter for map filename
  GenerateMapPackage(options_.whole_options.map_package_path + "map.xml");

  SaveMapPackage();
  SaveMaps();
  data_collector_->ClearAllCloud();
}

void MapBuilder::OutputPath() {
  PRINT_INFO("generating path files ...");

  current_trajectory_->OutputPathToPointcloud(
      options_.whole_options.export_file_path);

  data_collector_->RawGpsDataToFile(options_.whole_options.export_file_path +
                                    "original_gps.pcd");
  data_collector_->RawOdomDataToFile(options_.whole_options.export_file_path +
                                     "original_odom.pcd");
}

void MapBuilder::SubmapProcessing() {
  current_trajectory_->reserve(kSubmapResSize);
  auto& submap_options = options_.back_end_options.submap_options;
  const int submap_frame_count = submap_options.frame_count;

  std::unique_ptr<MemoryManager> submap_mem_manager;
  if (options_.back_end_options.submap_options.enable_disk_saving) {
    submap_mem_manager.reset(new MemoryManager(&trajectories_));
  }

  size_t current_index = 0;
  common::ThreadPool submap_match_thread_pool(6);
  // there is a deamon thread in this thread pool
  // it is to connect all submap into a global map
  // others are for submap matching
  submap_match_thread_pool.enqueue([&]() { ConnectAllSubmap(); });

  while (true) {
    const int frames_size = frames_.size();
    if (frames_size - current_index < submap_frame_count) {
      if (!scan_match_thread_running_.load()) {
        PRINT_INFO("no enough frames for new submap, quit");
        break;
      }
      SimpleTime::FromSec(0.5).Sleep();
      continue;
    }

    // Adding new submap using some frames.
    std::shared_ptr<Submap> submap(new Submap(submap_options));
    SubmapId current_id;
    current_id.trajectory_index = current_trajectory_->GetId();
    auto& current_submap_index = current_id.submap_index;
    current_submap_index = current_trajectory_->size();
    submap->SetId(current_id);
    submap->SetSavePath(options_.whole_options.map_package_path);
    // create a submap and init with the configs
    PRINT_DEBUG_FMT("Add a new submap : %d", current_submap_index);
    for (int i = 0; i < submap_frame_count; ++i) {
      submap->InsertFrame(frames_[current_index]);
      current_index++;
    }
    CHECK(submap->Full());
    submap->CalculateDescriptor();

    const auto time = submap->GetTimeStamp();
    if (use_gps_) {
      const auto gps_enu = data_collector_->InterpolateGps(time, 0.001, true);
      if (gps_enu) {
        submap->SetRelatedGpsInENU(*gps_enu);
      }
    }
    if (use_odom_) {
      const auto odom = data_collector_->InterpolateOdom(time, 0.001, true);
      if (odom) {
        submap->SetRelatedOdom(*odom);
      }
    }

    // Submap constructing finished, add to trajectory.
    current_trajectory_->push_back(submap);
    if (current_submap_index > 0) {
      submap_match_thread_pool.enqueue([=]() {
        SubmapPairMatch(current_submap_index, current_submap_index - 1);
      });
    }
  }
  submap_processing_done_ = true;
  PRINT_INFO("submap processing done.");
}

void MapBuilder::FinishAllComputations() {
  PRINT_INFO("Finishing Remaining Computations...");
  end_all_thread_ = true;

  size_t remaining_pointcloud_count = data_collector_->GetRemainingCloudSize();
  int delay = 0;
  SimpleTime delay_time = SimpleTime::FromSec(0.1);
  if (scan_match_thread_) {
    do {
      delay_time.Sleep();
      delay++;
      if (delay >= 20) {
        std::ostringstream progress_info;
        progress_info << "Remaining Point Cloud : "
                      << (1. - static_cast<double>(
                                   data_collector_->GetRemainingCloudSize()) /
                                   remaining_pointcloud_count) *
                             100.
                      << "% ...";
        PRINT_COLOR_FMT(BOLD, "%s", progress_info.str().c_str());
        delay = 0;
      }
    } while (scan_match_thread_running_.load());

    scan_match_thread_->join();
  }

  if (submap_thread_ && submap_thread_->joinable()) {
    submap_thread_->join();
    submap_thread_.reset();
  }
}

void MapBuilder::CalculateCoordTransformToGps() {
  if (!use_gps_) {
    return;
  }

  // @todo(edward) get gps origin
  const Eigen::Matrix4d map_enu_transform =
      isam_optimizer_->GetGpsCoordTransform();
  // update the submap pose and frame pose
  for (auto& submap : *current_trajectory_) {
    Eigen::Matrix4d new_submap_pose = map_enu_transform * submap->GlobalPose();
    submap->SetGlobalPose(new_submap_pose);
    submap->UpdateInnerFramePose();
  }
}

void MapBuilder::SetShowMapFunction(const ShowMapFunction& func) {
  show_map_function_ = std::move(func);
}

void MapBuilder::SetShowPathFunction(const ShowPathFunction& func) {
  show_path_function_ = std::move(func);
}

void MapBuilder::SetShowSubmapFunction(const ShowSubmapFunction& func) {
  show_submap_function_ = std::move(func);
}

void MapBuilder::SetShowEdgeFunction(const ShowEdgeFunction& func) {
  show_edge_function_ = std::move(func);
}

void MapBuilder::EnableUsingOdom(bool flag) {
  use_odom_ = flag;
  std::string str = flag ? "enable" : "disable";
  PRINT_INFO_FMT("%s odom.", str.c_str());
  if (!flag) {
    options_.back_end_options.isam_optimizer_options.use_odom = false;
  }
}

void MapBuilder::EnableUsingGps(bool flag) {
  use_gps_ = flag;
  std::string str = flag ? "enable" : "disable";
  PRINT_INFO_FMT("%s gps.", str.c_str());
  if (!flag) {
    if (options_.back_end_options.loop_detector_setting.use_gps) {
      PRINT_WARNING(
          "Gps is disabled, loop_detector_setting.use_gps is set to false.");
      options_.back_end_options.loop_detector_setting.use_gps = false;
    }
    if (options_.back_end_options.isam_optimizer_options.use_gps) {
      PRINT_WARNING(
          "Gps is disabled, isam_optimizer_options.use_gps is set to false.");
      options_.back_end_options.isam_optimizer_options.use_gps = false;
    }
  }
}

void MapBuilder::GenerateMapPackage(const std::string& filename) {
  pugi::xml_document doc;
  pugi::xml_node map_node = doc.append_child("Map");
  for (auto& single_trajectory : trajectories_) {
    single_trajectory->ToXmlNode(&map_node);
  }
  doc.save_file(filename.c_str());
}

void MapBuilder::SaveMapPackage() {
  if (!options_.map_package_options.enable) {
    return;
  }
  SaveTrajectoriesAsMapPackage(trajectories_, options_.map_package_options,
                               options_.output_mrvm_settings,
                               options_.whole_options.export_file_path);
}

void MapBuilder::SaveMaps() {
  if (!options_.whole_options.output_direct_combined_map &&
      !options_.whole_options.output_mrvm) {
    PRINT_INFO("Output nothing according to config.");
    return;
  }

  std::shared_ptr<MultiResolutionVoxelMap> mrvm_map(
      new MultiResolutionVoxelMap());
  mrvm_map->Initialise(options_.output_mrvm_settings);
  PointCloudPtr whole_map(new PointCloudType);
  const int submaps_size = current_trajectory_->size();
  for (auto& submap : *current_trajectory_) {
    const int submap_idx = submap->GetId().submap_index;
    std::cout << "submap index: " << submap_idx << " / " << submaps_size - 1
              << "\r" << std::flush;
    for (auto& frame : submap->GetFrames()) {
      data::InnerCloudType::Ptr output_cloud(new data::InnerCloudType);
      frame->Cloud()->GetInnerCloud()->ApplyTransformToOutput(
          frame->GlobalPose(), output_cloud.get());
      if (options_.whole_options.output_mrvm) {
        // REGISTER_BLOCK("mrvp_insert_one_frame");
        mrvm_map->InsertPointCloud(output_cloud,
                                   frame->GlobalTranslation().cast<float>());
      }
      if (options_.whole_options.output_direct_combined_map) {
        PointCloudType pcl_output_cloud;
        data::ToPclPointCloud(*output_cloud, &pcl_output_cloud);
        *whole_map += pcl_output_cloud;
      }
    }

    if (options_.whole_options.separate_output) {
      if (submap_idx == submaps_size - 1 ||
          (submap_idx + 1) % options_.whole_options.separate_step == 0) {
        const int map_part_index =
            submap_idx / options_.whole_options.separate_step;
        const int used_submap_count =
            submap_idx % options_.whole_options.separate_step + 1;

        if (options_.whole_options.output_direct_combined_map) {
          if (whole_map && !whole_map->empty()) {
            PRINT_INFO_FMT("creating part map: %d with %d submaps.",
                           map_part_index, used_submap_count);
            pcl::io::savePCDFileBinaryCompressed(
                options_.whole_options.export_file_path + "part_map_" +
                    std::to_string(map_part_index) + ".pcd",
                *whole_map);
            whole_map.reset(new PointCloudType);
          } else {
            PRINT_INFO_FMT("failed creating part map: %d with %d submaps.",
                           map_part_index, used_submap_count);
          }
        }
        if (options_.whole_options.output_mrvm && mrvm_map != nullptr) {
          PRINT_INFO_FMT("creating static part map: %d with %d submaps.",
                         map_part_index, used_submap_count);
          mrvm_map->OutputToPointCloud(
              options_.output_mrvm_settings.prob_threshold,
              options_.whole_options.export_file_path + "static_part_map_" +
                  std::to_string(map_part_index) + ".pcd");

          mrvm_map.reset(new MultiResolutionVoxelMap());
          mrvm_map->Initialise(options_.output_mrvm_settings);
        }
      }
    }

    submap->ClearCloud();
    submap->ClearCloudInFrames();
  }
  if (!options_.whole_options.separate_output) {
    if (options_.whole_options.output_mrvm) {
      PRINT_INFO("creating the static map ...");
      mrvm_map->OutputToPointCloud(
          options_.output_mrvm_settings.prob_threshold,
          options_.whole_options.export_file_path + "static_map.pcd");
    }
    if (options_.whole_options.output_direct_combined_map &&
        !whole_map->empty()) {
      PRINT_INFO("creating the whole map ...");
      pcl::io::savePCDFileBinaryCompressed(
          options_.whole_options.export_file_path + "whole_map.pcd",
          *whole_map);
    }
  }
}

}  // namespace static_map
