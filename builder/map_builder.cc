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
#include <utility>

// pcl
#include "pcl/filters/statistical_outlier_removal.h"
#include "unsupported/Eigen/MatrixFunctions"

// local headers
#include "builder/map_builder.h"
#include "builder/utm.h"
#include "common/macro_defines.h"
#include "common/make_unique.h"
#include "common/pugixml.hpp"
#include "cost_functions/odom_map_match.h"
#include "descriptor/m2dp.h"
#include "registrators/icp_fast.h"
#include "registrators/icp_libicp.h"
#include "registrators/lego_loam.h"
#include "registrators/ndt.h"
#include "registrators/ndt_gicp.h"

namespace static_map {

using registrator::IcpUsingLibicp;
using registrator::IcpUsingPointMatcher;
using registrator::LegoLoam;
using registrator::Ndt;
using registrator::NdtWithGicp;

constexpr int kOdomMsgMaxSize = 10000;
constexpr int kSubmapResSize = 100;
// usually, our longtitude is about 121E, in UTM "51R" zone
constexpr int kUtmZone = 51;

MapBuilder::MapBuilder()
    : accumulated_point_cloud_(new PointCloudType),
      accumulated_cloud_count_(0),
      use_imu_(false),
      use_gps_(false),
      end_all_thread_(false),
      end_managing_memory_(false),
      submap_processing_done_(false) {
  point_clouds_.reserve(200);
}
MapBuilder::~MapBuilder() {}

int MapBuilder::InitialiseInside() {
  if (scan_matcher_ != nullptr || submap_marcher_ != nullptr) {
    return -1;
  }

  PRINT_INFO("Init scan matchers.");
  // init front end (scan to scan matcher)
  auto& scan_matcher_options = options_.front_end_options.scan_matcher_options;
  switch (scan_matcher_options.type) {
    case registrator::kIcpPM:
      scan_matcher_ = common::make_unique<IcpUsingPointMatcher<PointType>>();
      break;
    case registrator::kLibicp:
      scan_matcher_ = common::make_unique<IcpUsingLibicp<PointType>>();
      break;
    case registrator::kNdtWithGicp:
      scan_matcher_ = common::make_unique<NdtWithGicp<PointType>>(
          scan_matcher_options.use_voxel_filter,
          scan_matcher_options.voxel_filter_resolution);
      dynamic_cast<NdtWithGicp<PointType>*>(scan_matcher_.get())
          ->enableNdt(scan_matcher_options.enable_ndt);
      break;
    case registrator::kLegoLoam:
      scan_matcher_ = common::make_unique<LegoLoam<PointType>>();
      dynamic_cast<LegoLoam<PointType>*>(scan_matcher_.get())
          ->InitialiseFiltersFromXmlNode(
              scan_matcher_options.inner_filters_node);
      break;
    case registrator::kNdt:
      scan_matcher_ = common::make_unique<Ndt<PointType>>();
      break;
    default:
      PRINT_ERROR("Wrong type");
      return -1;
  }

  use_imu_ = options_.front_end_options.imu_options.enabled;
  CHECK_GT(options_.front_end_options.imu_options.frequency, 1.e-6);
  imu_update_peroid_ = 1.f / options_.front_end_options.imu_options.frequency;

  PRINT_INFO("Init isam optimizer.");
  isam_optimizer_ = common::make_unique<back_end::IsamOptimizer<PointType>>(
      options_.back_end_options.isam_optimizer_options,
      options_.back_end_options.loop_detector_setting);
  isam_optimizer_->SetTransformOdomToLidar(transform_odom_lidar_);

#ifdef _USE_OPENCV_
  PRINT_INFO("Enable openCV.");
#endif

#ifdef _USE_TBB_
  PRINT_INFO("Enable TBB.");
#endif

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
  scan_match_thread_ = common::make_unique<std::thread>(
      std::bind(&MapBuilder::ScanMatchProcessing, this));
  submap_thread_ = common::make_unique<std::thread>(
      std::bind(&MapBuilder::SubmapProcessing, this));

  PRINT_INFO("Init finished.");
  return 0;
}

void MapBuilder::SetTransformOdomToLidar(const Eigen::Matrix4f& t) {
  transform_odom_lidar_ = t;
  PRINT_INFO("Got tf : odom -> lidar ");
  common::PrintTransform(t);
}

void MapBuilder::SetTransformImuToLidar(const Eigen::Matrix4f& t) {
  transform_imu_lidar_ = t;
  PRINT_INFO("Got tf : imu -> lidar ");
  common::PrintTransform(t);
}

void MapBuilder::SetTrackingToImu(const Eigen::Matrix4f& t) {
  tracking_to_imu_ = t;
  // CHECK the transform
  PRINT_INFO("Got tf : tracking -> imu ");
  common::PrintTransform(t);
  // @todo(edward) incude z!
  CHECK(t.block(0, 3, 2, 1).norm() < 1.e-6)
      << "The tracking frame should be just lying on the imu_link, otherwise "
         "the acceleration calculation should be wrong!";
}

void MapBuilder::SetTrackingToOdom(const Eigen::Matrix4f& t) {
  tracking_to_odom_ = t;
  PRINT_INFO("Got tf : tracking -> imu ");
  common::PrintTransform(t);
}

void MapBuilder::SetTrackingToLidar(const Eigen::Matrix4f& t) {
  tracking_to_lidar_ = t;
  PRINT_INFO("Got tf : tracking -> lidar ");
  common::PrintTransform(t);
}

void MapBuilder::InsertPointcloudMsg(const PointCloudPtr& point_cloud) {
  if (end_all_thread_.load() || extrapolator_ == nullptr ||
      sensors::ToLocalTime(point_cloud->header.stamp) <
          extrapolator_->GetLastPoseTime()) {
    return;
  }

  // transform to tracking frame
  pcl::transformPointCloud(*point_cloud, *point_cloud, tracking_to_lidar_);
  // accumulating clouds into one
  if (options_.front_end_options.accumulate_cloud_num > 1) {
    common::MutexLocker locker(&mutex_);
    *accumulated_point_cloud_ += *point_cloud;
    accumulated_cloud_count_++;
    if (accumulated_cloud_count_ <
        options_.front_end_options.accumulate_cloud_num) {
      return;
    }
  } else {
    accumulated_point_cloud_.reset();
    accumulated_point_cloud_ = point_cloud;
  }
  // filtering cloud
  PointCloudPtr filtered_cloud(new PointCloudType);
  DownSamplePointcloud(accumulated_point_cloud_, filtered_cloud);

  // registrator::IcpFast<PointType> matcher;
  // matcher.setInputTarget(point_cloud);

  point_cloud->points.clear();
  point_cloud->points.shrink_to_fit();
  accumulated_cloud_count_ = 0;
  accumulated_point_cloud_->clear();

  common::MutexLocker locker(&mutex_);
  point_clouds_.push_back(filtered_cloud);
  // just for debug
  got_clouds_count_++;
  if (got_clouds_count_ % 100 == 0) {
    PRINT_INFO_FMT("Got %u clouds already.", got_clouds_count_);
  }
}

void MapBuilder::InsertImuMsg(const sensors::ImuMsg::Ptr& imu_msg) {
  if (!use_imu_ || end_all_thread_.load()) {
    return;
  }

  const Eigen::Matrix3d rotation =
      common::Rotation(tracking_to_imu_).cast<double>();
  Eigen::Vector3d new_acc =
      rotation * Eigen::Vector3d(imu_msg->linear_acceleration.x,
                                 imu_msg->linear_acceleration.y,
                                 imu_msg->linear_acceleration.z);
  Eigen::Vector3d new_angular_velocity =
      rotation * Eigen::Vector3d(imu_msg->angular_velocity.x,
                                 imu_msg->angular_velocity.y,
                                 imu_msg->angular_velocity.z);

  imu_msg->linear_acceleration.x = new_acc[0];
  imu_msg->linear_acceleration.y = new_acc[1];
  imu_msg->linear_acceleration.z = new_acc[2];
  imu_msg->angular_velocity.x = new_angular_velocity[0];
  imu_msg->angular_velocity.y = new_angular_velocity[1];
  imu_msg->angular_velocity.z = new_angular_velocity[2];

  if (!extrapolator_) {
    extrapolator_ = PoseExtrapolator::InitializeWithImu(
        SimpleTime::from_sec(0.001),
        options_.front_end_options.imu_options.gravity_constant, *imu_msg);
  } else {
    extrapolator_->AddImuData(*imu_msg);
  }
}

void MapBuilder::InsertOdomMsg(const sensors::OdomMsg::Ptr& odom_msg) {
  if (!use_odom_ || end_all_thread_.load()) {
    return;
  }

  // transform to tracking frame
  const Eigen::Matrix4d tracking_frame_odom =
      odom_msg->PoseInMatrix() * tracking_to_odom_.inverse().cast<double>();
  odom_msg->SetPose(tracking_frame_odom);

  static bool warned = false;
  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator we cannot add odometry data.
    if (!warned) {
      PRINT_WARNING("Extrapolator not yet initialized.");
      warned = true;
    }
    return;
  }
  extrapolator_->AddOdometryData(*odom_msg);

  common::MutexLocker locker(&mutex_);
  // odom_msgs_ are just for generating path file now
  if (odom_msgs_.empty()) {
    init_odom_msg_ = *odom_msg;
  }
  Eigen::Matrix4d init_pose = init_odom_msg_.PoseInMatrix();
  Eigen::Matrix4d relative_pose =
      init_pose.inverse() * odom_msg->PoseInMatrix();
  odom_msg->SetPose(relative_pose);
  odom_msgs_.push_back(odom_msg);

  // for output pcd
  Eigen::Vector3d odom_path_point;
  odom_path_point << odom_msg->pose.pose.position.x,
      odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z;
  odom_path_.push_back(odom_path_point);
}

void MapBuilder::InsertGpsMsg(const sensors::NavSatFixMsg::Ptr& gps_msg) {
  if (!use_gps_ || end_all_thread_.load()) {
    return;
  }
  sensors::UtmMsg::Ptr utm(new sensors::UtmMsg);
  utm::LatLonToUTMXY(gps_msg->latitude, gps_msg->longtitude, kUtmZone, utm->x,
                     utm->y);
  utm->z = gps_msg->altitude;

  if (!utm_init_offset_) {
    if (gps_msg->status.status != sensors::STATUS_FIX) {
      return;
    }
    utm_init_offset_ = Eigen::Vector3d(utm->x, utm->y, utm->z);
  }
  utm->x -= utm_init_offset_.value()[0];
  utm->y -= utm_init_offset_.value()[1];
  utm->z -= utm_init_offset_.value()[2];

  Eigen::Vector4d utm_point;
  utm_point[0] = utm->x;
  utm_point[1] = utm->y;
  utm_point[2] = utm->z;
  utm_point[3] = static_cast<double>(gps_msg->status.status);
  common::MutexLocker locker(&mutex_);
  utm_path_.push_back(utm_point);
  if (gps_msg->status.status != sensors::STATUS_FIX) {
    return;
  }

  utm->header = gps_msg->header;
  utm_msgs_.push_back(utm);
}

void MapBuilder::AddNewTrajectory() {
  current_trajectory_.reset(new Trajectory<PointType>);
  trajectories_.push_back(current_trajectory_);
  current_trajectory_->SetId(static_cast<int>(trajectories_.size()) - 1);
  CHECK_GE(current_trajectory_->GetId(), 0);
  CHECK(current_trajectory_);
  PRINT_INFO_FMT("Add a new trajectory : %d", current_trajectory_->GetId());
}

void MapBuilder::InsertFrameForSubmap(const PointCloudPtr& cloud_ptr,
                                      const Eigen::Matrix4f& global_pose,
                                      const double match_score) {
  auto frame = std::make_shared<Frame<PointType>>();
  frame->SetCloud(cloud_ptr);
  frame->CalculateDescriptor();
  frame->SetTimeStamp(sensors::ToLocalTime(cloud_ptr->header.stamp));
  frame->SetGlobalPose(global_pose);

  common::MutexLocker locker(&mutex_);
  frames_.push_back(frame);
}

void MapBuilder::ScanMatchProcessing() {
  using Pose3d = PoseExtrapolator::RigidPose3d;

  scan_match_thread_running_ = true;

  PointCloudPtr target_cloud;
  PointCloudPtr source_cloud;
  Pose3d pose_target = Pose3d::Identity();
  Pose3d final_transform = Pose3d::Identity();
  Pose3d accumulative_transform = Pose3d::Identity();
  SimpleTime last_source_time;

  // get a new cloud from the cloud buffer
  // it is usually not the newest one but hasn't been calculated
  const auto get_new_cloud = [&](PointCloudPtr& cloud) -> bool {
    common::MutexLocker locker(&mutex_);
    if (point_clouds_.empty()) {
      return false;
    }
    cloud = point_clouds_[0];
    point_clouds_.erase(point_clouds_.begin());
    return true;
  };

  bool first_in_accumulate = true;
  PointCloudPtr history_cloud = nullptr;
  while (true) {
    if (get_new_cloud(source_cloud)) {
      auto source_time = sensors::ToLocalTime(source_cloud->header.stamp);
      if (!got_first_point_cloud_) {
        got_first_point_cloud_ = true;
        target_cloud = source_cloud;
        history_cloud = target_cloud;
        InsertFrameForSubmap(source_cloud, Eigen::Matrix4f::Identity(), 1.);
        continue;
      }
    } else {
      if (end_all_thread_.load()) {
        break;
      }
      SimpleTime::from_sec(0.01).sleep();
      continue;
    }

    const auto source_time = sensors::ToLocalTime(source_cloud->header.stamp);
    if (source_time < extrapolator_->GetLastPoseTime()) {
      PRINT_INFO("Extrapolator still initialising...");
      target_cloud = source_cloud;
      continue;
    }
    Pose3d pose_source = extrapolator_->ExtrapolatePose(source_time);

    // common::PrintTransform(pose_source);

    Pose3d guess = pose_target.inverse() * pose_source;
    common::NormalizeRotation(guess);

    // filter some registration if not moving
    // const double guess_angles =
    //     common::RotationMatrixToEulerAngles(common::Rotation(guess)).norm() *
    //     180. / M_PI;
    // const double guess_translation = common::Translation(guess).norm();
    // if (guess_translation <
    //         options_.front_end_options.motion_filter.translation_range &&
    //     guess_angles < options_.front_end_options.motion_filter.angle_range
    //     && sensors::ToLocalTime(source_cloud->header.stamp -
    //                          target_cloud->header.stamp)
    //             .toSec() <
    //         options_.front_end_options.motion_filter.time_range ) {
    //   source_cloud.reset();
    //   continue;
    // }

    scan_matcher_->setInputTarget(target_cloud);
    scan_matcher_->setInputSource(source_cloud);
    Eigen::Matrix4f align_result = Eigen::Matrix4f::Identity();
    scan_matcher_->align(guess.cast<float>(), align_result);
    accumulative_transform *= align_result.cast<double>();

    pose_source = pose_target * align_result.cast<double>();
    extrapolator_->AddPose(source_time, pose_source);
    const Eigen::Quaterniond gravity_alignment =
        extrapolator_->EstimateGravityOrientation(source_time);
    const float accu_translation =
        common::Translation(accumulative_transform).norm();
    Eigen::Vector3d euler_angles = common::RotationMatrixToEulerAngles(
        common::Rotation(accumulative_transform));
    const float accu_angles =
        (std::fabs(euler_angles[0]) + std::fabs(euler_angles[1]) +
         std::fabs(euler_angles[2])) *
        (180. / M_PI);
    const float translation_range =
        options_.front_end_options.motion_filter.translation_range;
    const float angle_range =
        options_.front_end_options.motion_filter.angle_range;
    if (accu_translation >= translation_range ||
        (angle_range > 1e-3 && accu_angles >= angle_range)) {
      // re-align if nessary
      if (!first_in_accumulate) {
        scan_matcher_->setInputSource(source_cloud);
        scan_matcher_->setInputTarget(history_cloud);
        Eigen::Matrix4f tmp_result;
        scan_matcher_->align(accumulative_transform.cast<float>(), tmp_result);
        accumulative_transform = tmp_result.cast<double>();
      }

      final_transform *= accumulative_transform;
      // extrapolator_->AddPose(sensors::ToLocalTime(source_cloud->header.stamp),
      //                        final_transform);
      InsertFrameForSubmap(source_cloud, final_transform.cast<float>(),
                           scan_matcher_->getFitnessScore());
      pose_source = final_transform;

      accumulative_transform = Pose3d::Identity();
      history_cloud = source_cloud;
      first_in_accumulate = true;
    } else {
      first_in_accumulate = false;
    }

    target_cloud = source_cloud;
    pose_target = pose_source;
  }

  scan_match_thread_running_ = false;
  PRINT_INFO("point cloud thread exit.");
}

sensors::OdomMsg InterpolateOdom(
    const sensors::OdomMsg& a,  ///< The first isometry.
    const sensors::OdomMsg& b,  ///< The second isometry.
    const SimpleTime& time      ///< The interpolation target time.
) {
  sensors::OdomMsg data;
  data.header.stamp = time;

  float factor = (time - a.header.stamp).toSec() /
                 (b.header.stamp - a.header.stamp).toSec();
  CHECK(factor >= 0. && factor <= 1.);
  Eigen::Matrix4d a_pose = a.PoseInMatrix();
  Eigen::Matrix4d b_pose = b.PoseInMatrix();
  Eigen::Matrix4d target_pose = Eigen::Matrix4d::Identity();

  Eigen::Vector3d delta_translation =
      (b_pose.block(0, 3, 3, 1) - a_pose.block(0, 3, 3, 1)) * factor;
  target_pose.block(0, 3, 3, 1) = a_pose.block(0, 3, 3, 1) + delta_translation;

  Eigen::Quaterniond q_a = a.RotationInMatrix();
  Eigen::Quaterniond q_b = b.RotationInMatrix();
  Eigen::Quaterniond q_delta = q_a.slerp(factor, q_b);
  target_pose.block(0, 0, 3, 3) = q_delta.toRotationMatrix();

  data.SetPose(target_pose);
  return data;
}

template <typename MsgTypePtr>
std::pair<int, int> TimeStampBinarySearch(const std::vector<MsgTypePtr>& msgs,
                                          const SimpleTime& time) {
  int mid;
  int start = 0;
  int end = msgs.size() - 1;
  while (end - start > 1) {
    mid = start + (end - start) / 2;
    if (time < msgs.at(mid)->header.stamp) {
      end = mid;
    } else {
      start = mid;
    }
  }
  return std::make_pair(start, end);
}

bool MapBuilder::GetOdomAtTime(const SimpleTime& time, sensors::OdomMsg* odom,
                               double threshold_in_sec) {
  CHECK(odom);
  if (threshold_in_sec < 1.e-6) {
    threshold_in_sec = 1.e-6;
  }
  sensors::OdomMsg::Ptr former_data;
  sensors::OdomMsg::Ptr latter_data;
  {
    common::MutexLocker locker(&mutex_);
    // size == 0
    if (odom_msgs_.empty()) {
      PRINT_WARNING("no odom data.");
      return false;
    }
    // size == 1
    if (odom_msgs_.size() == 1) {
      if (std::fabs(time.toSec() - odom_msgs_[0]->header.stamp.toSec()) <=
          threshold_in_sec) {
        *odom = *odom_msgs_[0];
        return true;
      } else {
        return false;
      }
    }
    // size >= 2
    if (time < odom_msgs_.front()->header.stamp ||
        time > odom_msgs_.back()->header.stamp) {
      PRINT_WARNING("too old or too new.");
      return false;
    }

    // binary search for the time period for the target time
    auto indices = TimeStampBinarySearch(odom_msgs_, time);
    former_data = odom_msgs_[indices.first];
    latter_data = odom_msgs_[indices.second];
  }

  CHECK(time >= former_data->header.stamp && time <= latter_data->header.stamp);
  // interpolate the data for more accurate odom data
  *odom = InterpolateOdom(*former_data, *latter_data, time);
  return true;
}

bool MapBuilder::GetUtmAtTime(const SimpleTime& time, sensors::UtmMsg* utm,
                              double threshold_in_sec) {
  CHECK(utm);
  if (threshold_in_sec < 1.e-6) {
    threshold_in_sec = 1.e-6;
  }
  sensors::UtmMsg::Ptr former_data;
  sensors::UtmMsg::Ptr latter_data;
  {
    common::MutexLocker locker(&mutex_);
    // size == 0
    if (utm_msgs_.empty()) {
      PRINT_WARNING("no utm data.");
      return false;
    }
    // size == 1
    if (utm_msgs_.size() == 1) {
      if (std::fabs(time.toSec() - utm_msgs_[0]->header.stamp.toSec()) <=
          threshold_in_sec) {
        *utm = *utm_msgs_[0];
        return true;
      } else {
        return false;
      }
    }
    // size >= 2
    if (time < utm_msgs_.front()->header.stamp ||
        time > utm_msgs_.back()->header.stamp) {
      PRINT_WARNING("too old or too new.");
      return false;
    }

    // binary search for the time period for the target time
    auto indices = TimeStampBinarySearch(utm_msgs_, time);
    former_data = utm_msgs_.at(indices.first);
    latter_data = utm_msgs_.at(indices.second);
  }

  CHECK(time >= former_data->header.stamp && time <= latter_data->header.stamp);
  if (latter_data->header.stamp.toSec() - former_data->header.stamp.toSec() >
      1.) {
    return false;
  }

  float factor =
      (time - former_data->header.stamp).toSec() /
      (latter_data->header.stamp - former_data->header.stamp).toSec();
  CHECK(factor >= 0. && factor <= 1.);

  utm->x = former_data->x + factor * (latter_data->x - former_data->x);
  utm->y = former_data->y + factor * (latter_data->y - former_data->y);
  utm->z = former_data->z + factor * (latter_data->z - former_data->z);
  return true;
}

void MapBuilder::SubmapPairMatch(const int source_index,
                                 const int target_index) {
  std::shared_ptr<Submap<PointType>> target_submap, source_submap;
  target_submap = current_trajectory_->at(target_index);
  source_submap = current_trajectory_->at(source_index);
  target_submap->ClearCloudInFrames();

  // init back end(submap to submap matcher)
  std::shared_ptr<registrator::Interface<PointType>> matcher;
  auto& submap_matcher_options =
      options_.back_end_options.submap_matcher_options;
  switch (submap_matcher_options.type) {
    case registrator::kIcpPM:
      matcher =
          std::make_shared<registrator::IcpUsingPointMatcher<PointType>>();
      break;
    case registrator::kLibicp:
      matcher = std::make_shared<IcpUsingLibicp<PointType>>();
      break;
    case registrator::kNdtWithGicp:
      matcher = std::make_shared<NdtWithGicp<PointType>>(
          submap_matcher_options.use_voxel_filter,
          submap_matcher_options.voxel_filter_resolution);
      dynamic_cast<NdtWithGicp<PointType>*>(matcher.get())
          ->enableNdt(submap_matcher_options.enable_ndt);
      break;

    default:
      PRINT_ERROR("Wrong type");
      return;
  }

  matcher->setInputSource(source_submap->Cloud());
  matcher->setInputTarget(target_submap->Cloud());
  Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f guess =
      target_submap->GetFrames()[0]->GlobalPose().inverse() *
      source_submap->GetFrames()[0]->GlobalPose();
  matcher->align(guess, result);
  common::NormalizeRotation(result);
  double submap_match_score = matcher->getFitnessScore();
  // PRINT_DEBUG_FMT("submap match score: %lf", submap_match_score);
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
  // finish mens that its global pose is ready
  int current_finished_index = 0;
  std::vector<std::shared_ptr<Submap<PointType>>> submaps_to_connect;
  submaps_to_connect.reserve(20);
  bool first_inserted = false;
  while (true) {
    int submap_size = current_trajectory_->size();
    if (submap_size == 0 || submap_size - 1 == current_finished_index) {
      if (submap_processing_done_.load()) {
        break;
      }
      SimpleTime::from_sec(0.1).sleep();
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
      SimpleTime::from_sec(0.1).sleep();
      continue;
    }
    for (size_t i = 1; i < submaps_to_connect.size(); ++i) {
      submaps_to_connect[i]->SetGlobalPose(
          submaps_to_connect[i - 1]->GlobalPose() *
          submaps_to_connect[i - 1]->TransformToNext());
      submaps_to_connect[i]->SetTransformFromLast(
          submaps_to_connect[i - 1]->TransformToNext());
      current_finished_index++;

      isam_optimizer_->AddFrame(
          submaps_to_connect[i],
          submaps_to_connect[i]->match_score_to_previous_submap_);
    }
  }

  // for last submap
  // make sure that all submap output to file if need
  if (!current_trajectory_->empty()) {
    current_trajectory_->back()->SetMatchedTransformedToNext(
        Eigen::Matrix4f::Identity());
  }
  PRINT_INFO("All submaps have been connected.");

  isam_optimizer_->RunFinalOptimazation();
  switch (options_.whole_options.odom_calib_mode) {
    case kNoCalib:
      PRINT_INFO("Do no calibration.");
      break;
    case kOnlineCalib:
      PRINT_INFO("Update tf(odom->lidar) from online calibration.");
      transform_odom_lidar_ = isam_optimizer_->GetTransformOdomToLidar();
      break;
    case kOfflineCalib:
      PRINT_INFO("Update tf(odom->lidar) from offline calibration.");
      OfflineCalibrationOdomToLidar();
      break;
    default:
      PRINT_ERROR("Unknown Calibration Mode.");
      break;
  }

  // all frame connected
  // generate results (clouds to pcd files)
  // output the path into pointcloud as a pcd file
  for (auto& submap : *current_trajectory_) {
    submap->UpdateInnerFramePose();
  }
  // clear all source clouds
  for (auto& frame : point_clouds_) {
    frame->points.clear();
    frame->points.shrink_to_fit();
  }
  point_clouds_.clear();
  point_clouds_.shrink_to_fit();

  // calculate the coord transform from the map th utm
  CalculateCoordTransformToUtm();
  // path to pcd file after rotation into utm corrdinate
  OutputPath();
  // @todo add a paramter for map filename
  GenerateMapPackage(options_.whole_options.map_package_path + "map.xml");

  if (options_.map_package_options.enable) {
    SaveMapPackage();
  } else {
    // output the whole map instead of seperated
    MultiResolutionVoxelMap<PointType> map;
    map.Initialise(options_.output_mrvm_settings);
    PointCloudPtr output_cloud(new PointCloudType);
    const int submaps_size = current_trajectory_->size();
    for (auto& submap : *current_trajectory_) {
      output_cloud->clear();
      pcl::transformPointCloud(*(submap->Cloud()), *output_cloud,
                               submap->GlobalPose());

      PRINT_DEBUG_FMT("submap index: %d / %d", submap->GetId().submap_index,
                      submaps_size - 1);
      Eigen::Vector3d translation = submap->GlobalTranslation().cast<double>();

      start_clock();
      map.InsertPointCloud(output_cloud, translation.cast<float>());
      end_clock(__FILE__, __FUNCTION__, __LINE__);
      submap->ClearCloud();
    }
    PRINT_INFO("creating the whole static map ...");
    map.OutputToPointCloud(
        options_.output_mrvm_settings.prob_threshold,
        options_.whole_options.export_file_path + "static_map.pcd");
  }
  end_managing_memory_ = true;
}

void MapBuilder::OutputPath() {
  int path_point_index = 0;
  std::ofstream path_text_file(options_.whole_options.export_file_path +
                               "path.csv");
  std::string path_content;
  bool write_to_text = path_text_file.is_open();
  pcl::PointCloud<pcl::_PointXYZI>::Ptr path_cloud(
      new pcl::PointCloud<pcl::_PointXYZI>);
  for (auto& submap : *current_trajectory_) {
    for (auto& frame : submap->GetFrames()) {
      pcl::_PointXYZI path_point;
      Eigen::Matrix4d pose = frame->GlobalPose().cast<double>();
      Eigen::Vector6<double> utm_pose_6d = common::TransformToVector6(pose);
      path_point.x = utm_pose_6d[0];
      path_point.y = utm_pose_6d[1];
      path_point.z = utm_pose_6d[2];
      path_point.intensity = path_point_index;

      path_cloud->push_back(path_point);
      if (write_to_text) {
        path_content += (std::to_string(path_point_index) + ", ");
        path_content += (std::to_string(utm_pose_6d[0]) + ", ");
        path_content += (std::to_string(utm_pose_6d[1]) + ", ");
        path_content += (std::to_string(utm_pose_6d[2]) + ", ");
        path_content += (std::to_string(utm_pose_6d[3]) + ", ");
        path_content += (std::to_string(utm_pose_6d[4]) + ", ");
        path_content += (std::to_string(utm_pose_6d[5]) + " \n");
      }
      path_point_index++;
    }
  }
  if (!path_cloud->empty()) {
    pcl::io::savePCDFileBinaryCompressed(
        options_.whole_options.export_file_path + "path.pcd", *path_cloud);
  }
  if (write_to_text) {
    path_text_file << path_content;
    path_text_file.close();
  }
}

void MapBuilder::SubmapMemoryManaging() {
  if (!options_.back_end_options.submap_options.enable_disk_saving) {
    PRINT_INFO("No need to manage submap memory, exit the thread.");
    return;
  }

  const int time = 1;
  while (true) {
    if (end_managing_memory_) {
      break;
    }

    // int submap_size = current_trajectory_->size();
    int active_submap_count = 0;
    for (auto& submap : *current_trajectory_) {
      if (submap->UpdateInactiveTime(time)) {
        active_submap_count++;
      }
    }
    SimpleTime::from_sec(static_cast<double>(time)).sleep();
    // PRINT_INFO_FMT("Active submaps %d in all %d submaps",
    // active_submap_count,
    //                submap_size);
  }
  PRINT_INFO("End managing memory.");
}

void MapBuilder::SubmapProcessing() {
  current_trajectory_->reserve(kSubmapResSize);
  auto& submap_options = options_.back_end_options.submap_options;
  const int submap_frame_count = submap_options.frame_count;

  size_t current_index = 0;
  std::vector<std::shared_ptr<Frame<PointType>>> local_frames;
  common::ThreadPool submap_match_thread_pool(6);
  // there is a deamon thread in this thread pool
  // it is to connect all submap into a global map
  // others are for submap matching
  submap_match_thread_pool.enqueue([&]() { ConnectAllSubmap(); });
  submap_match_thread_pool.enqueue([&]() { SubmapMemoryManaging(); });
  size_t frames_size = 0;
  while (true) {
    {
      common::MutexLocker locker(&mutex_);
      frames_size = frames_.size();
    }
    if (frames_size - current_index < submap_frame_count) {
      if (!scan_match_thread_running_) {
        PRINT_INFO("no enough frames for new submap, quit");
        break;
      }
      SimpleTime::from_sec(0.5).sleep();
      continue;
    }

    for (int i = 0; i < submap_frame_count; ++i) {
      local_frames.push_back(frames_[current_index]);
      current_index++;
    }

    // Adding new submap
    auto submap = std::make_shared<Submap<PointType>>(submap_options);
    SubmapId current_id;
    current_id.trajectory_index = current_trajectory_->GetId();
    auto& current_submap_index = current_id.submap_index;
    current_submap_index = current_trajectory_->size();
    submap->SetId(current_id);
    submap->SetSavePath(options_.whole_options.map_package_path);
    current_trajectory_->push_back(submap);
    // create a submap and init with the configs
    PRINT_DEBUG_FMT("Add a new submap : %d", current_submap_index);

    for (auto& frame : local_frames) {
      submap->InsertFrame(frame);
    }
    CHECK(submap->Full());
    submap->CalculateDescriptor();

    if (use_gps_) {
      sensors::UtmMsg utm;
      if (GetUtmAtTime(submap->GetTimeStamp(), &utm)) {
        submap->SetRelatedUtm(UtmPosition(utm.x, utm.y, utm.z));
      }
    }
    if (use_odom_) {
      sensors::OdomMsg odom;
      if (GetOdomAtTime(submap->GetTimeStamp(), &odom)) {
        submap->SetRelatedOdom(odom.PoseInMatrix().cast<double>());
      }
    }

    local_frames.clear();
    if (show_submap_function_) {
      show_submap_function_(submap->Cloud());
    }
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
  // output some file
  if (!odom_path_.empty()) {
    pcl::PointCloud<pcl::PointXYZ> odom_path_cloud;
    for (Eigen::Vector3d& odom_path_point : odom_path_) {
      odom_path_cloud.push_back(pcl::PointXYZ(
          odom_path_point[0], odom_path_point[1], odom_path_point[2]));
    }
    PRINT_INFO("Generated ODOM path file.");
    pcl::io::savePCDFileBinaryCompressed(
        options_.whole_options.export_file_path + "odom_path.pcd",
        odom_path_cloud);
  }
  if (!utm_path_.empty()) {
    Eigen::Vector4d init_point = utm_path_[0];
    pcl::PointCloud<pcl::PointXYZI> utm_path_cloud;
    for (Eigen::Vector4d& path_point : utm_path_) {
      pcl::PointXYZI point;
      point.x = path_point[0] - init_point[0];
      point.y = path_point[1] - init_point[1];
      point.z = path_point[2] - init_point[2];
      point.intensity = path_point[3];
      utm_path_cloud.push_back(point);
    }
    pcl::io::savePCDFileBinaryCompressed(
        options_.whole_options.export_file_path + "utm_path.pcd",
        utm_path_cloud);
  }

  size_t remaining_pointcloud_count = point_clouds_.size();
  int delay = 0;
  SimpleTime delay_time = SimpleTime::from_sec(0.1);
  if (scan_match_thread_) {
    do {
      delay_time.sleep();
      delay++;
      if (delay >= 20) {
        std::ostringstream progress_info;
        progress_info << "Remaining Point Cloud : "
                      << (1. - static_cast<double>(point_clouds_.size()) /
                                   remaining_pointcloud_count) *
                             100.
                      << "% ...";
        PRINT_COLOR_FMT(BOLD, "%s", progress_info.str().c_str());
        delay = 0;
      }
    } while (scan_match_thread_running_);

    scan_match_thread_->join();
  }

  if (submap_thread_ && submap_thread_->joinable()) {
    submap_thread_->join();
    submap_thread_.reset();
  }
}

void MapBuilder::OfflineCalibrationOdomToLidar() {
  if (!use_odom_) {
    return;
  }
  const int submap_size = current_trajectory_->size();
  if (submap_size <= 1) {
    PRINT_WARNING("too few submaps to calculate the transfrom");
    return;
  }

  std::vector<Eigen::Vector3d> map_path_positions;
  std::vector<Eigen::Vector3d> map_path_directions;
  std::vector<OdomPose> odom_poses;

  Eigen::Matrix4f init_estimate = transform_odom_lidar_;
  PointCloudType path_and_odom_cloud;
  for (auto& submap : *current_trajectory_) {
    if (!submap->HasOdom()) {
      continue;
    }

    PointType path_point;
    Eigen::Vector3d path_position = submap->GlobalTranslation().cast<double>();
    path_point.x = path_position[0];
    path_point.y = path_position[1];
    path_point.z = path_position[2];
    path_point.intensity = 1;
    map_path_positions.push_back(path_position);

    // refer to
    // https://stackoverflow.com/questions/1568568/how-to-convert-euler-angles-to-directional-vector
    // get the directional vector of a rotation matrix
    Eigen::Matrix3d path_roation = submap->GlobalRotation().cast<double>();
    Eigen::Vector3d eulers = common::RotationMatrixToEulerAngles(path_roation);
    Eigen::Vector3d direction(std::cos(eulers[2]) * std::cos(eulers[1]),
                              std::sin(eulers[2]) * std::cos(eulers[1]),
                              std::sin(eulers[1]));
    map_path_directions.push_back(direction);

    OdomPose odom_pose = submap->GetRelatedOdom();
    odom_poses.push_back(odom_pose);
    PointType odom_point;
    odom_point.x = odom_pose(0, 3);
    odom_point.y = odom_pose(1, 3);
    odom_point.z = odom_pose(2, 3);
    odom_point.intensity = 2;

    path_and_odom_cloud.points.push_back(path_point);
    path_and_odom_cloud.points.push_back(odom_point);
  }
  if (!path_and_odom_cloud.empty()) {
    pcl::io::savePCDFileBinaryCompressed(
        options_.whole_options.export_file_path + "path_and_odom_before.pcd",
        path_and_odom_cloud);
  }

  if (map_path_positions.empty()) {
    return;
  }

  const int size = map_path_positions.size();
  // @todo add a init estimate based-on tf_odom_lidar
  Eigen::Vector6<float> init_estimate_6d =
      common::TransformToVector6(init_estimate);
  double t[] = {init_estimate_6d[0], init_estimate_6d[1], init_estimate_6d[2]};
  double r[] = {init_estimate_6d[3], init_estimate_6d[4], init_estimate_6d[5]};
  ceres::Problem problem;
  for (int i = 0; i < size; ++i) {
    auto cost_function = cost_functions::OdomToMapPath::Create(
        map_path_positions[i], odom_poses[i], map_path_directions[i]);
    problem.AddResidualBlock(cost_function, new ceres::HuberLoss(1.0), t, r);
  }
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 100;
  options.num_threads = 8;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  Eigen::Vector3d translation(t[0], t[1], t[2]);
  Eigen::Matrix3d rotation =
      common::EulerAnglesToRotationMatrix(Eigen::Vector3d(r[0], r[1], r[2]));
  PointCloudType path_and_odom_cloud_after;
  for (int i = 0; i < size; ++i) {
    PointType path_point;
    Eigen::Vector3d path_position = map_path_positions[i];
    path_point.x = path_position[0];
    path_point.y = path_position[1];
    path_point.z = path_position[2];
    path_point.intensity = 1;

    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block(0, 0, 3, 3) = rotation;
    transform.block(0, 3, 3, 1) = translation;
    Eigen::Matrix4d odom_pose_in_map =
        transform.inverse() * odom_poses[i] * transform;
    PointType odom_point;
    odom_point.x = odom_pose_in_map(0, 3);
    odom_point.y = odom_pose_in_map(1, 3);
    odom_point.z = odom_pose_in_map(2, 3);
    odom_point.intensity = 2;

    path_and_odom_cloud_after.points.push_back(path_point);
    path_and_odom_cloud_after.points.push_back(odom_point);
  }
  if (!path_and_odom_cloud_after.empty()) {
    pcl::io::savePCDFileBinaryCompressed(
        options_.whole_options.export_file_path + "path_and_odom_after.pcd",
        path_and_odom_cloud_after);
  }

  // update the tf connection
  transform_odom_lidar_.block<3, 3>(0, 0) = rotation.cast<float>();
  transform_odom_lidar_.block<3, 1>(0, 3) = translation.cast<float>();
  PRINT_INFO("odom -> lidar match result :");
  common::PrintTransform(transform_odom_lidar_);
}

void MapBuilder::CalculateCoordTransformToUtm() {
  if (!use_gps_) {
    return;
  }

  Eigen::Matrix4d result = isam_optimizer_->GetGpsCoordTransfrom();
  map_utm_rotation_ = result.block(0, 0, 3, 3);
  map_utm_translation_ = result.block(0, 3, 3, 1);
  if (utm_init_offset_) {
    map_utm_translation_ += utm_init_offset_.value();
  }
  // output the result to file( txt or xml ).
  PRINT_INFO_FMT("utm translation: %.12lf, %.12lf", map_utm_translation_[0],
                 map_utm_translation_[1]);

  // update the submap pose and frame pose
  Eigen::Matrix4d map_utm_transform = Eigen::Matrix4d::Identity();
  map_utm_transform.block<3, 3>(0, 0) = map_utm_rotation_;
  for (auto& submap : *current_trajectory_) {
    Eigen::Matrix4d new_submap_pose =
        map_utm_transform * submap->GlobalPose().cast<double>();
    submap->SetGlobalPose(new_submap_pose.cast<float>());
    submap->UpdateInnerFramePose();
  }
  current_trajectory_->SetUtmOffset(map_utm_translation_[0],
                                    map_utm_translation_[1]);
}

void MapBuilder::DownSamplePointcloud(const PointCloudPtr& source,
                                      const PointCloudPtr& output) {
  filter_factory_.SetInputCloud(source);
  filter_factory_.Filter(output);
}

void MapBuilder::SetShowMapFunction(const MapBuilder::ShowMapFunction& func) {
  show_map_function_ = std::move(func);
}

void MapBuilder::SetShowPoseFunction(const MapBuilder::ShowPoseFunction& func) {
  show_pose_function_ = std::move(func);
}

void MapBuilder::SetShowSubmapFunction(
    const MapBuilder::ShowMapFunction& func) {
  show_submap_function_ = std::move(func);
}

void MapBuilder::EnableUsingOdom(bool flag) {
  use_odom_ = flag;
  std::string str = flag ? "enable" : "disable";
  PRINT_INFO_FMT("%s odom.", str.c_str());
  if (flag) {
    odom_msgs_.reserve(kOdomMsgMaxSize);
  }
}

void MapBuilder::EnableUsingGps(bool flag) {
  use_gps_ = flag;
  std::string str = flag ? "enable" : "disable";
  PRINT_INFO_FMT("%s gps.", str.c_str());
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
  // step1. calculate the bbox
  double min_x = 1.e50;
  double max_x = -1.e50;
  double min_y = 1.e50;
  double max_y = -1.e50;
  for (auto& single_trajectory : trajectories_) {
    for (auto& submap : *single_trajectory) {
      Eigen::Vector3f position = submap->GlobalTranslation();
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

  const double offset = options_.map_package_options.border_offset;
  min_x -= offset;
  min_y -= offset;
  max_x += offset;
  max_y += offset;

  // step2. seperate the map
  const double half_width = options_.map_package_options.piece_width * 0.5;
  int x_steps = (max_x - min_x) / half_width;
  int y_steps = (max_y - min_y) / half_width;
  if (x_steps < 0 || y_steps < 0) {
    PRINT_WARNING("No good bounding box, save no map package.");
    return;
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
  const Eigen::Vector2d part_offset(offset, offset);
  SeperatedPart parts[x_steps][y_steps];
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
      for (auto& trajectory : trajectories_) {
        for (auto& submap : *trajectory) {
          Eigen::Vector3d position = submap->GlobalTranslation().cast<double>();
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
      MultiResolutionVoxelMap<PointType> voxel_map;
      voxel_map.Initialise(options_.output_mrvm_settings);
      for (auto& submap : part.inside_submaps) {
        PointCloudPtr transformed_cloud(new PointCloudType);
        Eigen::Matrix4f pose = submap->GlobalPose();
        Eigen::Vector3f translation = submap->GlobalTranslation();
        pcl::transformPointCloud(*(submap->Cloud()), *transformed_cloud, pose);
        PRINT_DEBUG_FMT("submap in piece[%d][%d] : %d / %d", x, y, i,
                        submaps_size - 1);
        if (inside_bbox(translation.cast<double>(), part.bb_min, part.bb_max)) {
          start_clock();
          voxel_map.InsertPointCloud(transformed_cloud, translation);
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
            voxel_map.InsertPointCloud(transformed_cloud_in_bbox, translation);
          }
          end_clock(__FILE__, __FUNCTION__, __LINE__);
        }
        ++i;
      }

      PointCloudPtr whole_part_cloud(new PointCloudType);
      voxel_map.OutputToPointCloud(options_.output_mrvm_settings.prob_threshold,
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
      std::string filename = options_.map_package_options.cloud_file_prefix +
                             std::to_string(x) + "_" + std::to_string(y) +
                             ".pcd";
      pcl::io::savePCDFileBinaryCompressed(
          options_.whole_options.export_file_path + filename, *part.cloud);
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

      std::string filename = options_.map_package_options.cloud_file_prefix +
                             std::to_string(x) + "_" + std::to_string(y) +
                             ".pcd";
      map_piece_node.append_attribute("file") = filename.c_str();
    }
  }
  std::string filename = options_.whole_options.export_file_path +
                         options_.map_package_options.descript_filename;
  doc.save_file(filename.c_str());
}

}  // namespace static_map
