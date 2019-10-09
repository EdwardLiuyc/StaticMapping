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

// third party
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
// local
#include "back_end/isam_optimizer.h"
#include "back_end/odom_to_pose_factor.h"
#include "common/make_unique.h"
#include "common/math.h"

namespace static_map {
namespace back_end {

using gtsam::between;
using gtsam::compose;
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Pose3_;
using gtsam::Rot3;
namespace NM = gtsam::noiseModel;

// @todo(edward) add a gps coord static bias factor
// it is a static error of the tf connection (tracking to gps)
// @todo(edward) add imu factor

#define POSE_SYMBOL ('p')
#define GPS_COORD_SYMBOL ('g')
#define CALIB_SYMBOL ('c')
#define VELOCITY_SYMBOL ('v')

#define POSE_KEY(index) (gtsam::Symbol(POSE_SYMBOL, (index)))
#define VELOCITY_KEY(index) (gtsam::Symbol(VELOCITY_SYMBOL, (index)))
#define ODOM_CALIB_KEY (gtsam::Symbol(CALIB_SYMBOL, 0))
#define IMU_CALIB_KEY (gtsam::Symbol(CALIB_SYMBOL, 1))
#define GPS_COORD_KEY (gtsam::Symbol(GPS_COORD_SYMBOL, 0))

constexpr int kGpsSkipNum = 20;

template <typename PointT>
IsamOptimizer<PointT>::IsamOptimizer(const IsamOptimizerOptions &options,
                                     const LoopDetectorSettings &l_d_setting)
    : isam_factor_graph_(new gtsam::NonlinearFactorGraph),
      loop_detector_(l_d_setting),
      options_(options) {
  gtsam::ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  gtsam::ISAM2DoglegParams dogleg_param;
  dogleg_param.setVerbose(false);
  parameters.setOptimizationParams(dogleg_param);
  isam_ = common::make_unique<gtsam::ISAM2>(parameters);

  prior_noise_model_ = NM::Isotropic::Sigma(6, 1.e-3);
  gps_noise_model_ = NM::Isotropic::Sigma(3, 0.15);
  frame_match_noise_model_ = NM::Diagonal::Sigmas(
      (gtsam::Vector(6) << 0.15, 0.15, 0.1, 0.15, 0.15, 0.15).finished());
  loop_closure_noise_model_ = NM::Diagonal::Sigmas(
      (gtsam::Vector(6) << 0.15, 0.15, 0.1, 0.15, 0.15, 0.15).finished());
  odom_tf_noise_model_ = NM::Diagonal::Sigmas(
      (gtsam::Vector(6) << 0.5, 0.5, 1.5, 0.1, 0.1, 0.1).finished());
  odom_noise_model_ = NM::Robust::Create(
      NM::mEstimator::Huber::Create(1),
      NM::Diagonal::Sigmas(
          (gtsam::Vector(6) << 0.2, 0.2, 0.2, 1.5, 1.5, 2).finished()));
}

template <typename PointT>
void IsamOptimizer<PointT>::IsamUpdate(const int update_time) {
  CHECK_GE(update_time, 1);
  isam_->update(*isam_factor_graph_, initial_estimate_);
  for (int i = 0; i < update_time; ++i) {
    isam_->update();
  }
  isam_factor_graph_->resize(0);
  initial_estimate_.clear();
}

template <typename PointT>
gtsam::Values IsamOptimizer<PointT>::UpdateAllPose() {
  gtsam::Values estimate_poses = isam_->calculateBestEstimate();
  const auto &frames = loop_detector_.GetFrames();
  const int frames_size = frames.size();
  for (int i = 0; i < frames_size; ++i) {
    Eigen::Matrix4f pose =
        estimate_poses.at<gtsam::Pose3>(POSE_KEY(i)).matrix().cast<float>();
    frames[i]->SetGlobalPose(pose);
  }

  return estimate_poses;
}

template <typename PointT>
void IsamOptimizer<PointT>::AddLoopCloseEdge(
    const int target_index, const int source_index,
    const Eigen::Matrix4f &transform_tgt_to_src,
    const NM::Base::shared_ptr &loop_close_noise) {
  isam_factor_graph_->addExpressionFactor(
      loop_close_noise, Pose3(transform_tgt_to_src.cast<double>()),
      between(Pose3_(POSE_KEY(target_index)), Pose3_(POSE_KEY(source_index))));

  auto &frames = loop_detector_.GetFrames();
  frames[target_index]->AddConnectedSubmap(frames[source_index]->GetId());
  view_graph_.AddEdge(target_index, source_index, transform_tgt_to_src);
}

template <typename PointT>
void IsamOptimizer<PointT>::AddVertex(
    const int &index, const Eigen::Matrix4f &pose,
    const Eigen::Matrix4f &transform_from_last_pose,
    const NM::Base::shared_ptr &odom_noise) {
  auto &frames = loop_detector_.GetFrames();
  gtsam::Pose3 pose_gtsam = gtsam::Pose3(pose.cast<double>());
  gtsam::Pose3 pose_from_gtsam =
      gtsam::Pose3(transform_from_last_pose.cast<double>());
  initial_estimate_.insert(POSE_KEY(index), pose_gtsam);
  view_graph_.AddVertex(index, pose);
  if (index == 0) {
    isam_factor_graph_->addExpressionFactor(prior_noise_model_, pose_gtsam,
                                            Pose3_(POSE_KEY(index)));
    if (options_.use_odom) {
      Eigen::Vector6<float> tf_6d =
          common::TransformToVector6(tf_odom_lidar_.inverse().eval());
      Pose3 odom_tf = Pose3(Rot3::RzRyRx(tf_6d[3], tf_6d[4], tf_6d[5]),
                            Point3(tf_6d[0], tf_6d[1], tf_6d[2]));
      initial_estimate_.insert(ODOM_CALIB_KEY, odom_tf);
      isam_factor_graph_->addExpressionFactor(odom_tf_noise_model_, odom_tf,
                                              Pose3_(ODOM_CALIB_KEY));
      calib_factor_inserted_ = true;
    }
    if (options_.use_gps) {
      initial_estimate_.insert(GPS_COORD_KEY, gps_coord_transform_);
      isam_factor_graph_->addExpressionFactor(
          NM::Diagonal::Sigmas(
              (gtsam::Vector(6) << 0.5, 0.5, 1.57, 2, 2, 2.).finished()),
          gps_coord_transform_, Pose3_(GPS_COORD_KEY));
    }

  } else {
    isam_factor_graph_->addExpressionFactor(
        odom_noise, pose_from_gtsam,
        between(Pose3_(POSE_KEY(index - 1)), Pose3_(POSE_KEY(index))));

    frames[index - 1]->AddConnectedSubmap(frames[index]->GetId());
    view_graph_.AddEdge(index - 1, index, transform_from_last_pose);
  }

  IsamUpdate();
}

template <typename PointT>
void IsamOptimizer<PointT>::AddFrame(
    const std::shared_ptr<Submap<PointT>> &frame, const double match_score) {
  CHECK(frame);
  auto result = loop_detector_.AddFrame(frame, true);
  int frame_index = static_cast<int>(loop_detector_.GetFrames().size()) - 1;
  CHECK_EQ(frame_index, result.current_frame_index);
  PRINT_DEBUG_FMT("optimizer inserting a new frame, match score: %lf",
                  match_score);

  AddVertex(result.current_frame_index, frame->GlobalPose(),
            frame->TransformFromLast(), frame_match_noise_model_);

  if (options_.use_odom && frame->HasOdom()) {
    // the factor is connected to one vertex
    // for calibration the tf connection from lidar to odom
    // T.inverse * lidar_pose * T = odom_pose
    // so the cost function (factor)
    // minimise || T.inverse * lidar_pose * T - odom_pose || (l2.norm)
    auto calib_tf = Pose3_(ODOM_CALIB_KEY);
    auto pose = Pose3_(POSE_KEY(result.current_frame_index));
    // compose(a, b) = a * b
    // between(a, b) = a.inverse * b
    // so, the following expression equals to this :
    // calib_tf.inverse * pose * calib_tf
    auto uncalibrated = compose(between(calib_tf, pose), calib_tf);
    isam_factor_graph_->addExpressionFactor(
        odom_noise_model_, Pose3(frame->GetRelatedOdom()), uncalibrated);
  }

  if (options_.use_gps) {
    if (frame->HasUtm()) {
      // expression :
      // map_origin_in_gps * map_pose * tracking_to_gps = gps pose
      // but gps has no rotation since it only provides longtitude, latitude ...
      // so we use only translation in the tracking_to_gps
      auto map_origin_in_gps = Pose3_(GPS_COORD_KEY);
      auto pose = Pose3_(POSE_KEY(result.current_frame_index));
      const gtsam::Point3 tracking_gps_translation(
          tf_tracking_gps_.block(0, 3, 3, 1).cast<double>());
      isam_factor_graph_->addExpressionFactor(
          gps_noise_model_, gtsam::Point3(frame->GetRelatedUtm()),
          gtsam::transform_from(gtsam::compose(map_origin_in_gps, pose),
                                gtsam::Point3_(tracking_gps_translation)));
      accumulated_gps_count_++;
      IsamUpdate();
    } else {
      PRINT_WARNING("No Gps related.");
    }
  }

  // try to close loop and add constraint
  if (result.close_succeed) {
    const int edge_size = result.close_pair.size();
    for (int i = 0; i < edge_size; ++i) {
      AddLoopCloseEdge(result.close_pair[i].first, result.close_pair[i].second,
                       result.transform[i], loop_closure_noise_model_);
      IsamUpdate();
    }
    PRINT_INFO_FMT(BOLD "Add %d loop closure edges." NONE_FORMAT, edge_size);
  }

  IsamUpdate();
  CHECK(isam_->valueExists(POSE_KEY(frame_index)));
  gtsam::Values estimate_poses = isam_->calculateEstimate();
  Eigen::Matrix4f current_pose =
      estimate_poses.at<gtsam::Pose3>(POSE_KEY(frame_index))
          .matrix()
          .cast<float>();
  frame->SetGlobalPose(current_pose);
  view_graph_.AddVertex(frame_index, current_pose);
  PRINT_DEBUG("update pose.");
  // if (options_.use_gps &&
  //     accumulated_gps_count_ % kGpsSkipNum == kGpsSkipNum - 1) {
  //   UpdateAllPose();
  // } else {
  //   isam_->calculateEstimate();
  // }
}

template <typename PointT>
void IsamOptimizer<PointT>::RunFinalOptimazation() {
  IsamUpdate();
  gtsam::Values estimate_poses = UpdateAllPose();

  // @todo(edward) no static directory
  view_graph_.SaveTextFile("pcd/graph.txt");
  view_graph_.SaveImage("pcd/graph.jpg");

  if (options_.use_odom && calib_factor_inserted_) {
    // update the calibration result
    tf_odom_lidar_ = estimate_poses.at<gtsam::Pose3>(ODOM_CALIB_KEY)
                         .matrix()
                         .inverse()
                         .cast<float>();
    PRINT_INFO("odom calibration result (odom->lidar) : ");
    common::PrintTransform(tf_odom_lidar_);
  }
}

template <typename PointT>
Eigen::Matrix4d IsamOptimizer<PointT>::GetGpsCoordTransfrom() {
  if (options_.use_gps) {
    gtsam::Values estimate_poses = isam_->calculateEstimate();
    if (estimate_poses.exists<gtsam::Pose3>(GPS_COORD_KEY)) {
      gps_coord_transform_ = estimate_poses.at<gtsam::Pose3>(GPS_COORD_KEY);
      return gps_coord_transform_.matrix();
    }
  }
  return Eigen::Matrix4d::Identity();
}

template <typename PointT>
void IsamOptimizer<PointT>::SetTransformOdomToLidar(const Eigen::Matrix4f &t) {
  tf_odom_lidar_ = t;
  loop_detector_.SetTransformOdomToLidar(t);
}

template <typename PointT>
void IsamOptimizer<PointT>::SetTrackingToGps(const Eigen::Matrix4f &t) {
  tf_tracking_gps_ = t;
}

template <typename PointT>
Eigen::Matrix4f IsamOptimizer<PointT>::GetTransformOdomToLidar() {
  return tf_odom_lidar_;
}

template class IsamOptimizer<pcl::PointXYZI>;

}  // namespace back_end
}  // namespace static_map

#undef POSE_SYMBOL
#undef GPS_COORD_SYMBOL
#undef CALIB_SYMBOL
#undef VELOCITY_SYMBOL

#undef POSE_KEY
#undef VELOCITY_KEY
#undef ODOM_CALIB_KEY
#undef GPS_COORD_KEY
