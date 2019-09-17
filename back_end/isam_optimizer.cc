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

#define POSE_SYMBOL ('p')
#define CALIB_SYMBOL ('c')

#define POSE_KEY(index) (gtsam::Symbol(POSE_SYMBOL, (index)))
#define ODOM_CALIB_KEY (gtsam::Symbol(CALIB_SYMBOL, 0))

template <typename PointT>
IsamOptimizer<PointT>::IsamOptimizer(const IsamOptimizerOptions &options,
                                     const LoopDetectorSettings &l_d_setting)
    : isam_factor_graph_(new gtsam::ExpressionFactorGraph),
      loop_detector_(l_d_setting),
      options_(options) {
  gtsam::ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  gtsam::ISAM2DoglegParams dogleg_param;
  dogleg_param.setVerbose(false);
  parameters.setOptimizationParams(dogleg_param);
  isam_ = common::make_unique<gtsam::ISAM2>(parameters);

  prior_noise_model_ = NM::Diagonal::Sigmas(
      (gtsam::Vector(6) << 0.2, 0.2, 0.2, 0.01, 0.01, 0.01).finished());
  frame_match_noise_model_ = NM::Diagonal::Sigmas(
      (gtsam::Vector(6) << 0.05, 0.05, 0.1, 0.1, 0.1, 0.1).finished());
  loop_closure_noise_model_ = NM::Diagonal::Sigmas(
      (gtsam::Vector(6) << 0.05, 0.05, 0.1, 0.1, 0.1, 0.1).finished());
  odom_tf_noise_model_ = NM::Diagonal::Sigmas(
      (gtsam::Vector(6) << 0.5, 0.5, 1.5, 0.1, 0.1, 0.1).finished());
  odom_noise_model_ = NM::Robust::Create(
      NM::mEstimator::Huber::Create(1),
      NM::Diagonal::Sigmas(
          (gtsam::Vector(6) << 0.2, 0.2, 0.2, 1.5, 1.5, 2).finished()));
}

template <typename PointT>
void IsamOptimizer<PointT>::AddLoopCloseEdge(
    const int target_index, const int source_index,
    const Eigen::Matrix4f &transform_tgt_to_src,
    const NM::Base::shared_ptr &loop_close_noise) {
  isam_factor_graph_->addExpressionFactor(
      between(Pose3_(POSE_KEY(target_index)), Pose3_(POSE_KEY(source_index))),
      Pose3(transform_tgt_to_src.cast<double>()), loop_close_noise);

  auto &frames = loop_detector_.GetFrames();
  frames[target_index]->AddConnectedSubmap(frames[source_index]->GetId());

  view_graph_.AddEdge(target_index, source_index, transform_tgt_to_src);
  isam_->update(*isam_factor_graph_);
  isam_->update();
  isam_factor_graph_->resize(0);

  gtsam::Values estimate_poses = isam_->calculateEstimate();
  const int frames_size = frames.size();
  for (int i = 0; i < frames_size; ++i) {
    Eigen::Matrix4f pose =
        estimate_poses.at<gtsam::Pose3>(POSE_KEY(i)).matrix().cast<float>();
    frames[i]->SetGlobalPose(pose);
    view_graph_.AddVertex(i, pose);
  }
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
    isam_factor_graph_->addExpressionFactor(Pose3_(POSE_KEY(index)), pose_gtsam,
                                            prior_noise_model_);
    if (options_.use_odom) {
      Eigen::Vector6<float> tf_6d =
          common::TransformToVector6(tf_odom_lidar_.inverse().eval());
      Pose3 odom_tf = Pose3(Rot3::RzRyRx(tf_6d[3], tf_6d[4], tf_6d[5]),
                            Point3(tf_6d[0], tf_6d[1], tf_6d[2]));
      initial_estimate_.insert(ODOM_CALIB_KEY, odom_tf);
      isam_factor_graph_->addExpressionFactor(Pose3_(ODOM_CALIB_KEY), odom_tf,
                                              odom_tf_noise_model_);
      calib_factor_inserted_ = true;
    }
  } else {
    isam_factor_graph_->addExpressionFactor(
        between(Pose3_(POSE_KEY(index - 1)), Pose3_(POSE_KEY(index))),
        pose_from_gtsam, odom_noise);

    frames[index - 1]->AddConnectedSubmap(frames[index]->GetId());
    view_graph_.AddEdge(index - 1, index, transform_from_last_pose);
  }
  isam_->update(*isam_factor_graph_, initial_estimate_);
  isam_->update();

  isam_factor_graph_->resize(0);
  initial_estimate_.clear();
}

template <typename PointT>
void IsamOptimizer<PointT>::AddFrame(
    const std::shared_ptr<Submap<PointT>> &frame, const double match_score) {
  CHECK(frame);
  auto result = loop_detector_.AddFrame(frame, true);
  int frame_index = static_cast<int>(loop_detector_.GetFrames().size()) - 1;
  CHECK_GE(frame_index, 0);

  // auto odom_noise_score = -std::log(match_score);
  PRINT_DEBUG_FMT("optimizer inserting a new frame, match score: %lf",
                  match_score);

  if (options_.use_odom) {
    if (frame->HasOdom()) {
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
          uncalibrated, Pose3(frame->GetRelatedOdom()), odom_noise_model_);
    }
  }

  AddVertex(result.current_frame_index, frame->GlobalPose(),
            frame->TransformFromLast(), frame_match_noise_model_);
  // try to close loop and add constraint
  if (result.close_succeed) {
    const int edge_size = result.close_pair.size();
    for (int i = 0; i < edge_size; ++i) {
      AddLoopCloseEdge(result.close_pair[i].first, result.close_pair[i].second,
                       result.transform[i], loop_closure_noise_model_);
    }
    PRINT_INFO_FMT(BOLD "Add %d loop closure edges." NONE_FORMAT, edge_size);
  }
  // @todo why esitimating twice ?
  // isam_->calculateEstimate();
  gtsam::Values estimate_poses = isam_->calculateEstimate();
  Eigen::Matrix4f current_pose =
      estimate_poses.at<gtsam::Pose3>(POSE_KEY(frame_index))
          .matrix()
          .cast<float>();
  frame->SetGlobalPose(current_pose);
  view_graph_.AddVertex(frame_index, current_pose);
}

template <typename PointT>
void IsamOptimizer<PointT>::RunFinalOptimazation() {
  view_graph_.SaveTextFile("pcd/graph.txt");
  view_graph_.SaveImage("pcd/graph.jpg");

  if (options_.use_odom && calib_factor_inserted_) {
    gtsam::Values estimate_poses = isam_->calculateEstimate();
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
void IsamOptimizer<PointT>::SetTransformOdomToLidar(const Eigen::Matrix4f &t) {
  tf_odom_lidar_ = t;
  loop_detector_.SetTransformOdomToLidar(t);
}

template <typename PointT>
Eigen::Matrix4f IsamOptimizer<PointT>::GetTransformOdomToLidar() {
  return tf_odom_lidar_;
}

template class IsamOptimizer<pcl::PointXYZI>;

}  // namespace back_end
}  // namespace static_map
