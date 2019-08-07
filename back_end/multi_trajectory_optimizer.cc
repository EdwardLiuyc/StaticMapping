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
#include <gtsam/slam/expressions.h>
// local
#include "back_end/multi_trajectory_optimizer.h"
#include "common/make_unique.h"

namespace static_map {
namespace back_end {

#define SUBMAP_SYMBOL ('s')
#define SUBMAP_KEY(index) (gtsam::Symbol(SUBMAP_SYMBOL, (index)))

// we assume that
// the count of submaps in one single trajectory is less than 2^32-1
// and the count of trajectories is less than 2^32-1
inline uint64_t SubmapIdToUint64(const SubmapId& id) {
  return (((uint64_t)id.trajectory_index) << 32) + (uint64_t)id.submap_index;
}
inline uint32_t SubmapIndexFromUint64(uint64_t index) {
  uint32_t* submap_id = reinterpret_cast<uint32_t*>(&index);
  return *submap_id;
}

namespace NM = gtsam::noiseModel;
using gtsam::between;
using gtsam::compose;
using gtsam::Pose3;
using gtsam::Pose3_;

template <typename PointT>
MultiTrajectoryOptimizer<PointT>::MultiTrajectoryOptimizer(
    const LoopDetectorSettings& l_d_setting)
    : loop_detector_(l_d_setting),
      isam_factor_graph_(new gtsam::ExpressionFactorGraph) {
  gtsam::ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  isam_ = common::make_unique<gtsam::ISAM2>(parameters);

  // init several noise models
  prior_noise_model_ = NM::Diagonal::Sigmas(
      (gtsam::Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished());
  stable_odom_model_ = NM::Robust::Create(
      NM::mEstimator::Huber::Create(1),
      NM::Diagonal::Sigmas(
          (gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01).finished()));
  unstable_odom_model_ =
      NM::Robust::Create(NM::mEstimator::Huber::Create(1),
                         NM::Diagonal::Sigmas((gtsam::Vector(6) << 0.025, 0.025,
                                               0.025, 0.025, 0.025, 0.025)
                                                  .finished()));
  loop_closure_model_ =
      NM::Robust::Create(NM::mEstimator::Huber::Create(1),
                         NM::Diagonal::Sigmas((gtsam::Vector(6) << 0.025, 0.025,
                                               0.025, 0.025, 0.025, 0.025)
                                                  .finished()));
}

template <typename PointT>
MultiTrajectoryOptimizer<PointT>::~MultiTrajectoryOptimizer() {}

template <typename PointT>
void MultiTrajectoryOptimizer<PointT>::AddSubmap(
    const std::shared_ptr<Submap<PointT>>& submap, bool do_loop_detect) {
  const SubmapId id = submap->GetId();
  PRINT_INFO_FMT("Add submap: %s", id.DebugString().c_str());
  const int t_index = id.trajectory_index;
  trajectories_[t_index].push_back(submap);

  auto result = loop_detector_.AddFrame(submap, do_loop_detect);
  AddVertex(SubmapIdToUint64(id), submap->GlobalPose());

  if (result.close_succeed) {
    const int edge_size = result.close_pair.size();
    auto& frames = loop_detector_.GetFrames();
    for (int i = 0; i < edge_size; ++i) {
      const SubmapId target_id = frames[result.close_pair[i].first]->GetId();
      const SubmapId source_id = frames[result.close_pair[i].second]->GetId();
      CHECK_NE(target_id.trajectory_index, source_id.trajectory_index);
      const Eigen::Matrix4f transform = result.transform[i];
      AddSubmapConnection(target_id, source_id, transform, true);
    }
    PRINT_INFO_FMT(BOLD "Add %d loop closure edges." NONE_FORMAT, edge_size);
  }
}

template <typename PointT>
void MultiTrajectoryOptimizer<PointT>::AddSubmapConnection(
    const SubmapId& target, const SubmapId& source,
    const Eigen::Matrix4f& transform, bool is_loop_constraint) {
  PRINT_DEBUG_FMT("target_id : %s, source_id: %s", target.DebugString().c_str(),
                  source.DebugString().c_str());
  if (is_loop_constraint) {
    auto target_submap =
        trajectories_[target.trajectory_index].at(target.submap_index);
    CHECK(target_submap->GetId() == target);
    target_submap->AddConnectedSubmap(source);
  }
  gtsam::Pose3 transform_gtsam = gtsam::Pose3(transform.cast<double>());
  gtsam::noiseModel::Base::shared_ptr noise_model;
  if (is_loop_constraint) {
    noise_model = loop_closure_model_;
  } else {
    noise_model = stable_odom_model_;
  }
  const uint64_t target_index = SubmapIdToUint64(target);
  const uint64_t source_index = SubmapIdToUint64(source);
  isam_factor_graph_->addExpressionFactor(
      between(Pose3_(SUBMAP_KEY(target_index)),
              Pose3_(SUBMAP_KEY(source_index))),
      transform_gtsam, noise_model);

  view_graph_.AddEdge(static_cast<int64_t>(target_index),
                      static_cast<int64_t>(source_index), transform);

  isam_->update(*isam_factor_graph_);
  isam_->update();
  isam_factor_graph_->resize(0);
  isam_->calculateEstimate();
}

template <typename PointT>
void MultiTrajectoryOptimizer<PointT>::AddVertex(const uint64_t index,
                                                 const Eigen::Matrix4f& pose) {
  gtsam::Pose3 pose_gtsam = gtsam::Pose3(pose.cast<double>());
  initial_estimate_.insert(SUBMAP_KEY(index), pose_gtsam);
  uint64_t pair_id = index;
  uint32_t* submap_id = reinterpret_cast<uint32_t*>(&pair_id);
  if (*submap_id == 0) {
    PRINT_INFO("It is the first submap of its trajectory, add a prior factor.");
    isam_factor_graph_->addExpressionFactor(Pose3_(SUBMAP_KEY(index)),
                                            pose_gtsam, prior_noise_model_);
  }

  view_graph_.AddVertex(index, pose);
  isam_->update(*isam_factor_graph_, initial_estimate_);
  isam_->update();
  initial_estimate_.clear();
}

template <typename PointT>
void MultiTrajectoryOptimizer<PointT>::UseCurrentStateAsBaseMap() {
  auto& frames = loop_detector_.GetFrames();
  if (frames.empty()) {
    return;
  }
  const int base_end_index = frames.size() - 1;
  const int base_start_index = 0;
  loop_detector_.SetSearchWindow(base_start_index, base_end_index);
}

template <typename PointT>
void MultiTrajectoryOptimizer<PointT>::RunFinalOptimizing() {
  PRINT_INFO("Final optimizing ... ");
  gtsam::Values result = isam_->calculateBestEstimate();
  for (auto& pair : trajectories_) {
    for (auto& submap : pair.second) {
      const uint64_t index = SubmapIdToUint64(submap->GetId());
      Eigen::Matrix4f pose =
          result.at<gtsam::Pose3>(SUBMAP_KEY(index)).matrix().cast<float>();
      submap->SetGlobalPose(pose);
      view_graph_.AddVertex(index, pose);
    }
  }
  // view_graph_.SaveTextFile("pcd/graph.txt");
  view_graph_.SaveImage("multi_graph.jpg", 0.5);
}

template class MultiTrajectoryOptimizer<pcl::PointXYZI>;

}  // namespace back_end
}  // namespace static_map
