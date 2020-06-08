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

#ifndef BACK_END_MULTI_TRAJECTORY_OPTIMIZER_H_
#define BACK_END_MULTI_TRAJECTORY_OPTIMIZER_H_

// third party
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
// stl
#include <map>
#include <memory>
#include <vector>
// local
#include "back_end/loop_detector.h"
#include "back_end/view_graph.h"
#include "builder/submap.h"

namespace static_map {
namespace back_end {

template <typename PointT>
class MultiTrajectoryOptimizer {
 public:
  explicit MultiTrajectoryOptimizer(const LoopDetectorSettings &l_d_setting);
  ~MultiTrajectoryOptimizer();

  void AddSubmap(const std::shared_ptr<Submap<PointT>> &submap,
                 bool do_loop_detect);
  void AddSubmapConnection(const SubmapId &target, const SubmapId &source,
                           const Eigen::Matrix4d &transform,
                           bool is_loop_constraint = false);

  void UseCurrentStateAsBaseMap();

  void RunFinalOptimizing();

 protected:
  void AddVertex(const uint64_t index, const Eigen::Matrix4d &pose);

 private:
  std::map<int /*trajectory id*/, std::vector<std::shared_ptr<Submap<PointT>>>>
      trajectories_;

  LoopDetector<PointT> loop_detector_;

  std::unique_ptr<gtsam::ISAM2> isam_;
  std::shared_ptr<gtsam::ExpressionFactorGraph> isam_factor_graph_;
  gtsam::Values initial_estimate_;

  gtsam::noiseModel::Base::shared_ptr prior_noise_model_;
  gtsam::noiseModel::Base::shared_ptr stable_odom_model_;
  gtsam::noiseModel::Base::shared_ptr unstable_odom_model_;
  gtsam::noiseModel::Base::shared_ptr loop_closure_model_;

  ViewGraph view_graph_;
};

}  // namespace back_end
}  // namespace static_map

#endif  // BACK_END_MULTI_TRAJECTORY_OPTIMIZER_H_
