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

#ifndef BACK_END_ISAM_OPTIMIZER_H_
#define BACK_END_ISAM_OPTIMIZER_H_

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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// stl
#include <map>
#include <memory>
#include <utility>
// local
#include <boost/optional.hpp>
#include "back_end/loop_detector_options.h"
#include "back_end/view_graph.h"
#include "builder/submap.h"

namespace static_map {
// forward declare
namespace sensors {
struct ImuMsg;
}

namespace back_end {

template <typename PointT>
class LoopDetector;

struct IsamOptimizerOptions {
  bool use_odom = false;
  bool use_gps = false;
  bool output_graph_pic = false;
  int gps_skip_num = 25;
};

template <typename PointT>
class IsamOptimizer {
 public:
  IsamOptimizer(const IsamOptimizerOptions &options,
                const LoopDetectorSettings &l_d_setting);
  ~IsamOptimizer() {}

  IsamOptimizer(const IsamOptimizer &) = delete;
  IsamOptimizer &operator=(const IsamOptimizer &) = delete;

  /// @brief add a new vertex
  void AddFrame(const std::shared_ptr<Submap<PointT>> &frame,
                const double match_score);
  /// @brief for imu factor in optimization
  void AddImuData(const data::ImuMsg &imu_msg);
  /// @brief set static tf link from odom to lidar(cloud frame)
  void SetTransformOdomToLidar(const Eigen::Matrix4d &t);
  /// @brief get the odom->lidar tf after calibration
  Eigen::Matrix4d GetTransformOdomToLidar();
  /// @brief tf connection tracking_frame -> gps
  void SetTrackingToGps(const Eigen::Matrix4d &t);

  void RunFinalOptimazation();

  Eigen::Matrix4d GetGpsCoordTransform();

  std::map<int64_t, ViewGraph::GraphItem> GetWholeGraph() const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  void AddLoopCloseEdge(
      const int target_index, const int source_index,
      const Eigen::Matrix4d &transform_from_last_pose,
      const gtsam::noiseModel::Base::shared_ptr &constraint_noise);
  void AddVertex(const int &index, const Eigen::Matrix4d &pose,
                 const Eigen::Matrix4d &transform_from_last_pose,
                 const gtsam::noiseModel::Base::shared_ptr &odom_noise);
  void IsamUpdate(const int update_time = 1);

  double AnalyseAllFramePoseForMaxRotation();

  void SolveGpsCorrdAlone();

  gtsam::Values UpdateAllPose();

 private:
  std::unique_ptr<gtsam::ISAM2> isam_;
  std::shared_ptr<gtsam::NonlinearFactorGraph> isam_factor_graph_;
  gtsam::Values initial_estimate_;
  gtsam::Pose3 gps_coord_transform_;  // map origin in GPS coord
  gtsam::noiseModel::Base::shared_ptr prior_noise_model_;
  gtsam::noiseModel::Base::shared_ptr gps_noise_model_;
  gtsam::noiseModel::Base::shared_ptr frame_match_noise_model_;
  gtsam::noiseModel::Base::shared_ptr loop_closure_noise_model_;
  // used in online calibration
  gtsam::noiseModel::Base::shared_ptr odom_tf_noise_model_;
  gtsam::noiseModel::Base::shared_ptr odom_noise_model_;

  Eigen::Matrix4d tf_odom_lidar_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d tf_tracking_gps_ = Eigen::Matrix4d::Identity();

  std::unique_ptr<LoopDetector<PointT>> loop_detector_;
  IsamOptimizerOptions options_;

  std::map<int /* frame index*/, EnuPosition> cached_enu_;
  bool calculated_first_gps_coord_ = false;

  ViewGraph view_graph_;
  bool calib_factor_inserted_ = false;
  int accumulated_gps_count_ = 0;
};

}  // namespace back_end
}  // namespace static_map

#endif  // BACK_END_ISAM_OPTIMIZER_H_
