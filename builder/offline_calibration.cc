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

#include "builder/offline_calibration.h"

#include <vector>

#include "builder/cost_functions/odom_map_match.h"
#include "builder/submap.h"
#include "builder/trajectory.h"
#include "common/math.h"

namespace static_map {

template <typename PointT>
bool OfflineCalibrationOdomToLidar(
    const std::shared_ptr<Trajectory<PointT>> trajectory,
    const Eigen::Matrix4d& init_estimate, Eigen::Matrix4d* const result) {
  CHECK(trajectory);
  CHECK(result);

  const int submap_size = trajectory->size();
  if (submap_size <= 1) {
    PRINT_WARNING("too few submaps to calculate the transfrom");
    return false;
  }

  std::vector<Eigen::Vector3d> map_path_positions;
  std::vector<Eigen::Vector3d> map_path_directions;
  std::vector<OdomPose> odom_poses;

  for (auto& submap : *trajectory) {
    if (!submap->HasOdom()) {
      continue;
    }
    Eigen::Vector3d path_position = submap->GlobalTranslation();
    map_path_positions.push_back(path_position);

    // refer to
    // https://stackoverflow.com/questions/1568568/how-to-convert-euler-angles-to-directional-vector
    // get the directional vector of a rotation matrix
    Eigen::Matrix3d path_roation = submap->GlobalRotation();
    Eigen::Vector3d eulers = common::RotationMatrixToEulerAngles(path_roation);
    Eigen::Vector3d direction(std::cos(eulers[2]) * std::cos(eulers[1]),
                              std::sin(eulers[2]) * std::cos(eulers[1]),
                              std::sin(eulers[1]));
    map_path_directions.push_back(direction);

    OdomPose odom_pose = submap->GetRelatedOdom();
    odom_poses.push_back(odom_pose);
  }

  if (map_path_positions.empty()) {
    return false;
  }

  const int size = map_path_positions.size();
  // @todo add a init estimate based-on tf_odom_lidar
  Eigen::Vector6<double> init_estimate_6d =
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

  // update the tf connection
  result->block<3, 3>(0, 0) = rotation;
  result->block<3, 1>(0, 3) = translation;
  PRINT_INFO("odom -> lidar match result :");
  common::PrintTransform(*result);

  return true;
}

template bool OfflineCalibrationOdomToLidar(
    const std::shared_ptr<Trajectory<pcl::PointXYZI>> trajectory,
    const Eigen::Matrix4d& init_estimate, Eigen::Matrix4d* const result);

}  // namespace static_map
