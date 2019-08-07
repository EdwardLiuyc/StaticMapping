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

#ifndef BUILDER_COST_FUNCTIONS_ODOM_MAP_MATCH_H_
#define BUILDER_COST_FUNCTIONS_ODOM_MAP_MATCH_H_

// third party
#include "Eigen/Eigen"
#include "ceres/ceres.h"
// local
#include "common/math.h"

namespace static_map {
namespace cost_functions {

class OdomToMapPath {
 public:
  OdomToMapPath(const Eigen::Vector3d& map_position,
                const Eigen::Matrix4d& odom_pose,
                const Eigen::Vector3d& map_direction)
      : map_(map_position), odom_pose_(odom_pose), direction_(map_direction) {}

  template <typename T>
  bool operator()(const T* const t, const T* const r, T* residuals_ptr) const {
    using Vector3T = Eigen::Matrix<T, 3, 1>;
    using Matrix4T = Eigen::Matrix<T, 4, 4>;

    // init R and T
    Vector3T eulers;
    eulers << r[0], r[1], r[2];

    Matrix4T tf_odom_path = Matrix4T::Identity();
    tf_odom_path.template block(0, 0, 3, 3) =
        common::EulerAnglesToRotationMatrix(eulers);
    tf_odom_path.template block(0, 3, 3, 1) << t[0], t[1], t[2];

    Matrix4T odom_pose_in_map = tf_odom_path.template inverse() *
                                odom_pose_.template cast<T>() * tf_odom_path;

    Vector3T direction = direction_.template cast<T>().normalized();
    Vector3T a =
        odom_pose_in_map.template block(0, 3, 3, 1) - map_.template cast<T>();
    Vector3T c = direction * a.dot(direction);

    // calcualte distance
    residuals_ptr[0] = (a - c).template norm();
    return true;
  }

  static ceres::CostFunction* Create(const Eigen::Vector3d& map_position,
                                     const Eigen::Matrix4d& odom_pose,
                                     const Eigen::Vector3d& map_direction) {
    return new ceres::AutoDiffCostFunction<OdomToMapPath, 1, 3, 3>(
        new OdomToMapPath(map_position, odom_pose, map_direction));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  Eigen::Vector3d map_;
  Eigen::Matrix4d odom_pose_;
  Eigen::Vector3d direction_;
};

}  // namespace cost_functions
}  // namespace static_map

#endif  // BUILDER_COST_FUNCTIONS_ODOM_MAP_MATCH_H_
