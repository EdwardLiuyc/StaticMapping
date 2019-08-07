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

#ifndef BUILDER_COST_FUNCTIONS_UTM_MAP_MATCH_H_
#define BUILDER_COST_FUNCTIONS_UTM_MAP_MATCH_H_

#include "ceres/ceres.h"

namespace static_map {
namespace cost_functions {

class UtmToMapPath {
 public:
  UtmToMapPath(const Eigen::Vector2d& map_position,
               const Eigen::Vector2d& utm_position) {
    map_[0] = map_position[0];
    map_[1] = map_position[1];
    utm_[0] = utm_position[0];
    utm_[1] = utm_position[1];
  }

  template <typename T>
  bool operator()(const T* const t_x, const T* const t_y, const T* const theta,
                  T* residuals_ptr /* point to plane distance */) const {
    Eigen::Matrix<T, 2, 1> t(*t_x, *t_y);
    Eigen::Matrix<T, 2, 2> r;
    r << ceres::cos(*theta), -ceres::sin(*theta), ceres::sin(*theta),
        ceres::cos(*theta);
    Eigen::Matrix<T, 2, 1> utm;
    utm << T(utm_[0]), T(utm_[1]);
    Eigen::Matrix<T, 2, 1> utm_in_map = r * utm + t;

    residuals_ptr[0] = ceres::sqrt(ceres::pow(utm_in_map(0, 0) - map_[0], 2) +
                                   ceres::pow(utm_in_map(1, 0) - map_[1], 2));

    return true;
  }

  static ceres::CostFunction* Create(const Eigen::Vector2d& map_position,
                                     const Eigen::Vector2d& utm_position) {
    return new ceres::AutoDiffCostFunction<UtmToMapPath, 1, 1, 1, 1>(
        new UtmToMapPath(map_position, utm_position));
  }

 private:
  double map_[2];
  double utm_[2];
};

class UtmToMapPathNormal {
 public:
  UtmToMapPathNormal(const Eigen::Vector2d& map_position,
                     const Eigen::Vector2d& utm_position,
                     const Eigen::Vector2d& normal)
      : map_(map_position), utm_(utm_position), normal_(normal) {}

  template <typename T>
  bool operator()(const T* const t_x, const T* const t_y, const T* const theta,
                  T* residuals_ptr /* point to plane distance */) const {
    Eigen::Matrix<T, 2, 1> t(*t_x, *t_y);
    Eigen::Matrix<T, 2, 2> r;
    r << ceres::cos(*theta), -ceres::sin(*theta), ceres::sin(*theta),
        ceres::cos(*theta);
    Eigen::Matrix<T, 2, 1> a =
        r * utm_.template cast<T>() + t - map_.template cast<T>();
    T dot = a.template dot(normal_.template cast<T>());
    Eigen::Matrix<T, 2, 1> e =
        a - normal_.normalized().template cast<T>() * dot;

    residuals_ptr[0] = e.template norm();
    return true;
  }

  static ceres::CostFunction* Create(const Eigen::Vector2d& map_position,
                                     const Eigen::Vector2d& utm_position,
                                     const Eigen::Vector2d& normal) {
    return new ceres::AutoDiffCostFunction<UtmToMapPathNormal, 1, 1, 1, 1>(
        new UtmToMapPathNormal(map_position, utm_position, normal));
  }

 private:
  Eigen::Vector2d map_;
  Eigen::Vector2d utm_;
  Eigen::Vector2d normal_;
};

class UtmToMap3D {
 public:
  UtmToMap3D(const Eigen::Vector3d& map_position,
             const Eigen::Vector3d& utm_position,
             const Eigen::Vector3d& map_direction)
      : map_(map_position), utm_(utm_position), direction_(map_direction) {}

  template <typename T>
  bool operator()(const T* const t, const T* const r, T* residuals_ptr) const {
    using Vector3T = Eigen::Matrix<T, 3, 1>;
    using Matrix3T = Eigen::Matrix<T, 3, 3>;

    // init R and T
    Vector3T eulers;
    eulers << r[0], r[1], r[2];
    Matrix3T rotation = common::EulerAnglesToRotationMatrix(eulers);
    Vector3T translation;
    translation << t[0], t[1], t[2];

    Vector3T direction = direction_.template cast<T>().normalized();
    Vector3T a = rotation * utm_.template cast<T>() + translation -
                 map_.template cast<T>();
    Vector3T c = direction * a.dot(direction);

    // calcualte distance
    residuals_ptr[0] = (a - c).template norm();
    return true;
  }

  static ceres::CostFunction* Create(const Eigen::Vector3d& map_position,
                                     const Eigen::Vector3d& utm_position,
                                     const Eigen::Vector3d& map_direction) {
    return new ceres::AutoDiffCostFunction<UtmToMap3D, 1, 3, 3>(
        new UtmToMap3D(map_position, utm_position, map_direction));
  }

 private:
  Eigen::Vector3d map_;
  Eigen::Vector3d utm_;
  Eigen::Vector3d direction_;
};

}  // namespace cost_functions
}  // namespace static_map

#endif  // BUILDER_COST_FUNCTIONS_UTM_MAP_MATCH_H_
