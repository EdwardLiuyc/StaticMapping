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

#ifndef COMMON_MATH_H_
#define COMMON_MATH_H_

// stl
#include <cmath>
#include <iostream>
#include <limits>
#include <utility>
#include <vector>
// third party
#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Eigen"
#include "Eigen/SVD"
#include "glog/logging.h"

namespace Eigen {
namespace Rigid3 {

template <typename Scalar>
using Transform = Matrix<Scalar, 4, 4>;
template <typename Scalar>
using Translation = Matrix<Scalar, 3, 1>;
template <typename Scalar>
using Rotation = Matrix<Scalar, 3, 3>;
template <typename Scalar>
using Eulers = Matrix<Scalar, 3, 1>;

}  // namespace Rigid3

template <typename Scalar>
using Vector6 = Matrix<Scalar, 6, 1>;

template <int N>
using VectorNd = Matrix<double, N, 1>;
template <int N>
using VectorNf = Matrix<float, N, 1>;
}  // namespace Eigen

namespace static_map {
namespace common {

// Clamps 'value' to be in the range ['min', 'max'].
template <typename T>
T Clamp(const T value, const T min, const T max) {
  if (value > max) {
    return max;
  }
  if (value < min) {
    return min;
  }
  return value;
}

// Calculates 'base'^'exponent'.
template <typename T>
constexpr T Power(T base, int exponent) {
  return (exponent != 0) ? base * Power(base, exponent - 1) : T(1);
}

// Calculates a^2.
template <typename T>
constexpr T Pow2(T a) {
  return Power(a, 2);
}

// Converts from degrees to radians.
constexpr double DegToRad(double deg) { return M_PI * deg / 180.; }

// Converts form radians to degrees.
constexpr double RadToDeg(double rad) { return 180. * rad / M_PI; }

// Bring the 'difference' between two angles into [-pi; pi].
template <typename T>
T NormalizeAngleDifference(T difference) {
  while (difference > M_PI) {
    difference -= T(2. * M_PI);
  }
  while (difference < -M_PI) {
    difference += T(2. * M_PI);
  }
  return difference;
}

template <typename T>
Eigen::Rigid3::Eulers<T> RotationMatrixToEulerAngles(
    const Eigen::Rigid3::Rotation<T>& R) {
  Eigen::Matrix3d double_R = R.template cast<double>();

  double sy = std::sqrt(double_R(0, 0) * double_R(0, 0) +
                        double_R(1, 0) * double_R(1, 0));
  bool singular = sy < 1e-6;

  double x, y, z;
  if (!singular) {
    x = std::atan2(double_R(2, 1), double_R(2, 2));
    y = std::atan2(-double_R(2, 0), sy);
    z = std::atan2(double_R(1, 0), double_R(0, 0));
  } else {
    x = std::atan2(-double_R(1, 2), double_R(1, 1));
    y = std::atan2(-double_R(2, 0), sy);
    z = 0;
  }
  return Eigen::Vector3d(x, y, z).template cast<T>();
}

template <typename T>
Eigen::Quaternion<T> EulerAnglesToQuaternion(
    const Eigen::Rigid3::Eulers<T>& eulers) {
  Eigen::AngleAxis<T> yaw_angle(eulers[2], Eigen::Rigid3::Eulers<T>::UnitZ());
  Eigen::AngleAxis<T> pitch_angle(eulers[1], Eigen::Rigid3::Eulers<T>::UnitY());
  Eigen::AngleAxis<T> roll_angle(eulers[0], Eigen::Rigid3::Eulers<T>::UnitX());
  Eigen::Quaternion<T> q = yaw_angle * pitch_angle * roll_angle;

  return q;
}

template <typename T>
Eigen::Rigid3::Rotation<T> EulerAnglesToRotationMatrix(
    const Eigen::Rigid3::Eulers<T>& eulers) {
  return EulerAnglesToQuaternion(eulers).template toRotationMatrix();
}

template <typename T>
Eigen::Rigid3::Eulers<T> QuaternionToEulers(const Eigen::Quaternion<T>& q) {
  auto R = q.toRotationMatrix();
  return RotationMatrixToEulerAngles(R);
}

template <typename T>
Eigen::Vector6<T> TransformToVector6(const Eigen::Rigid3::Transform<T>& t) {
  Eigen::Vector6<T> ret;
  ret.topRows(3) = t.block(0, 3, 3, 1);
  ret.bottomRows(3) = RotationMatrixToEulerAngles(
      Eigen::Rigid3::Rotation<T>(t.block(0, 0, 3, 3)));

  return ret;
}

template <typename T>
Eigen::Rigid3::Transform<T> Vector6ToTransform(const Eigen::Vector6<T>& v) {
  Eigen::Rigid3::Transform<T> ret = Eigen::Rigid3::Transform<T>::Identity();
  Eigen::AngleAxis<T> yaw_angle(v[5], Eigen::Rigid3::Eulers<T>::UnitZ());
  Eigen::AngleAxis<T> pitch_angle(v[4], Eigen::Rigid3::Eulers<T>::UnitY());
  Eigen::AngleAxis<T> roll_angle(v[3], Eigen::Rigid3::Eulers<T>::UnitX());
  Eigen::Quaternion<T> q = yaw_angle * pitch_angle * roll_angle;

  ret.template block<3, 1>(0, 3) = v.topRows(3);
  ret.template block<3, 3>(0, 0) = q.toRotationMatrix();

  return ret;
}

template <typename T>
Eigen::Rigid3::Rotation<T> Rotation(const Eigen::Rigid3::Transform<T>& t) {
  return t.template block<3, 3>(0, 0);
}

template <typename T>
Eigen::Rigid3::Translation<T> Translation(
    const Eigen::Rigid3::Transform<T>& t) {
  return t.template block<3, 1>(0, 3);
}

template <typename T>
void PrintTransform(const Eigen::Rigid3::Transform<T>& t) {
  Eigen::Vector6<T> vec_6d = TransformToVector6(t);
  const double factor = 180. / M_PI;
  std::cout << "  translation : (" << vec_6d[0] << ", " << vec_6d[1] << ", "
            << vec_6d[2] << ")\n"
            << "  rotation    : (" << vec_6d[3] * factor << ", "
            << vec_6d[4] * factor << ", " << vec_6d[5] * factor << ")"
            << std::endl;
}

template <typename T>
Eigen::Matrix<T, 4, 4> InterpolateTransform(const Eigen::Matrix<T, 4, 4>& t1,
                                            const Eigen::Matrix<T, 4, 4>& t2,
                                            const float factor) {
  CHECK(factor >= 0. && factor <= 1.);
  Eigen::Matrix<T, 4, 4> new_transform = Eigen::Matrix<T, 4, 4>::Identity();
  const Eigen::Quaternion<T> q_a(Eigen::Matrix<T, 3, 3>(t1.block(0, 0, 3, 3)));
  const Eigen::Quaternion<T> q_b(Eigen::Matrix<T, 3, 3>(t2.block(0, 0, 3, 3)));
  new_transform.block(0, 0, 3, 3) = q_a.slerp(factor, q_b).toRotationMatrix();
  new_transform.block(0, 3, 3, 1) =
      t1.block(0, 3, 3, 1) +
      (t2.block(0, 3, 3, 1) - t1.block(0, 3, 3, 1)) * factor;
  return new_transform;
}

template <int N>
double DistanceToLine(const Eigen::Matrix<double, N, 1>& point,
                      const Eigen::Matrix<double, N, 1>& origin,
                      const Eigen::Matrix<double, N, 1>& direction) {
  const Eigen::Matrix<double, N, 1> op = point - origin;
  const Eigen::Matrix<double, N, 1> oc =
      op.dot(direction) * direction.normalized();
  return (oc - op).norm();
}

template <typename T>
Eigen::Rigid3::Transform<T> AverageTransforms(
    const std::vector<Eigen::Rigid3::Transform<T>>& transforms);

std::vector<Eigen::Vector3i> VoxelCastingBresenham(
    const Eigen::Vector3f& ray_start, const Eigen::Vector3f& ray_end,
    const float& step_size);

std::vector<Eigen::Vector3i> VoxelCastingDDA(const Eigen::Vector3f& ray_start,
                                             const Eigen::Vector3f& ray_end,
                                             const float& step_size);

template <typename Scalar>
std::pair<Eigen::Matrix<Scalar, 3, 1>, Eigen::Matrix<Scalar, 3, 1>>
PlaneFitting(const std::vector<Eigen::Matrix<Scalar, 3, 1>>& points);

template <typename T>
void NormalizeRotation(Eigen::Rigid3::Transform<T>& transform) {  // NOLINT
  Eigen::Quaternion<T> q(
      Eigen::Rigid3::Rotation<T>(transform.template block(0, 0, 3, 3)));
  transform.template block(0, 0, 3, 3) =
      q.template normalized().template toRotationMatrix();
}

}  // namespace common
}  // namespace static_map

#endif  // COMMON_MATH_H_
