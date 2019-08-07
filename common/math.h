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
#include "Eigen/SVD"
#include "glog/logging.h"

namespace Eigen {
template <typename Scalar>
using Vector6 = Matrix<Scalar, 6, 1>;

template <int N>
using VectorNd = Matrix<double, N, 1>;
template <int N>
using VectorNf = Matrix<float, N, 1>;
}  // namespace Eigen

#define TRANSFORM(Scalar) Eigen::Matrix<Scalar, 4, 4>
#define TRANSFORM_6D(Scalar) Eigen::Vector6<Scalar>
#define TRANSLATION(Scalar) Eigen::Matrix<Scalar, 3, 1>
#define EULERS(Scalar) Eigen::Matrix<Scalar, 3, 1>
#define ROTATION(Scalar) Eigen::Matrix<Scalar, 3, 3>

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
EULERS(T)
RotationMatrixToEulerAngles(const ROTATION(T) & R) {
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
Eigen::Quaternion<T> EulerAnglesToQuaternion(const EULERS(T) & eulers) {
  Eigen::AngleAxis<T> yaw_angle(eulers[2], EULERS(T)::UnitZ());
  Eigen::AngleAxis<T> pitch_angle(eulers[1], EULERS(T)::UnitY());
  Eigen::AngleAxis<T> roll_angle(eulers[0], EULERS(T)::UnitX());
  Eigen::Quaternion<T> q = yaw_angle * pitch_angle * roll_angle;

  return q;
}

template <typename T>
ROTATION(T)
EulerAnglesToRotationMatrix(const EULERS(T) & eulers) {
  return EulerAnglesToQuaternion(eulers).template toRotationMatrix();
}

template <typename T>
EULERS(T)
QuaternionToEulers(const Eigen::Quaternion<T>& q) {
  auto R = q.toRotationMatrix();
  return RotationMatrixToEulerAngles(R);
}

template <typename T>
TRANSFORM_6D(T)
TransformToVector6(const TRANSFORM(T) & t) {
  TRANSFORM_6D(T) ret;
  ret.topRows(3) = t.block(0, 3, 3, 1);
  ret.bottomRows(3) =
      RotationMatrixToEulerAngles(ROTATION(T)(t.block(0, 0, 3, 3)));

  return ret;
}

template <typename T>
TRANSFORM(T)
Vector6ToTransform(const TRANSFORM_6D(T) & v) {
  TRANSFORM(T) ret = TRANSFORM(T)::Identity();
  Eigen::AngleAxis<T> yaw_angle(v[5], EULERS(T)::UnitZ());
  Eigen::AngleAxis<T> pitch_angle(v[4], EULERS(T)::UnitY());
  Eigen::AngleAxis<T> roll_angle(v[3], EULERS(T)::UnitX());
  Eigen::Quaternion<T> q = yaw_angle * pitch_angle * roll_angle;

  ret.template block<3, 1>(0, 3) = v.topRows(3);
  ret.template block<3, 3>(0, 0) = q.toRotationMatrix();

  return ret;
}

template <typename T>
void PrintTransform(const TRANSFORM(T) & t) {
  TRANSFORM_6D(T) vec_6d = TransformToVector6(t);
  std::cout << "  translation : (" << vec_6d[0] << ", " << vec_6d[1] << ", "
            << vec_6d[2] << ")\n"
            << "  rotation    : (" << vec_6d[3] << ", " << vec_6d[4] << ", "
            << vec_6d[5] << ")" << std::endl;
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

struct Int3D {
  Int3D() : x(0), y(0), z(0) {}
  Int3D(const int& _x, const int& _y, const int& _z) : x(_x), y(_y), z(_z) {}

  Int3D operator+(const Int3D& b) const {
    return Int3D(x + b.x, y + b.y, z + b.z);
  }
  Int3D operator-(const Int3D& b) const {
    return Int3D(x - b.x, y - b.y, z - b.z);
  }

  int x, y, z;
};

std::vector<Eigen::Vector3i> VoxelCasting(const Eigen::Vector3f& ray_start,
                                          const Eigen::Vector3f& ray_end,
                                          const float& step_size);

std::vector<Eigen::Vector3i> VoxelCastingBresenham(
    const Eigen::Vector3f& ray_start, const Eigen::Vector3f& ray_end,
    const float& step_size);

std::vector<Eigen::Vector3i> VoxelCastingDDA(const Eigen::Vector3f& ray_start,
                                             const Eigen::Vector3f& ray_end,
                                             const float& step_size);

template <typename Scalar>
std::pair<Eigen::Matrix<Scalar, 3, 1>, Eigen::Matrix<Scalar, 3, 1>>
PlaneFitting(const std::vector<Eigen::Matrix<Scalar, 3, 1>>& points) {
  if (points.size() < 2) {
    Scalar limit = std::numeric_limits<Scalar>::max();
    return std::make_pair(Eigen::Matrix<Scalar, 3, 1>(limit, limit, limit),
                          Eigen::Matrix<Scalar, 3, 1>(limit, limit, limit));
  }
  // copy coordinates to  matrix in Eigen format
  size_t num_atoms = points.size();
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> coord(3, num_atoms);
  for (size_t i = 0; i < num_atoms; ++i) {
    coord.template col(i) = points[i];
  }

  // calculate centroid
  Eigen::Matrix<Scalar, 3, 1> centroid(coord.template row(0).template mean(),
                                       coord.template row(1).template mean(),
                                       coord.template row(2).template mean());

  // subtract centroid
  coord.template row(0).array() -= centroid(0);
  coord.template row(1).array() -= centroid(1);
  coord.template row(2).array() -= centroid(2);

  // we only need the left-singular matrix here
  // http://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
  Eigen::JacobiSVD<Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>> svd =
      coord.template jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeThinV);
  Eigen::Matrix<Scalar, 3, 1> plane_normal =
      svd.template matrixU().template rightCols<1>();
  return std::make_pair(centroid, plane_normal);
}

template <typename T>
void NormalizeRotation(TRANSFORM(T) & transform) {
  Eigen::Quaternion<T> q(ROTATION(T)(transform.template block(0, 0, 3, 3)));
  transform.template block(0, 0, 3, 3) =
      q.template normalized().template toRotationMatrix();
}

#ifdef __cplusplus
#define cast_uint32_t static_cast<uint32_t>
#else
#define cast_uint32_t (uint32_t)
#endif

static inline float fastpow2(float p) {
  float offset = (p < 0) ? 1.0f : 0.0f;
  float clipp = (p < -126) ? -126.0f : p;
  int w = clipp;
  float z = clipp - w + offset;
  union {
    uint32_t i;
    float f;
  } v = {cast_uint32_t((1 << 23) *
                       (clipp + 121.2740575f + 27.7280233f / (4.84252568f - z) -
                        1.49012907f * z))};

  return v.f;
}

static inline float fastexp(float p) { return fastpow2(1.442695040f * p); }

static inline float fasterlog(float x) {
  union {
    float f;
    uint32_t i;
  } vx = {x};
  float y = vx.i;
  y *= 8.2629582881927490e-8f;
  return y - 87.989971088f;
}

}  // namespace common
}  // namespace static_map

#undef TRANSFORM
#undef TRANSFORM_6D
#undef TRANSLATION
#undef EULERS
#undef ROTATION

#endif  // COMMON_MATH_H_
