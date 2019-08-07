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

#ifndef BUILDER_SENSORS_H_
#define BUILDER_SENSORS_H_
// third party
#include <Eigen/Eigen>
// stl
#include <cmath>
#include <memory>
#include <string>
// local
#include "common/simple_time.h"

namespace static_map {

typedef struct {
  double x;
  double y;
  double z;
  double w;
} Quaternion;

struct Vector3 {
 public:
  Vector3() { x = y = z = 0.; }

  Vector3(double x_, double y_, double z_) {
    x = x_;
    y = y_;
    z = z_;
  }

  double x, y, z;

  inline double distance_to_vector3(const Vector3 &vector) const {
    return std::sqrt(pow(vector.x - x, 2.) + std::pow(vector.y - y, 2.) +
                     std::pow(vector.z - z, 2.));
  }

  inline double distance_to_point3d(const Vector3 &vector) const {
    return distance_to_vector3(vector);
  }

  void set_xy(double d_x, double d_y) {
    x = d_x;
    y = d_y;
  }

  void set_xyz(double d_x, double d_y, double d_z) {
    x = d_x;
    y = d_y;
    z = d_z;
  }

  Vector3 mid_point_with(const Vector3 &vector) const {
    Vector3 point;
    point.set_xyz((vector.x + x) * 0.5, (vector.y + y) * 0.5,
                  (vector.z + z) * 0.5);
    return point;
  }

  Vector3 operator+(const Vector3 &b) const {
    return Vector3(x + b.x, y + b.y, z + b.z);
  }

  Vector3 operator-(const Vector3 &b) const {
    return Vector3(x - b.x, y - b.y, z - b.z);
  }

  Eigen::Vector3d ToEigenVector() { return Eigen::Vector3d(x, y, z); }

  std::string DebugString() const {
    std::ostringstream out;
    out << x << "," << y << "," << z;
    return out.str();
  }
};
typedef Vector3 Point;
typedef Vector3 Point3d;

namespace sensors {

struct Header {
 public:
  Header() = default;
  ~Header() = default;

  Header &operator=(const Header &header) {
    seq = header.seq;
    stamp = header.stamp;
    frame_id = header.frame_id;

    return *this;
  }

  uint32_t seq;
  SimpleTime stamp;
  std::string frame_id;
};

struct ImuMsg {
 public:
  ImuMsg() = default;
  ~ImuMsg() = default;

  typedef double Convariance[9];

  Header header;

  // gesture
  Quaternion orientation;
  Convariance orientation_covariance;

  // angular_velocity
  // rad/s
  Vector3 angular_velocity;
  Convariance angular_velocity_covariance;

  // linear_acceleration
  // m/s^2
  Vector3 linear_acceleration;
  Convariance linear_acceleration_covariance;

  double roll;
  double pitch;
  double yaw;

  using Ptr = std::shared_ptr<ImuMsg>;
  using ConstPtr = std::shared_ptr<const ImuMsg>;
};

/************ NavSatStatus **************/
// status
constexpr int8_t STATUS_NO_FIX = -1;
constexpr int8_t STATUS_FIX = 0;
constexpr int8_t STATUS_SBAS_FIX = 1;
constexpr int8_t STATUS_GBAS_FIX = 2;
// service
constexpr uint16_t SERVICE_GPS = 1;
constexpr uint16_t SERVICE_GLONASS = 2;
constexpr uint16_t SERVICE_COMPASS = 4;
constexpr uint16_t SERVICE_GALILEO = 8;

constexpr uint8_t COVARIANCE_TYPE_UNKNOWN = 0;
constexpr uint8_t COVARIANCE_TYPE_APPROXIMATED = 1;
constexpr uint8_t COVARIANCE_TYPE_DIAGONAL_KNOWN = 2;
constexpr uint8_t COVARIANCE_TYPE_KNOWN = 3;
struct NavSatFixMsg {
 public:
  using Ptr = std::shared_ptr<NavSatFixMsg>;
  using ConstPtr = std::shared_ptr<const NavSatFixMsg>;

  Header header;
  struct NavSatStatus {
    int8_t status;
    uint16_t service;
  } status;
  double latitude;
  double longtitude;
  double altitude;
  double position_covariance[9];

  uint8_t position_covariance_type;
};

struct OdomMsg {
  using Ptr = std::shared_ptr<OdomMsg>;
  using ConstPtr = std::shared_ptr<const OdomMsg>;

  Header header;
  std::string child_frame_id;

  struct PoseWithCovariance {
    struct {
      Point position;
      struct {
        double w = 1.;
        double x = 0.;
        double y = 0.;
        double z = 0.;
      } orientation;
    } pose;
    double covariance[36];
  } pose;

  struct TwistWithCovariance {
    struct {
      Vector3 linear;
      Vector3 angular;
    } twist;
    double covariance[36];
  } twist;

  Eigen::Matrix4d PoseInMatrix() const {
    Eigen::Matrix4d pose_in_matrix = Eigen::Matrix4d::Identity();
    pose_in_matrix.block(0, 3, 3, 1) << pose.pose.position.x,
        pose.pose.position.y, pose.pose.position.z;
    pose_in_matrix.block(0, 0, 3, 3) =
        Eigen::Quaterniond(pose.pose.orientation.w, pose.pose.orientation.x,
                           pose.pose.orientation.y, pose.pose.orientation.z)
            .toRotationMatrix();
    return pose_in_matrix;
  }

  void SetPose(const Eigen::Matrix4d &pose_mat) {
    pose.pose.position.set_xyz(pose_mat(0, 3), pose_mat(1, 3), pose_mat(2, 3));
    Eigen::Quaterniond q(Eigen::Matrix3d(pose_mat.block(0, 0, 3, 3)));
    pose.pose.orientation.w = q.w();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
  }

  Eigen::Quaterniond RotationInMatrix() const {
    return Eigen::Quaterniond(pose.pose.orientation.w, pose.pose.orientation.x,
                              pose.pose.orientation.y, pose.pose.orientation.z);
  }
};

struct UtmMsg {
  using Ptr = std::shared_ptr<UtmMsg>;
  using ConstPtr = std::shared_ptr<const UtmMsg>;

  Header header;

  double x = 0.;
  double y = 0.;
  double z = 0.;

  Eigen::Vector3d ToMatrix() { return Eigen::Vector3d(x, y, z); }
};

}  // namespace sensors
}  // namespace static_map

#endif  // BUILDER_SENSORS_H_
