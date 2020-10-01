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

#ifndef BUILDER_DATA_TYPES_H_
#define BUILDER_DATA_TYPES_H_
// third party
#include <Eigen/Eigen>
// stl
#include <cmath>
#include <memory>
#include <string>
#include <vector>
// local
#include "common/simple_time.h"
#include "glog/logging.h"
#include "pcl/common/transforms.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace static_map {
namespace sensors {

struct Header {
  uint32_t seq;
  SimpleTime stamp;
  std::string frame_id;
};

enum ImuType { kNormalImu, kInsCombinedImu, kImuTypeCount };
struct ImuMsg {
  typedef double Convariance[9];

  Header header;

  // gesture
  Eigen::Quaterniond orientation;
  Convariance orientation_covariance;

  // angular_velocity
  // rad/s
  Eigen::Vector3d angular_velocity;
  Convariance angular_velocity_covariance;

  // linear_acceleration
  // m/s^2
  Eigen::Vector3d linear_acceleration;
  Convariance linear_acceleration_covariance;

  double roll;
  double pitch;
  double yaw;

  using Ptr = std::shared_ptr<ImuMsg>;
  using ConstPtr = std::shared_ptr<const ImuMsg>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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

struct OdomInnerPose {
  Eigen::Quaterniond orientation;
  Eigen::Vector3d position;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct OdomMsg {
  using Ptr = std::shared_ptr<OdomMsg>;
  using ConstPtr = std::shared_ptr<const OdomMsg>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Header header;
  std::string child_frame_id;

  struct PoseWithCovariance {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    OdomInnerPose pose;
    double covariance[36];
  } pose;

  struct TwistWithCovariance {
    struct {
      Eigen::Vector3d linear;
      Eigen::Vector3d angular;
    } twist;
    double covariance[36];
  } twist;

  Eigen::Matrix4d PoseInMatrix() const;

  void SetPose(const Eigen::Matrix4d &pose_mat);

  Eigen::Quaterniond RotationInMatrix() const { return pose.pose.orientation; }
};

struct GpsEnuMsg {
  using Ptr = std::shared_ptr<GpsEnuMsg>;
  using ConstPtr = std::shared_ptr<const GpsEnuMsg>;

  Header header;

  double x = 0.;
  double y = 0.;
  double z = 0.;

  Eigen::Vector3d ToMatrix() { return Eigen::Vector3d(x, y, z); }
};

struct TimedPose {
  TimedPose() = default;
  TimedPose(const SimpleTime &t, const Eigen::Matrix4d &p) : time(t), pose(p) {}
  SimpleTime time;
  Eigen::Matrix4d pose;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class EigenPointCloud {
 public:
  EigenPointCloud() = default;

  template <typename PointType>
  void FromPointCloud(const typename pcl::PointCloud<PointType>::Ptr &cloud);

  bool HasNormals() const;

  void ApplyTransform(const Eigen::Matrix4d &transform);

  void ApplyMotionCompensation(const Eigen::Matrix4d &transform);

  void CalculateNormals();

  std::vector<int> indices;
  std::vector<int> indices_to_keep;
  std::vector<double> factors;
  Eigen::MatrixXd points;
  Eigen::MatrixXd normals;
};

template <typename PointType>
void EigenPointCloud::FromPointCloud(
    const typename pcl::PointCloud<PointType>::Ptr &cloud) {
  CHECK(cloud && !cloud->empty());

  const int size = cloud->size();
  indices.resize(size);
  factors.resize(size);
  // We assume points are in 3d by default.
  points.resize(3, size);
  // We leave normals not inited, because we will need the normals only if we
  // use the cloud as a target, so initialize them later.

  for (int i = 0; i < size; ++i) {
    indices[i] = i;
    factors[i] = static_cast<double>(i) / size;
    points.col(i) << cloud->points[i].x, cloud->points[i].y, cloud->points[i].z;
  }
}

template <typename PointT>
class InnerPointCloudData {
 public:
  using PclCloudType = pcl::PointCloud<PointT>;
  using PclCloudPtr = typename PclCloudType::Ptr;
  using Ptr = std::shared_ptr<InnerPointCloudData<PointT>>;

  InnerPointCloudData() = default;
  explicit InnerPointCloudData(const PclCloudPtr cloud) { SetPclCloud(cloud); }

  bool Empty() { return pcl_cloud_ == nullptr || pcl_cloud_->points.empty(); }

  void Clear() {
    pcl_cloud_.reset(new PclCloudType);
    eigen_cloud_.reset(new EigenPointCloud);
  }

  void TransformCloud(const Eigen::Matrix4d &T) {
    if (!Empty()) {
      typename pcl::PointCloud<PointT>::Ptr transformed_cloud(
          new typename pcl::PointCloud<PointT>);
      pcl::transformPointCloud(*pcl_cloud_, *transformed_cloud, T);
      pcl_cloud_ = transformed_cloud;
    }
    if (eigen_cloud_) {
      eigen_cloud_->ApplyTransform(T);
    }
  }

  void CalculateNormals() {
    CHECK(eigen_cloud_);
    eigen_cloud_->CalculateNormals();
  }

  /// @brief SetPclCloud: The function will take care of all members inside,
  /// including time&pcl_cloud&eigen_cloud.
  void SetPclCloud(const PclCloudPtr cloud) {
    pcl_cloud_ = cloud;
    if (!pcl_cloud_) {
      return;
    }
    eigen_cloud_.reset(new EigenPointCloud);
    eigen_cloud_->template FromPointCloud<PointT>(pcl_cloud_);

    time_ = ToLocalTime(pcl_cloud_->header.stamp);
  }

  PclCloudPtr GetPclCloud() const { return pcl_cloud_; }
  std::shared_ptr<EigenPointCloud> GetEigenCloud() const {
    return eigen_cloud_;
  }
  SimpleTime GetTime() const { return time_; }

 public:
  float delta_time_in_cloud;

 private:
  PclCloudPtr pcl_cloud_;
  std::shared_ptr<EigenPointCloud> eigen_cloud_;
  SimpleTime time_;
};

}  // namespace sensors
}  // namespace static_map

#endif  // BUILDER_DATA_TYPES_H_
