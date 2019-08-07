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

#ifndef BUILDER_SIMPLE_FRAME_H_
#define BUILDER_SIMPLE_FRAME_H_
// third_party
#include <Eigen/Eigen>
// stl
#include <memory>
#include <string>
// local
#include "common/math.h"
#include "common/simple_time.h"
#include "descriptor/m2dp.h"
#include "registrators/registrator_interface.h"

namespace static_map {

using UtmPosition = Eigen::Vector3d;
using OdomPose = Eigen::Matrix4d;

/*
 * @class SimpleFrame
 * @brief base class for all frames, such as frame, submap,
 * a simple frame has its global pose(in map), local pose, cloud and
 * descriptor
 * @todo using double matrices instead of float ones
 */
template <typename PointType>
class SimpleFrame {
 public:
  using PointCloudType = pcl::PointCloud<PointType>;
  using PointCloudPtr = typename PointCloudType::Ptr;
  using PointCloudConstPtr = typename PointCloudType::ConstPtr;

  SimpleFrame()
      : global_pose_(Eigen::Matrix4f::Identity()),
        local_pose_(Eigen::Matrix4f::Identity()),
        transform_from_last_frame_(Eigen::Matrix4f::Identity()),
        transform_to_next_frame_(Eigen::Matrix4f::Identity()),
        related_utm_(UtmPosition::Zero()),
        got_related_utm_(false),
        related_odom_(OdomPose::Zero()),
        got_related_odom_(false) {}

  ~SimpleFrame() = default;

  virtual void ToPcdFile(const std::string& filename) {
    if (cloud_ == nullptr || cloud_->empty()) {
      return;
    }
    if (!filename.empty()) {
      pcl::io::savePCDFileBinaryCompressed(filename, *cloud_);
    } else {
      pcl::io::savePCDFileBinaryCompressed("simple_frame.pcd", *cloud_);
    }
  }
  inline Eigen::Matrix3f GlobalRotation() const {
    return Eigen::Matrix3f(global_pose_.block(0, 0, 3, 3));
  }
  inline Eigen::Vector3f GlobalTranslation() const {
    return Eigen::Vector3f(global_pose_.block(0, 3, 3, 1));
  }
  inline Eigen::Matrix3f LocalRotation() const {
    return Eigen::Matrix3f(local_pose_.block(0, 0, 3, 3));
  }
  inline Eigen::Vector3f LocalTranslation() const {
    return Eigen::Vector3f(local_pose_.block(0, 3, 3, 1));
  }

  inline Eigen::Matrix4f GlobalPose() const { return global_pose_; }
  inline void SetGlobalPose(const Eigen::Matrix4f& t) {
    global_pose_ = t;
    common::NormalizeRotation(global_pose_);
  }
  inline Eigen::VectorXf GlobalPoseIn6Dof() {
    return common::TransformToVector6(global_pose_);
  }

  inline Eigen::Matrix4f LocalPose() const { return local_pose_; }
  inline void SetLocalPose(const Eigen::Matrix4f& t) {
    local_pose_ = t;
    common::NormalizeRotation(local_pose_);
  }

  // cloud
  virtual PointCloudPtr Cloud() = 0;
  inline void SetCloud(const PointCloudPtr& cloud) { cloud_ = cloud; }
  inline void ResetCloud() { cloud_.reset(); }
  inline void ClearCloud() {
    if (cloud_) {
      cloud_->clear();
      cloud_->points.shrink_to_fit();
    }
  }

  // descriptor (m2dp)
  inline typename descriptor::M2dp<PointType>::Descriptor GetDescriptor()
      const {
    return descriptor_;
  }
  inline void SetDescriptor(
      const typename descriptor::M2dp<PointType>::Descriptor& d) {
    descriptor_ = d;
  }
  inline void CalculateDescriptor() {
    descriptor::M2dp<PointType> m2dp;
    if (m2dp.setInputCloud(cloud_)) {
      descriptor_ = m2dp.getFinalDescriptor();
    } else {
      PRINT_ERROR("did not get a descriptor for the frame.");
    }
  }

  // transform between last&next
  virtual void SetTransformToNext(const Eigen::Matrix4f& t) {
    transform_to_next_frame_ = t;
  }
  virtual void SetTransformFromLast(const Eigen::Matrix4f& t) {
    transform_from_last_frame_ = t;
  }
  inline Eigen::Matrix4f TransformToNext() const {
    return transform_to_next_frame_;
  }
  inline Eigen::Matrix4f TransformFromLast() const {
    return transform_from_last_frame_;
  }

  // utm
  inline void SetRelatedUtm(const UtmPosition& utm) {
    related_utm_ = utm;
    got_related_utm_ = true;
  }
  inline UtmPosition GetRelatedUtm() { return related_utm_; }
  inline bool HasUtm() { return got_related_utm_; }

  // odom
  inline void SetRelatedOdom(const OdomPose& odom) {
    related_odom_ = odom;
    got_related_odom_ = true;
  }
  inline OdomPose GetRelatedOdom() { return related_odom_; }
  inline bool HasOdom() { return got_related_odom_; }

  inline SimpleTime GetTimeStamp() { return stamp_; }
  inline void SetTimeStamp(const SimpleTime& time) { stamp_ = time; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  Eigen::Matrix4f global_pose_;
  Eigen::Matrix4f local_pose_;
  Eigen::Matrix4f transform_from_last_frame_;
  Eigen::Matrix4f transform_to_next_frame_;
  PointCloudPtr cloud_;
  SimpleTime stamp_;
  typename descriptor::M2dp<PointType>::Descriptor descriptor_;

  UtmPosition related_utm_;
  bool got_related_utm_;

  OdomPose related_odom_;
  bool got_related_odom_;
};

using registrator::InlierPointPairs;
template <typename PointT>
InlierPointPairs GetPointPairs(
    const std::shared_ptr<SimpleFrame<PointT>>& first_frame,
    const std::shared_ptr<SimpleFrame<PointT>>& last_frame, double max_distance,
    int sample = 1);

}  // namespace static_map

#endif  // BUILDER_SIMPLE_FRAME_H_
