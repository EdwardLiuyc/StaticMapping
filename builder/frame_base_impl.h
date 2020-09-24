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

#ifndef BUILDER_FRAME_BASE_IMPL_H_
#define BUILDER_FRAME_BASE_IMPL_H_

#include <string>

namespace static_map {

template <typename PointType>
inline Eigen::Matrix3d FrameBase<PointType>::GlobalRotation() const {
  return Eigen::Matrix3d(global_pose_.block(0, 0, 3, 3));
}
template <typename PointType>
inline Eigen::Vector3d FrameBase<PointType>::GlobalTranslation() const {
  return Eigen::Vector3d(global_pose_.block(0, 3, 3, 1));
}
template <typename PointType>
inline Eigen::Matrix3d FrameBase<PointType>::LocalRotation() const {
  return Eigen::Matrix3d(local_pose_.block(0, 0, 3, 3));
}
template <typename PointType>
inline Eigen::Vector3d FrameBase<PointType>::LocalTranslation() const {
  return Eigen::Vector3d(local_pose_.block(0, 3, 3, 1));
}
template <typename PointType>
inline Eigen::Matrix4d FrameBase<PointType>::GlobalPose() const {
  return global_pose_;
}
template <typename PointType>
inline void FrameBase<PointType>::SetGlobalPose(const Eigen::Matrix4d& t) {
  global_pose_ = t;
  common::NormalizeRotation(global_pose_);
}
template <typename PointType>
inline Eigen::VectorXd FrameBase<PointType>::GlobalPoseIn6Dof() const {
  return common::TransformToVector6(global_pose_);
}
template <typename PointType>
inline Eigen::Matrix4d FrameBase<PointType>::LocalPose() const {
  return local_pose_;
}
template <typename PointType>
inline void FrameBase<PointType>::SetLocalPose(const Eigen::Matrix4d& t) {
  local_pose_ = t;
  common::NormalizeRotation(local_pose_);
}

template <typename PointType>
void FrameBase<PointType>::SetTransformToNext(const Eigen::Matrix4d& t) {
  transform_to_next_frame_ = t;
}

template <typename PointType>
void FrameBase<PointType>::SetTransformFromLast(const Eigen::Matrix4d& t) {
  transform_from_last_frame_ = t;
}

template <typename PointType>
inline Eigen::Matrix4d FrameBase<PointType>::TransformToNext() const {
  return transform_to_next_frame_;
}

template <typename PointType>
inline Eigen::Matrix4d FrameBase<PointType>::TransformFromLast() const {
  return transform_from_last_frame_;
}

template <typename PointType>
inline void FrameBase<PointType>::SetRelatedGpsInENU(const EnuPosition& enu) {
  related_enu_ = enu;
}

template <typename PointType>
inline EnuPosition FrameBase<PointType>::GetRelatedGpsInENU() {
  CHECK(HasGps());
  return related_enu_.value();
}

template <typename PointType>
inline bool FrameBase<PointType>::HasGps() const {
  return related_enu_.is_initialized();
}

template <typename PointType>
inline void FrameBase<PointType>::SetRelatedOdom(const OdomPose& odom) {
  related_odom_ = odom;
}

template <typename PointType>
inline OdomPose FrameBase<PointType>::GetRelatedOdom() const {
  CHECK(HasOdom());
  return related_odom_.value();
}

template <typename PointType>
inline bool FrameBase<PointType>::HasOdom() const {
  return related_odom_.is_initialized();
}

template <typename PointType>
inline typename descriptor::M2dp<PointType>::Descriptor
FrameBase<PointType>::GetDescriptor() const {
  return descriptor_;
}

template <typename PointType>
inline void FrameBase<PointType>::SetDescriptor(
    const typename descriptor::M2dp<PointType>::Descriptor& d) {
  descriptor_ = d;
}

template <typename PointType>
inline void FrameBase<PointType>::CalculateDescriptor() {
  descriptor::M2dp<PointType> m2dp;
  if (m2dp.setInputCloud(inner_cloud_->GetPclCloud())) {
    descriptor_ = m2dp.getFinalDescriptor();
  } else {
    PRINT_ERROR("did not get a descriptor for the frame.");
  }
}

template <typename PointType>
inline void FrameBase<PointType>::SetCloud(
    typename FrameBase<PointType>::InnerCloudPtr cloud) {
  inner_cloud_ = cloud;
}

template <typename PointType>
inline void FrameBase<PointType>::ClearCloud() {
  if (inner_cloud_) {
    inner_cloud_->Clear();
  }
}

template <typename PointType>
void FrameBase<PointType>::ToPcdFile(const std::string& filename) {
  if (!inner_cloud_ || inner_cloud_->Empty()) {
    return;
  }
  const std::string actual_filename =
      filename.empty() ? "simple_frame.pcd" : filename;
  pcl::io::savePCDFileBinaryCompressed(actual_filename,
                                       *(inner_cloud_->GetPclCloud()));
}

}  // namespace static_map

#endif  // BUILDER_FRAME_BASE_IMPL_H_
