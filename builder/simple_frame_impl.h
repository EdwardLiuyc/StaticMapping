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

#ifndef BUILDER_SIMPLE_FRAME_IMPL_H_
#define BUILDER_SIMPLE_FRAME_IMPL_H_

namespace static_map {

template <typename PointType>
inline Eigen::Matrix3f SimpleFrame<PointType>::GlobalRotation() const {
  return Eigen::Matrix3f(global_pose_.block(0, 0, 3, 3));
}
template <typename PointType>
inline Eigen::Vector3f SimpleFrame<PointType>::GlobalTranslation() const {
  return Eigen::Vector3f(global_pose_.block(0, 3, 3, 1));
}
template <typename PointType>
inline Eigen::Matrix3f SimpleFrame<PointType>::LocalRotation() const {
  return Eigen::Matrix3f(local_pose_.block(0, 0, 3, 3));
}
template <typename PointType>
inline Eigen::Vector3f SimpleFrame<PointType>::LocalTranslation() const {
  return Eigen::Vector3f(local_pose_.block(0, 3, 3, 1));
}
template <typename PointType>
inline Eigen::Matrix4f SimpleFrame<PointType>::GlobalPose() const {
  return global_pose_;
}
template <typename PointType>
inline void SimpleFrame<PointType>::SetGlobalPose(const Eigen::Matrix4f& t) {
  global_pose_ = t;
  common::NormalizeRotation(global_pose_);
}
template <typename PointType>
inline Eigen::VectorXf SimpleFrame<PointType>::GlobalPoseIn6Dof() const {
  return common::TransformToVector6(global_pose_);
}
template <typename PointType>
inline Eigen::Matrix4f SimpleFrame<PointType>::LocalPose() const {
  return local_pose_;
}
template <typename PointType>
inline void SimpleFrame<PointType>::SetLocalPose(const Eigen::Matrix4f& t) {
  local_pose_ = t;
  common::NormalizeRotation(local_pose_);
}

template <typename PointType>
void SimpleFrame<PointType>::SetTransformToNext(const Eigen::Matrix4f& t) {
  transform_to_next_frame_ = t;
}

template <typename PointType>
void SimpleFrame<PointType>::SetTransformFromLast(const Eigen::Matrix4f& t) {
  transform_from_last_frame_ = t;
}

template <typename PointType>
inline Eigen::Matrix4f SimpleFrame<PointType>::TransformToNext() const {
  return transform_to_next_frame_;
}

template <typename PointType>
inline Eigen::Matrix4f SimpleFrame<PointType>::TransformFromLast() const {
  return transform_from_last_frame_;
}

template <typename PointType>
inline void SimpleFrame<PointType>::SetRelatedGpsInENU(const EnuPosition& enu) {
  related_enu_ = enu;
}

template <typename PointType>
inline EnuPosition SimpleFrame<PointType>::GetRelatedGpsInENU() {
  CHECK(HasGps());
  return related_enu_.value();
}

template <typename PointType>
inline bool SimpleFrame<PointType>::HasGps() const {
  return related_enu_.is_initialized();
}

template <typename PointType>
inline void SimpleFrame<PointType>::SetRelatedOdom(const OdomPose& odom) {
  related_odom_ = odom;
  got_related_odom_ = true;
}

template <typename PointType>
inline OdomPose SimpleFrame<PointType>::GetRelatedOdom() const {
  return related_odom_;
}

template <typename PointType>
inline bool SimpleFrame<PointType>::HasOdom() const {
  return got_related_odom_;
}

template <typename PointType>
inline typename descriptor::M2dp<PointType>::Descriptor
SimpleFrame<PointType>::GetDescriptor() const {
  return descriptor_;
}

template <typename PointType>
inline void SimpleFrame<PointType>::SetDescriptor(
    const typename descriptor::M2dp<PointType>::Descriptor& d) {
  descriptor_ = d;
}

template <typename PointType>
inline void SimpleFrame<PointType>::CalculateDescriptor() {
  descriptor::M2dp<PointType> m2dp;
  if (m2dp.setInputCloud(cloud_)) {
    descriptor_ = m2dp.getFinalDescriptor();
  } else {
    PRINT_ERROR("did not get a descriptor for the frame.");
  }
}

template <typename PointType>
inline void SimpleFrame<PointType>::SetCloud(const PointCloudPtr& cloud) {
  cloud_ = cloud;
}

template <typename PointType>
inline void SimpleFrame<PointType>::ClearCloud() {
  if (cloud_) {
    cloud_->clear();
    cloud_->points.shrink_to_fit();
  }
}

template <typename PointType>
void SimpleFrame<PointType>::ToPcdFile(const std::string& filename) {
  if (cloud_ == nullptr || cloud_->empty()) {
    return;
  }
  if (!filename.empty()) {
    pcl::io::savePCDFileBinaryCompressed(filename, *cloud_);
  } else {
    pcl::io::savePCDFileBinaryCompressed("simple_frame.pcd", *cloud_);
  }
}

}  // namespace static_map

#endif  // BUILDER_SIMPLE_FRAME_IMPL_H_
