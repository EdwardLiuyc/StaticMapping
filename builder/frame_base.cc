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

#include "builder/frame_base.h"

#include <string>

namespace static_map {

Eigen::Matrix3d FrameBase::GlobalRotation() const {
  return Eigen::Matrix3d(global_pose_.block(0, 0, 3, 3));
}

Eigen::Vector3d FrameBase::GlobalTranslation() const {
  return Eigen::Vector3d(global_pose_.block(0, 3, 3, 1));
}

Eigen::Matrix3d FrameBase::LocalRotation() const {
  return Eigen::Matrix3d(local_pose_.block(0, 0, 3, 3));
}

Eigen::Vector3d FrameBase::LocalTranslation() const {
  return Eigen::Vector3d(local_pose_.block(0, 3, 3, 1));
}

Eigen::Matrix4d FrameBase::GlobalPose() const { return global_pose_; }

void FrameBase::SetGlobalPose(const Eigen::Matrix4d& t) {
  global_pose_ = t;
  common::NormalizeRotation(global_pose_);
}

Eigen::VectorXd FrameBase::GlobalPoseIn6Dof() const {
  return common::TransformToVector6(global_pose_);
}

Eigen::Matrix4d FrameBase::LocalPose() const { return local_pose_; }

void FrameBase::SetLocalPose(const Eigen::Matrix4d& t) {
  local_pose_ = t;
  common::NormalizeRotation(local_pose_);
}

void FrameBase::SetTransformToNext(const Eigen::Matrix4d& t) {
  transform_to_next_frame_ = t;
}

void FrameBase::SetTransformFromLast(const Eigen::Matrix4d& t) {
  transform_from_last_frame_ = t;
}

Eigen::Matrix4d FrameBase::TransformToNext() const {
  return transform_to_next_frame_;
}

Eigen::Matrix4d FrameBase::TransformFromLast() const {
  return transform_from_last_frame_;
}

void FrameBase::SetRelatedGpsInENU(const EnuPosition& enu) {
  related_enu_ = enu;
}

EnuPosition FrameBase::GetRelatedGpsInENU() {
  CHECK(HasGps());
  return related_enu_.value();
}

bool FrameBase::HasGps() const { return related_enu_.is_initialized(); }

void FrameBase::SetRelatedOdom(const OdomPose& odom) { related_odom_ = odom; }

OdomPose FrameBase::GetRelatedOdom() const {
  CHECK(HasOdom());
  return related_odom_.value();
}

bool FrameBase::HasOdom() const { return related_odom_.is_initialized(); }

descriptor::M2dp::Descriptor FrameBase::GetDescriptor() const {
  return descriptor_;
}

void FrameBase::SetDescriptor(const descriptor::M2dp::Descriptor& d) {
  descriptor_ = d;
}

void FrameBase::CalculateDescriptor() {
  descriptor::M2dp m2dp;
  if (m2dp.setInputCloud(inner_cloud_->GetInnerCloud())) {
    descriptor_ = m2dp.getFinalDescriptor();
  } else {
    PRINT_ERROR("did not get a descriptor for the frame.");
  }
}

void FrameBase::SetCloud(typename FrameBase::InnerCloudPtr cloud) {
  inner_cloud_ = cloud;
}

void FrameBase::ClearCloud() {
  if (inner_cloud_) {
    inner_cloud_->Clear();
  }
}

void FrameBase::ToPcdFile(const std::string& filename) {
  if (!inner_cloud_ || inner_cloud_->Empty()) {
    return;
  }
  // TODO(edward) Uniform this function for all child class.
  // const std::string actual_filename =
  //     filename.empty() ? "simple_frame.pcd" : filename;
  // pcl::io::savePCDFileBinaryCompressed(actual_filename,
  //                                      *(inner_cloud_->GetPclCloud()));
}

}  // namespace static_map
