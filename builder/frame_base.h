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

#ifndef BUILDER_FRAME_BASE_H_
#define BUILDER_FRAME_BASE_H_
// third_party
#include <Eigen/Eigen>
// stl
#include <memory>
#include <string>
// local
#include "boost/optional.hpp"
#include "builder/data_types.h"
#include "common/math.h"
#include "common/simple_time.h"
#include "descriptor/m2dp.h"
#include "glog/logging.h"
#include "registrators/interface.h"

namespace static_map {

using EnuPosition = Eigen::Vector3d;
using OdomPose = Eigen::Matrix4d;

/*
 * @class FrameBase
 * @brief base class for all frames, such as frame, submap,
 * a simple frame has its global pose(in map), local pose, cloud and
 * descriptor
 */
template <typename PointType>
class FrameBase {
 public:
  using PointCloudType = pcl::PointCloud<PointType>;
  using PointCloudPtr = typename PointCloudType::Ptr;
  using PointCloudConstPtr = typename PointCloudType::ConstPtr;
  using InnerCloudPtr = typename sensors::InnerPointCloudData<PointType>::Ptr;

  FrameBase()
      : global_pose_(Eigen::Matrix4d::Identity()),
        local_pose_(Eigen::Matrix4d::Identity()),
        transform_from_last_frame_(Eigen::Matrix4d::Identity()),
        transform_to_next_frame_(Eigen::Matrix4d::Identity()) {}

  virtual ~FrameBase() = default;

  Eigen::Matrix3d GlobalRotation() const;
  Eigen::Vector3d GlobalTranslation() const;
  Eigen::Matrix3d LocalRotation() const;
  Eigen::Vector3d LocalTranslation() const;

  Eigen::VectorXd GlobalPoseIn6Dof() const;
  Eigen::Matrix4d GlobalPose() const;
  Eigen::Matrix4d LocalPose() const;

  void SetGlobalPose(const Eigen::Matrix4d& t);
  void SetLocalPose(const Eigen::Matrix4d& t);

  // cloud
  virtual InnerCloudPtr Cloud() = 0;
  virtual void ToPcdFile(const std::string& filename);
  void SetCloud(InnerCloudPtr cloud);
  void ClearCloud();

  // descriptor (m2dp)
  typename descriptor::M2dp<PointType>::Descriptor GetDescriptor() const;
  void SetDescriptor(const typename descriptor::M2dp<PointType>::Descriptor& d);
  void CalculateDescriptor();

  // transform between last&next
  virtual void SetTransformToNext(const Eigen::Matrix4d& t);
  virtual void SetTransformFromLast(const Eigen::Matrix4d& t);
  Eigen::Matrix4d TransformToNext() const;
  Eigen::Matrix4d TransformFromLast() const;

  // enu
  void SetRelatedGpsInENU(const EnuPosition& enu);
  EnuPosition GetRelatedGpsInENU();
  bool HasGps() const;

  // odom
  void SetRelatedOdom(const OdomPose& odom);
  OdomPose GetRelatedOdom() const;
  bool HasOdom() const;

  inline SimpleTime GetTimeStamp() { return stamp_; }
  inline void SetTimeStamp(const SimpleTime& time) { stamp_ = time; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  Eigen::Matrix4d global_pose_;
  Eigen::Matrix4d local_pose_;
  Eigen::Matrix4d transform_from_last_frame_;
  Eigen::Matrix4d transform_to_next_frame_;
  InnerCloudPtr inner_cloud_;
  SimpleTime stamp_;
  typename descriptor::M2dp<PointType>::Descriptor descriptor_;

  boost::optional<EnuPosition> related_enu_;
  boost::optional<OdomPose> related_odom_;
};

}  // namespace static_map

#include "builder/frame_base_impl.h"

#endif  // BUILDER_FRAME_BASE_H_
