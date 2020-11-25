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

#ifndef BUILDER_DATA_CLOUD_TYPES_H_
#define BUILDER_DATA_CLOUD_TYPES_H_

#include <atomic>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "Eigen/Eigen"
#include "common/mutex.h"
#include "common/simple_time.h"
#include "glog/logging.h"
#include "pcl/common/transforms.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace static_map {
namespace data {

///
/// @class EigenPointCloud
/// @brief Use eigen matrixes to store the points and normals. for the
/// convenience of calculation.
///
class EigenPointCloud {
 public:
  using Ptr = std::shared_ptr<EigenPointCloud>;
  using ConstPtr = std::shared_ptr<EigenPointCloud const>;

  EigenPointCloud() = default;

  template <typename PointType>
  explicit EigenPointCloud(
      const typename pcl::PointCloud<PointType>::Ptr &cloud);

  /// @brief FromPointCloud: Initialise from a pcl cloud.
  template <typename PointType>
  void FromPointCloud(const typename pcl::PointCloud<PointType>::Ptr &cloud);
  /// @brief CalculateNormals: Normal estimation.
  void CalculateNormals();
  /// @brief HasNormals: Check if normals calculated.
  bool HasNormals() const;
  /// @brief ApplyTransform: Transform the whole point cloud and normals.
  void ApplyTransform(const Eigen::Matrix4d &transform);
  /// @brief ApplyMotionCompensation: Simlidar above, the difference is that we
  /// apply different transform on each point.
  void ApplyMotionCompensation(const Eigen::Matrix4d &transform);

 public:
  std::vector<int> indices;
  std::vector<int> indices_to_keep;
  std::vector<double> factors;
  Eigen::MatrixXd points;
  Eigen::MatrixXd normals;
};

template <typename PointType>
EigenPointCloud::EigenPointCloud(
    const typename pcl::PointCloud<PointType>::Ptr &cloud) {
  FromPointCloud(cloud);
}

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

///
/// @class InnerPointCloudData
/// @brief This template class is used for all registrators. It contains pcl
/// cloud and eigen cloud.
///
template <typename PointT>
class InnerPointCloudData {
 public:
  using PclCloudType = pcl::PointCloud<PointT>;
  using PclCloudPtr = typename PclCloudType::Ptr;
  using Ptr = std::shared_ptr<InnerPointCloudData<PointT>>;

  explicit InnerPointCloudData(const PclCloudPtr cloud);

  /// @brief SetPclCloud: The function will take care of all members inside,
  /// including time, pcl_cloud, eigen_cloud.
  void SetPclCloud(const PclCloudPtr cloud);
  /// @brief Empty: return if the inner pcl_cloud is nullptr or empty.
  bool Empty();
  /// @brief Clear: remove all data.
  void Clear();
  /// @brief TransformCloud: Transform pcl_cloud & eigen_cloud.
  void TransformCloud(const Eigen::Matrix4d &transform);
  /// @brief CalculateNormals: Calculate the normals in eigen_cloud.
  void CalculateNormals();
  /// @brief Save to pcd file.
  bool SaveToFile(const std::string &filename);
  /// @brief return whether the cloud is in mem, if not, load from file.
  bool CloudInMemory();

  /// Set Geters.
  PclCloudPtr GetPclCloud() const;
  EigenPointCloud::Ptr GetEigenCloud() const;
  SimpleTime GetTime() const;

 public:
  float delta_time_in_cloud;

 private:
  void SetPclCloudImpl(const PclCloudPtr cloud);

 private:
  common::ReadWriteMutex mutex_;

  PclCloudPtr pcl_cloud_;
  EigenPointCloud::Ptr eigen_cloud_;
  SimpleTime time_;
  std::string filename_;
  std::atomic<bool> is_cloud_in_memory_;

  PROHIBIT_COPY_AND_ASSIGN(InnerPointCloudData);
};

}  // namespace data
}  // namespace static_map

#endif  // BUILDER_DATA_CLOUD_TYPES_H_
