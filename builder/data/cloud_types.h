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
#include <fstream>
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

/// @struct InnerPointType
/// We use this class to isolate inner class from pcl point type.
struct InnerPointType {
  float x = 0.;
  float y = 0.;
  float z = 0.;
  float intensity = 0.;
  float factor = 0.;

  // TODO(edward) Serialize using 3rd party lib.
  int Serialize(std::fstream *stream) const;
  int Deserialize(std::fstream *stream);
};

InnerPointType TransformPoint(const Eigen::Matrix4d &transform,
                              const InnerPointType &point);
struct InnerCloudType {
  SimpleTime stamp;
  std::vector<InnerPointType> points;

  using Ptr = std::shared_ptr<InnerCloudType>;
  using UniquePtr = std::unique_ptr<InnerCloudType>;
  using ConstPtr = std::shared_ptr<const InnerCloudType>;

  int Serialize(std::fstream *stream) const;
  int Deserialize(std::fstream *stream);

  void ApplyTransformInplace(const Eigen::Matrix4d &transform);
  void ApplyTransformToOutput(const Eigen::Matrix4d &transform,
                              InnerCloudType *const output) const;

  InnerCloudType &operator+=(const InnerCloudType &b);
};

static inline InnerPointType ToInnerPoint(const pcl::PointXYZ &point) {
  return InnerPointType{
      .x = point.x, .y = point.y, .z = point.z, .intensity = 0., .factor = 0.};
}

static inline InnerPointType ToInnerPoint(const pcl::PointXYZI &point) {
  return InnerPointType{.x = point.x,
                        .y = point.y,
                        .z = point.z,
                        .intensity = point.intensity,
                        .factor = 0.};
}

static inline void ToPclPoint(const InnerPointType &point,
                              pcl::PointXYZ *pcl_point) {
  CHECK(pcl_point);
  pcl_point->x = point.x;
  pcl_point->y = point.y;
  pcl_point->z = point.z;
}

static inline void ToPclPoint(const InnerPointType &point,
                              pcl::PointXYZI *pcl_point) {
  CHECK(pcl_point);
  pcl_point->x = point.x;
  pcl_point->y = point.y;
  pcl_point->z = point.z;
  pcl_point->intensity = point.intensity;
}

template <typename PointT>
InnerCloudType::Ptr ToInnerPointCloud(const pcl::PointCloud<PointT> &cloud);

template <typename PointT>
void ToPclPointCloud(const InnerCloudType &cloud,
                     pcl::PointCloud<PointT> *pcl_cloud);

///
/// @class EigenPointCloud
/// @brief Use eigen matrixes to store the points and normals. for the
/// convenience of calculation.
///
class EigenPointCloud {
 public:
  using Ptr = std::shared_ptr<EigenPointCloud>;
  using ConstPtr = std::shared_ptr<const EigenPointCloud>;

  EigenPointCloud() = default;
  explicit EigenPointCloud(const std::vector<InnerPointType> &cloud);

  /// @brief FromPointCloud: Initialise from a pcl cloud.
  void FromPointCloud(const std::vector<InnerPointType> &cloud);
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

///
/// @class InnerPointCloudData
/// @brief This template class is used for all registrators. It contains pcl
/// cloud and eigen cloud.
///
class InnerPointCloudData {
 public:
  using Ptr = std::shared_ptr<InnerPointCloudData>;

  explicit InnerPointCloudData(const InnerCloudType::Ptr cloud);

  /// @brief SetInnerCloud: The function will take care of all members inside,
  /// including time, pcl_cloud, eigen_cloud, inner_cloud.
  void SetInnerCloud(const InnerCloudType::Ptr cloud);
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
  EigenPointCloud::Ptr GetEigenCloud() const;
  SimpleTime GetTime() const;
  InnerCloudType::Ptr GetInnerCloud() const;

 private:
  void SetInnerCloudImpl(const InnerCloudType::Ptr cloud);
  bool EmptyImpl() const;

 private:
  common::ReadWriteMutex mutex_;

  InnerCloudType::Ptr inner_cloud_;
  EigenPointCloud::Ptr eigen_cloud_;
  SimpleTime time_;
  std::string filename_;
  std::atomic<bool> is_cloud_in_memory_;

  PROHIBIT_COPY_AND_ASSIGN(InnerPointCloudData);
};

}  // namespace data
}  // namespace static_map

#endif  // BUILDER_DATA_CLOUD_TYPES_H_
