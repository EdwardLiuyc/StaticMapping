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

#include "builder/data/cloud_types.h"

#include <algorithm>
#include <utility>

#include "common/macro_defines.h"
#include "common/math.h"
#include "common/performance/simple_prof.h"
#include "pcl/io/pcd_io.h"

namespace static_map {
namespace data {

namespace {
constexpr int kDim = 3;
constexpr int kNormalEstimationKnn = 7;

inline size_t ArgMax(const Eigen::Vector3d& v) {
  // FIXME: Change that to use the new API. the new Eigen API (3.2.8) allows
  // this with the call maxCoeff. See the section Visitors in
  // https://eigen.tuxfamily.org/dox/group__TutorialReductionsVisitorsBroadcasting.html
  // @todo(edward) use eigen api
  const int size(v.size());
  double maxVal(0.);
  size_t maxIdx(0);
  for (int i = 0; i < size; ++i) {
    if (v[i] > maxVal) {
      maxVal = v[i];
      maxIdx = i;
    }
  }
  return maxIdx;
}

struct CompareDim {
  const int dim;
  const EigenPointCloud& cloud;
  CompareDim(const int dim, const EigenPointCloud& eigen_point_cloud)
      : dim(dim), cloud(eigen_point_cloud) {}
  bool operator()(const int& p0, const int& p1) {
    return cloud.points(dim, p0) < cloud.points(dim, p1);
  }
};

// Refer to
//   "Fast and Accurate Computation of Surface Normals from Range Images"
//   H. Badino, D. Huber, Y. Park and T. Kanade
//   IV.C. "Unconstrained Least Squares" part
// TODO(yongchuan) Try more normal estimation methods.
void CalculateNormals(EigenPointCloud* const data, const int first,
                      const int last) {
  REGISTER_FUNC;
  const int col_count = last - first;
  // build nearest neighbors list
  Eigen::MatrixXd d(kDim, col_count);
  Eigen::MatrixXd M_wave = Eigen::Matrix<double, kDim, kDim>::Zero();
  for (int i = 0; i < col_count; ++i) {
    d.col(i) = data->points.block(0, data->indices[first + i], kDim, 1);
    M_wave += d.col(i) * d.col(i).transpose();
  }
  const Eigen::VectorXd b_wave = d.rowwise().sum();
  const Eigen::VectorXd mean = b_wave / col_count;
  const Eigen::MatrixXd NN = (d.colwise() - mean);

  // compute covariance
  const Eigen::MatrixXd C(NN * NN.transpose());
  if (C.fullPivHouseholderQr().rank() + 1 < kDim) {
    return;
  }

  Eigen::VectorXd normal = M_wave.inverse() * Eigen::VectorXd(b_wave);

  // sampling in a simple way
  // use just one point in a box
  const int k = data->indices[first];
  // Mark the indices which will be part of the final data
  data->indices_to_keep.push_back(k);
  data->points.col(k) = mean;
  data->normals.col(k) = normal.normalized();
}

void BuildNormals(EigenPointCloud* const data, const int first, const int last,
                  Eigen::Vector3d&& minValues, Eigen::Vector3d&& maxValues) {
  const int count(last - first);
  if (count <= kNormalEstimationKnn) {
    // compute for this range
    CalculateNormals(data, first, last);
    return;
  }

  // find the largest dimension of the box
  const int cut_dim = ArgMax(maxValues - minValues);

  // compute number of elements
  const int rightCount(count / 2);
  const int leftCount(count - rightCount);
  CHECK_EQ(last - rightCount, first + leftCount);

  // sort, hack std::nth_element
  std::nth_element(data->indices.begin() + first,
                   data->indices.begin() + first + leftCount,
                   data->indices.begin() + last, CompareDim(cut_dim, *data));

  // get value
  const int cut_index(data->indices[first + leftCount]);
  const double cutVal(data->points(cut_dim, cut_index));

  // update bounds for left
  Eigen::Vector3d leftMaxValues(maxValues);
  leftMaxValues[cut_dim] = cutVal;
  // update bounds for right
  Eigen::Vector3d rightMinValues(minValues);
  rightMinValues[cut_dim] = cutVal;

  // recurse
  BuildNormals(data, first, first + leftCount,
               std::forward<Eigen::Vector3d>(minValues),
               std::move(leftMaxValues));
  BuildNormals(data, first + leftCount, last, std::move(rightMinValues),
               std::forward<Eigen::Vector3d>(maxValues));
}
}  // namespace

int InnerPointType::Serialize(std::fstream* stream) const {
  if (stream && stream->is_open()) {
    stream->write(reinterpret_cast<const char*>(this), sizeof(InnerPointType));
    return sizeof(InnerPointType);
  }
  return -1;
}

int InnerPointType::Deserialize(std::fstream* stream) {
  if (stream && stream->is_open()) {
    stream->read(reinterpret_cast<char*>(this), sizeof(InnerPointType));
    if (stream->good() || stream->eof()) {
      return sizeof(InnerPointType);
    } else {
      return -1;
    }
  }
  return -1;
}

InnerPointType TransformPoint(const Eigen::Matrix4d& transform,
                              const InnerPointType& point) {
  Eigen::Vector4f homo_point;
  homo_point << point.x, point.y, point.z, 1.;
  const Eigen::Vector4f transformed_point =
      transform.cast<float>() * homo_point;
  return InnerPointType{.x = transformed_point[0],
                        .y = transformed_point[1],
                        .z = transformed_point[2],
                        .intensity = point.intensity,
                        .factor = point.factor};
}

int InnerCloudType::Serialize(std::fstream* stream) const {
  if (stream && stream->is_open()) {
    // Write time.
    stream->write(reinterpret_cast<const char*>(&stamp), sizeof(SimpleTime));
    // Write size.
    size_t size = points.size();
    stream->write(reinterpret_cast<const char*>(&size), sizeof(size_t));
    // Write points.
    for (const auto& point : points) {
      point.Serialize(stream);
    }
    return 0;
  }
  return -1;
}

int InnerCloudType::Deserialize(std::fstream* stream) {
  if (stream && stream->is_open()) {
    // Read time.
    stream->read(reinterpret_cast<char*>(&stamp), sizeof(SimpleTime));
    if (!stream->good()) {
      return -1;
    }
    // Read size.
    size_t size = 0;
    stream->read(reinterpret_cast<char*>(&size), sizeof(size_t));
    if (!stream->good()) {
      return -1;
    }
    // Read points.
    points.resize(size);
    for (size_t i = 0; i < size; ++i) {
      if (points[i].Deserialize(stream) < 0) {
        return -1;
      }
    }
    return 0;
  }
  return -1;
}

void InnerCloudType::ApplyTransformInplace(const Eigen::Matrix4d& transform) {
  for (auto& point : points) {
    point = TransformPoint(transform, point);
  }
}

void InnerCloudType::ApplyTransformToOutput(
    const Eigen::Matrix4d& transform, InnerCloudType* const output) const {
  CHECK(output);
  output->stamp = stamp;
  output->points.clear();
  output->points.reserve(points.size());
  for (const auto& point : points) {
    output->points.push_back(TransformPoint(transform, point));
  }
}

InnerCloudType& InnerCloudType::operator+=(const InnerCloudType& b) {
  for (const auto& point : b.points) {
    this->points.push_back(point);
  }
  return *this;
}

template <typename PointT>
InnerCloudType::Ptr ToInnerPointCloud(const pcl::PointCloud<PointT>& cloud) {
  // Set time stamp
  auto inner_cloud = std::make_shared<InnerCloudType>();
  inner_cloud->stamp = ToLocalTime(cloud.header.stamp);
  // Set points.
  if (!cloud.empty()) {
    inner_cloud->points.reserve(cloud.size());
    for (const auto& point : cloud) {
      inner_cloud->points.push_back(ToInnerPoint(point));
    }
  }
  return inner_cloud;
}

template <typename PointT>
void ToPclPointCloud(const InnerCloudType& cloud,
                     pcl::PointCloud<PointT>* pcl_cloud) {
  CHECK(pcl_cloud);
  pcl_cloud->header.stamp = ToPclTime(cloud.stamp);
  pcl_cloud->clear();

  if (!cloud.points.empty()) {
    pcl_cloud->points.reserve(cloud.points.size());
    for (const auto& point : cloud.points) {
      PointT pcl_point;
      ToPclPoint(point, &pcl_point);
      pcl_cloud->points.push_back(pcl_point);
    }
  }
}

void EigenPointCloud::ApplyTransform(const Eigen::Matrix4d& transform) {
  const int points_count = points.cols();
  CHECK_GT(points_count, 0);
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> homo_data(4,
                                                                  points_count);
  homo_data.setConstant(1.);
  homo_data.block(0, 0, 3, points_count) = points;
  points = (transform * homo_data).block(0, 0, 3, points_count);

  if (normals.cols() > 0) {
    CHECK_EQ(normals.cols(), points_count);
    homo_data.block(0, 0, 3, points_count) = normals;
    normals = (transform * homo_data).block(0, 0, 3, points_count);
  }
}

bool EigenPointCloud::HasNormals() const { return normals.cols() > 0; }

void EigenPointCloud::ApplyMotionCompensation(
    const Eigen::Matrix4d& transform) {
  const size_t cloud_size = points.cols();
  CHECK_EQ(cloud_size, factors.size());

  for (size_t i = 0; i < cloud_size; ++i) {
    const Eigen::Matrix4d transform = common::InterpolateTransform(
        Eigen::Matrix4d::Identity().eval(), transform, factors[i]);
    const Eigen::Vector3d new_point_start =
        transform.block(0, 0, 3, 3) * points.col(i).topRows(3) +
        transform.block(0, 3, 3, 1);
    points.col(i).topRows(3) = new_point_start;
  }

  // TODO(edward) Maybe the normals need transform too.
}

EigenPointCloud::EigenPointCloud(
    const std::vector<InnerPointType>& inner_points) {
  FromPointCloud(inner_points);
}

void EigenPointCloud::FromPointCloud(
    const std::vector<InnerPointType>& inner_points) {
  CHECK(!inner_points.empty());

  const int size = inner_points.size();
  indices.resize(size);
  factors.resize(size);
  // We assume points are in 3d by default.
  points.resize(3, size);
  // We leave normals not inited, because we will need the normals only if we
  // use the cloud as a target, so initialize them later.

  for (int i = 0; i < size; ++i) {
    indices[i] = i;
    factors[i] = static_cast<double>(i) / size;
    points.col(i) << inner_points[i].x, inner_points[i].y, inner_points[i].z;
  }
}

void EigenPointCloud::CalculateNormals() {
  const int points_num = points.cols();
  normals.resize(kDim, points_num);

  // step1 normal estimation and filtering
  BuildNormals(this, 0, points_num, points.rowwise().minCoeff(),
               points.rowwise().maxCoeff());

  // step2 remove useless points
  // Bring the data we keep to the front of the arrays then
  // wipe the leftover unused space.
  std::sort(indices_to_keep.begin(), indices_to_keep.end());
  const int points_count_remained = indices_to_keep.size();
  for (int i = 0; i < points_count_remained; ++i) {
    const int k = indices_to_keep[i];
    CHECK(i <= k);
    points.col(i) = points.col(k);
    normals.col(i) = normals.col(k);
  }
  points.conservativeResize(Eigen::NoChange, points_count_remained);
  normals.conservativeResize(Eigen::NoChange, points_count_remained);
}

InnerPointCloudData::InnerPointCloudData(const InnerCloudType::Ptr cloud) {
  SetInnerCloud(cloud);
}

bool InnerPointCloudData::Empty() {
  common::ReadMutexLocker locker(mutex_);
  return EmptyImpl();
}

bool InnerPointCloudData::EmptyImpl() const {
  return inner_cloud_ == nullptr || inner_cloud_->points.empty();
}

void InnerPointCloudData::Clear() {
  if (!(is_cloud_in_memory_.load())) {
    return;
  }
  boost::upgrade_lock<common::ReadWriteMutex> locker(mutex_);
  common::WriteMutexLocker write_locker(locker);
  // TODO(edward) reset or just clear the inner points.
  eigen_cloud_.reset(new EigenPointCloud);
  inner_cloud_.reset(new InnerCloudType);
  is_cloud_in_memory_ = false;
}

void InnerPointCloudData::TransformCloud(const Eigen::Matrix4d& T) {
  boost::upgrade_lock<common::ReadWriteMutex> locker(mutex_);
  common::WriteMutexLocker write_locker(locker);
  if (!EmptyImpl()) {
    inner_cloud_->ApplyTransformInplace(T);
  }
  if (eigen_cloud_) {
    eigen_cloud_->ApplyTransform(T);
  }
}

void InnerPointCloudData::CalculateNormals() {
  // Use this carefully, this function is not locked.
  CHECK(eigen_cloud_);
  eigen_cloud_->CalculateNormals();
}

void InnerPointCloudData::SetInnerCloud(const InnerCloudType::Ptr cloud) {
  boost::upgrade_lock<common::ReadWriteMutex> locker(mutex_);
  common::WriteMutexLocker write_locker(locker);
  SetInnerCloudImpl(cloud);
}

void InnerPointCloudData::SetInnerCloudImpl(const InnerCloudType::Ptr cloud) {
  CHECK(cloud);
  inner_cloud_ = cloud;
  // Reset eigen cloud
  eigen_cloud_.reset(new EigenPointCloud(inner_cloud_->points));
  // Reset time stamp
  time_ = inner_cloud_->stamp;
  is_cloud_in_memory_ = true;
}

bool InnerPointCloudData::SaveToFile(const std::string& filename) {
  common::ReadMutexLocker locker(mutex_);
  CHECK(!filename.empty());
  CHECK(!EmptyImpl());
  filename_ = filename;

  std::fstream of(filename_, std::ios::out | std::ios::binary);
  CHECK(of.is_open()) << "Could not open file: " << filename_;
  CHECK_GE(inner_cloud_->Serialize(&of), 0);
  of.close();
  return true;
}

bool InnerPointCloudData::CloudInMemory() {
  boost::upgrade_lock<common::ReadWriteMutex> locker(mutex_);
  if (!is_cloud_in_memory_.load()) {
    common::WriteMutexLocker write_locker(locker);

    InnerCloudType::Ptr inner_cloud(new InnerCloudType);
    std::fstream bin_fs(filename_, std::ios::in | std::ios::binary);
    CHECK(bin_fs.is_open()) << "Could not open file: " << filename_;
    inner_cloud->Deserialize(&bin_fs);
    bin_fs.close();

    SetInnerCloudImpl(inner_cloud);
    is_cloud_in_memory_ = true;
    // PRINT_DEBUG("get submap data from disk.");
  }
  return is_cloud_in_memory_.load();
}

std::shared_ptr<EigenPointCloud> InnerPointCloudData::GetEigenCloud() const {
  return eigen_cloud_;
}

SimpleTime InnerPointCloudData::GetTime() const { return time_; }
InnerCloudType::Ptr InnerPointCloudData::GetInnerCloud() const {
  return inner_cloud_;
}

template InnerCloudType::Ptr ToInnerPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>& cloud);
template InnerCloudType::Ptr ToInnerPointCloud(
    const pcl::PointCloud<pcl::PointXYZI>& cloud);

template void ToPclPointCloud(const InnerCloudType& cloud,
                              pcl::PointCloud<pcl::PointXYZ>* pcl_cloud);
template void ToPclPointCloud(const InnerCloudType& cloud,
                              pcl::PointCloud<pcl::PointXYZI>* pcl_cloud);

}  // namespace data
}  // namespace static_map
