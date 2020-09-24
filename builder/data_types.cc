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

#include "builder/data_types.h"

#include <algorithm>
#include <utility>

#include "common/math.h"
#include "common/performance/simple_prof.h"

namespace static_map {
namespace sensors {
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

Eigen::Matrix4d OdomMsg::PoseInMatrix() const {
  Eigen::Matrix4d pose_in_matrix = Eigen::Matrix4d::Identity();
  pose_in_matrix.block(0, 3, 3, 1) = pose.pose.position;
  pose_in_matrix.block(0, 0, 3, 3) = pose.pose.orientation.toRotationMatrix();
  return pose_in_matrix;
}

void OdomMsg::SetPose(const Eigen::Matrix4d& pose_mat) {
  pose.pose.position = pose_mat.block(0, 3, 3, 1);
  pose.pose.orientation =
      Eigen::Quaterniond(Eigen::Matrix3d(pose_mat.block(0, 0, 3, 3)));
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

}  // namespace sensors
}  // namespace static_map
