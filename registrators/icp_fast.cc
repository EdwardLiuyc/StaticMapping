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

#include "registrators/icp_fast.h"

#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <limits>
#include <utility>
#include <vector>

#include <boost/typeof/typeof.hpp>

#include "common/macro_defines.h"
#include "common/math.h"
#include "common/performance/simple_prof.h"

namespace static_map {
namespace registrator {

constexpr int kNormalEstimationKnn = 7;
constexpr int kDim = 3;

using Matrix = Eigen::MatrixXd;
using Vector = Eigen::VectorXd;
using OutlierWeights = Matrix;

struct BuildData {
  std::vector<int> indices;
  std::vector<int> indices_to_keep;
  std::vector<double> factors;
  Matrix points;
  Matrix normals;

  template <typename PointType>
  void FromPointCloud(const typename pcl::PointCloud<PointType>::Ptr& cloud) {
    CHECK(cloud && !cloud->empty());

    const int size = cloud->size();
    indices.resize(size);
    factors.resize(size);
    points.resize(kDim, size);
    normals.resize(kDim, size);
    for (int i = 0; i < size; ++i) {
      indices[i] = i;
      factors[i] = static_cast<double>(i) / size;
      points.col(i) << cloud->points[i].x, cloud->points[i].y,
          cloud->points[i].z;
    }
  }
};

void MotionCompensation(const Eigen::Matrix4d& delta_transform,
                        BuildData* const in_output_cloud) {
  const size_t cloud_size = in_output_cloud->points.cols();
  CHECK_EQ(cloud_size, in_output_cloud->factors.size());

  for (size_t i = 0; i < cloud_size; ++i) {
    const Eigen::Matrix4d transform = common::InterpolateTransform(
        Eigen::Matrix4d::Identity().eval(), delta_transform,
        in_output_cloud->factors[i]);

    const Eigen::Vector3d new_point_start =
        transform.block(0, 0, 3, 3) *
            in_output_cloud->points.col(i).topRows(3) +
        transform.block(0, 3, 3, 1);

    in_output_cloud->points.col(i).topRows(3) = new_point_start;
  }
}

struct Matches {
  //!< Squared distances to closest points, dense matrix
  using Dists = Matrix;
  //!< Identifiers of closest points, dense matrix of integers
  using Ids = Eigen::MatrixXi;

  Matches() = default;
  Matches(const int knn, const int points_count)
      : dists(Dists(knn, points_count)), ids(Ids(knn, points_count)) {}

  //!< squared distances to closest points
  Dists dists;
  //!< identifiers of closest points
  Ids ids;

  double GetDistsQuantile(const double quantile) const {
    // build array
    CHECK(quantile >= 0.0 && quantile <= 1.0);
    std::vector<double> values;
    values.reserve(dists.rows() * dists.cols());
    for (int x = 0; x < dists.cols(); ++x) {
      for (int y = 0; y < dists.rows(); ++y) {
        if (dists(y, x) != std::numeric_limits<double>::infinity()) {
          values.push_back(dists(y, x));
        }
      }
    }

    CHECK(!values.empty());
    // get quantile
    if (quantile == 1.0) {
      return *std::max_element(values.begin(), values.end());
    }
    const int quantile_index = values.size() * quantile;
    std::nth_element(values.begin(), values.begin() + quantile_index,
                     values.end());
    return values[quantile_index];
  }
};

struct ErrorElements {
  BuildData reading;       //!< reading point cloud
  BuildData reference;     //!< reference point cloud
  OutlierWeights weights;  //!< weights for every association
  Matches matches;         //!< associations
  int nbRejectedMatches;   //!< number of matches with zero weights
  int nbRejectedPoints;    //!< number of points with all matches set to zero
                           //!< weights
  double pointUsedRatio;   //!< the ratio of how many points were used for error
                           //!< minimization
  double weightedPointUsedRatio;  //!< the ratio of how many points were used
                                  //!< (with weight) for error minimization

  ErrorElements();
  ErrorElements(const BuildData& source_points, const BuildData& target_points,
                const OutlierWeights& weights, const Matches& matches) {
    CHECK_GT(matches.ids.rows(), 0);
    CHECK_GT(matches.ids.cols(), 0);
    CHECK(matches.ids.cols() == source_points.points.cols());  // nbpts
    CHECK(weights.rows() == matches.ids.rows());               // knn

    const int knn = weights.rows();
    const int dimPoints = source_points.points.rows();

    // Count points with no weights
    const int pointsCount = (weights.array() != 0.0).count();
    CHECK_GT(pointsCount, 0) << "no point to minimize";

    Matrix keptPoints(dimPoints, pointsCount);
    std::vector<double> kept_points_factor(pointsCount);
    Matches keptMatches(1, pointsCount);
    OutlierWeights keptWeights(1, pointsCount);

    int j = 0;
    int rejectedMatchCount = 0;
    int rejectedPointCount = 0;
    bool matchExist = false;
    this->weightedPointUsedRatio = 0;

    for (int i = 0; i < source_points.points.cols(); ++i) {
      matchExist = false;
      for (int k = 0; k < knn; k++) {
        const auto matchDist = matches.dists(k, i);
        if (matchDist == NNS::InvalidValue) {
          continue;
        }

        if (weights(k, i) != 0.0) {
          keptPoints.col(j) = source_points.points.col(i);
          kept_points_factor[j] = source_points.factors[i];
          keptMatches.ids(0, j) = matches.ids(k, i);
          keptMatches.dists(0, j) = matchDist;
          keptWeights(0, j) = weights(k, i);
          ++j;
          this->weightedPointUsedRatio += weights(k, i);
          matchExist = true;
        } else {
          rejectedMatchCount++;
        }
      }

      if (matchExist == false) {
        rejectedPointCount++;
      }
    }

    CHECK_EQ(j, pointsCount);

    this->pointUsedRatio =
        static_cast<double>(j) / (knn * source_points.points.cols());
    this->weightedPointUsedRatio /=
        static_cast<double>(knn * source_points.points.cols());

    CHECK_EQ(dimPoints, target_points.points.rows());
    const int dim_target_normals = target_points.normals.rows();
    Matrix associated_target_normals;
    if (dim_target_normals > 0) {
      associated_target_normals = Matrix(dim_target_normals, pointsCount);
    }

    Matrix associatedPoints(dimPoints, pointsCount);
    // Fetch matched points
    for (int i = 0; i < pointsCount; ++i) {
      const int refIndex(keptMatches.ids(i));
      associatedPoints.col(i) =
          target_points.points.block(0, refIndex, dimPoints, 1);

      if (dim_target_normals > 0)
        associated_target_normals.col(i) =
            target_points.normals.block(0, refIndex, dim_target_normals, 1);
    }

    // Copy final data to structure
    reading.points = keptPoints;
    reading.factors = std::move(kept_points_factor);
    reference.points = associatedPoints;
    reference.normals = associated_target_normals;

    this->weights = keptWeights;
    this->matches = keptMatches;
    this->nbRejectedMatches = rejectedMatchCount;
    this->nbRejectedPoints = rejectedPointCount;
  }
};

struct CompareDim {
  const int dim;
  const BuildData& buildData;
  CompareDim(const int dim, const BuildData& buildData)
      : dim(dim), buildData(buildData) {}
  bool operator()(const int& p0, const int& p1) {
    return buildData.points(dim, p0) < buildData.points(dim, p1);
  }
};

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

// Refer to
//   "Fast and Accurate Computation of Surface Normals from Range Images"
//   H. Badino, D. Huber, Y. Park and T. Kanade
//   IV.C. "Unconstrained Least Squares" part
// TODO(yongchuan) Try more normal estimation methods.
void CalculateNormals(BuildData* const data, const int first, const int last) {
  // REGISTER_FUNC;
  const int col_count = last - first;
  // build nearest neighbors list
  Matrix d(kDim, col_count);
  Matrix M_wave = Eigen::Matrix<double, kDim, kDim>::Zero();
  for (int i = 0; i < col_count; ++i) {
    d.col(i) = data->points.block(0, data->indices[first + i], kDim, 1);
    M_wave += d.col(i) * d.col(i).transpose();
  }
  const Vector b_wave = d.rowwise().sum();
  const Vector mean = b_wave / col_count;
  const Matrix NN = (d.colwise() - mean);

  // compute covariance
  const Matrix C(NN * NN.transpose());
  if (C.fullPivHouseholderQr().rank() + 1 < kDim) {
    return;
  }

  Vector normal = M_wave.inverse() * Vector(b_wave);

  // sampling in a simple way
  // use just one point in a box
  const int k = data->indices[first];
  // Mark the indices which will be part of the final data
  data->indices_to_keep.push_back(k);
  data->points.col(k) = mean;
  data->normals.col(k) = normal.normalized();
}

void TransformData(BuildData* const data, const Eigen::Matrix4d& transform) {
  const int points_count = data->points.cols();
  CHECK_GT(points_count, 0);
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> homo_data(4,
                                                                  points_count);
  homo_data.setConstant(1.);
  homo_data.block(0, 0, 3, points_count) = data->points;
  data->points = (transform * homo_data).block(0, 0, 3, points_count);

  if (data->normals.cols() > 0) {
    CHECK_EQ(data->normals.cols(), points_count);
    homo_data.block(0, 0, 3, points_count) = data->normals;
    data->normals = (transform * homo_data).block(0, 0, 3, points_count);
  }
}

void BuildNormals(BuildData* const data, const int first, const int last,
                  Eigen::Vector3d&& minValues, Eigen::Vector3d&& maxValues) {
  const int count(last - first);
  if (count <= kNormalEstimationKnn) {
    // compute for this range
    CalculateNormals(data, first, last);
    return;
  }

  // find the largest dimension of the box
  const int cutDim = ArgMax(maxValues - minValues);

  // compute number of elements
  const int rightCount(count / 2);
  const int leftCount(count - rightCount);
  CHECK_EQ(last - rightCount, first + leftCount);

  // sort, hack std::nth_element
  std::nth_element(data->indices.begin() + first,
                   data->indices.begin() + first + leftCount,
                   data->indices.begin() + last, CompareDim(cutDim, *data));

  // get value
  const int cut_index(data->indices[first + leftCount]);
  const double cutVal(data->points(cutDim, cut_index));

  // update bounds for left
  Eigen::Vector3d leftMaxValues(maxValues);
  leftMaxValues[cutDim] = cutVal;
  // update bounds for right
  Eigen::Vector3d rightMinValues(minValues);
  rightMinValues[cutDim] = cutVal;

  // recurse
  BuildNormals(data, first, first + leftCount,
               std::forward<Eigen::Vector3d>(minValues),
               std::move(leftMaxValues));
  BuildNormals(data, first + leftCount, last, std::move(rightMinValues),
               std::forward<Eigen::Vector3d>(maxValues));
}

Matches FindClosests(const std::shared_ptr<NNS>& nns_kdtree,
                     const BuildData& source_cloud) {
  const int points_count(source_cloud.points.cols());
  const int search_count = 1;
  const double epsilon = 3.16;
  // Matches matches(typename Matches::Dists(knn, points_count),
  //                 typename Matches::Ids(knn, points_count));

  // static_assert(NNS::InvalidIndex == Matches::InvalidId, "");
  // static_assert(NNS::InvalidValue == Matches::InvalidDist, "");
  // this->visitCounter +=
  //! A dense integer matrix
  Matches matches(search_count, points_count);
  nns_kdtree->knn(source_cloud.points, matches.ids, matches.dists, search_count,
                  epsilon, NNS::ALLOW_SELF_MATCH);
  return matches;
}

Eigen::MatrixXd CrossProduct(const Eigen::MatrixXd& A,
                             const Eigen::MatrixXd& B) {
  // Expecting matched points
  CHECK_EQ(A.cols(), B.cols());
  // Expecting homogenous coord X eucl. coord
  CHECK_EQ(A.rows(), B.rows());

  constexpr unsigned int x = 0;
  constexpr unsigned int y = 1;
  constexpr unsigned int z = 2;

  Eigen::MatrixXd cross(B.rows(), B.cols());
  cross.row(x) =
      A.row(y).array() * B.row(z).array() - A.row(z).array() * B.row(y).array();
  cross.row(y) =
      A.row(z).array() * B.row(x).array() - A.row(x).array() * B.row(z).array();
  cross.row(z) =
      A.row(x).array() * B.row(y).array() - A.row(y).array() * B.row(x).array();

  return cross;
}

void SolvePossiblyUnderdeterminedLinearSystem(const Matrix& A, const Vector& b,
                                              Vector& x) {  // NOLINT
  CHECK_EQ(A.cols(), A.rows());
  CHECK_EQ(b.cols(), 1);
  CHECK_EQ(b.rows(), A.rows());
  CHECK_EQ(x.cols(), 1);
  CHECK_EQ(x.rows(), A.cols());

  // using Matrix = typename Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

  BOOST_AUTO(Aqr, A.fullPivHouseholderQr());
  if (!Aqr.isInvertible()) {
    // Solve reduced problem R1 x = Q1^T b instead of QR x = b, where Q = [Q1
    // Q2] and R = [ R1 ; R2 ] such that ||R2|| is small (or zero) and therefore
    // A = QR ~= Q1 * R1
    const int rank = Aqr.rank();
    const int rows = A.rows();
    const Matrix Q1t = Aqr.matrixQ().transpose().block(0, 0, rank, rows);
    const Matrix R1 = (Q1t * A * Aqr.colsPermutation()).block(0, 0, rank, rows);

    // The under-determined system R1 x = Q1^T b is made unique ..
    // by getting the solution of smallest norm (x = R1^T * (R1 * R1^T)^-1
    // Q1^T b.
    x = R1.triangularView<Eigen::Upper>().transpose() *
        (R1 * R1.transpose()).llt().solve(Q1t * b);
    x = Aqr.colsPermutation() * x;

    BOOST_AUTO(ax, (A * x).eval());
    if (!b.isApprox(ax, 1e-5)) {
      // LOG(INFO)
      //     << "PointMatcher::icp - encountered almost singular matrix while "
      //        "minimizing point to plane distance. QR solution was too "
      //        "inaccurate. "
      //        "Trying more accurate approach using double precision SVD.";
      x = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
      ax = A * x;

      if ((b - ax).norm() > 1e-5 * std::max(A.norm() * x.norm(), b.norm())) {
        // clang-format off
        LOG(WARNING)
            << "PointMatcher::icp - encountered numerically singular matrix "
               "while minimizing point to plane distance and the current "
               "workaround remained inaccurate."
            << " b=" << b.transpose()
            << " !~ A * x=" << (ax).transpose().eval()
            << ": ||b- ax||=" << (b - ax).norm()
            << ", ||b||=" << b.norm()
            << ", ||ax||=" << ax.norm();
        // clang-format on
      }
    }
  } else {
    // Cholesky decomposition
    // std::cout << "icp_fast test0.." << std::endl;
    x = A.llt().solve(b);
  }
}

Eigen::Matrix4d ComputePointToPlane(const BuildData& source_cloud,
                                    const BuildData& target_cloud,
                                    const Eigen::MatrixXd& weights,
                                    const Matches& matches,
                                    bool compensation = false) {
  const Eigen::MatrixXd target_normals = target_cloud.normals;
  CHECK_EQ(target_normals.rows(), kDim);
  CHECK_EQ(target_normals.cols(), target_cloud.points.cols());
  CHECK_EQ(source_cloud.points.cols(), target_cloud.points.cols());

  // Compute cross product of cross = cross(reading X target_normals)
  const Matrix cross = CrossProduct(source_cloud.points, target_normals);

  // wF = [weights*cross, weights*normals]
  // F  = [cross, normals]
  Matrix wF(target_normals.rows() + cross.rows(), target_normals.cols());
  Matrix F(target_normals.rows() + cross.rows(), target_normals.cols());

  for (int i = 0; i < cross.rows(); i++) {
    wF.row(i) = weights.array() * cross.row(i).array();
    F.row(i) = cross.row(i);
  }
  for (int i = 0; i < target_normals.rows(); i++) {
    wF.row(i + cross.rows()) = weights.array() * target_normals.row(i).array();
    F.row(i + cross.rows()) = target_normals.row(i);
  }

  if (compensation) {
    for (int i = 0; i < target_normals.cols(); ++i) {
      wF.col(i) *= source_cloud.factors[i];
      F.col(i) *= source_cloud.factors[i];
    }
  }

  // Unadjust covariance A = wF * F'
  const Matrix A = wF * F.transpose();
  const Matrix deltas = source_cloud.points - target_cloud.points;

  // dot product of dot = dot(deltas, normals)
  Matrix dotProd = Matrix::Zero(1, target_normals.cols());
  for (int i = 0; i < target_normals.rows(); i++) {
    dotProd += (deltas.row(i).array() * target_normals.row(i).array()).matrix();
  }

  // b = -(wF' * dot)
  const Vector b = -(wF * dotProd.transpose());
  Vector x(A.rows());
  SolvePossiblyUnderdeterminedLinearSystem(A, b, x);

  // Transform parameters to matrix
  Eigen::Matrix4d mOut;
  Eigen::Transform<double, 3, Eigen::Affine> transform;
  // PLEASE DONT USE EULAR ANGLES!!!!
  // Rotation in Eular angles follow roll-pitch-yaw (1-2-3) rule
  /*transform = Eigen::AngleAxis<T>(x(0), Eigen::Matrix<T,1,3>::UnitX())
   * Eigen::AngleAxis<T>(x(1), Eigen::Matrix<T,1,3>::UnitY())
   * Eigen::AngleAxis<T>(x(2), Eigen::Matrix<T,1,3>::UnitZ());*/
  transform =
      Eigen::AngleAxis<double>(x.head(3).norm(), x.head(3).normalized());

  // Reverse roll-pitch-yaw conversion, very useful piece of knowledge, keep
  // it with you all time!
  /*const T pitch = -asin(transform(2,0));
          const T roll = atan2(transform(2,1), transform(2,2));
          const T yaw = atan2(transform(1,0) / cos(pitch), transform(0,0) /
     cos(pitch)); std::cerr << "d angles" << x(0) - roll << ", " << x(1) -
     pitch << "," << x(2) - yaw << std::endl;*/
  transform.translation() = x.segment(3, 3);
  mOut = transform.matrix();

  if (mOut != mOut) {
    // Degenerate situation. This can happen when the source and reading
    // clouds are identical, and then b and x above are 0, and the rotation
    // matrix cannot be determined, it comes out full of NaNs. The correct
    // rotation is the identity.
    mOut.block(0, 0, kDim, kDim) = Matrix::Identity(kDim, kDim);
  }

  return mOut;
}

Eigen::Matrix4d ComputePointToPoint(BuildData& source_cloud,  // NOLINT
                                    BuildData& target_cloud,  // NOLINT
                                    const Eigen::MatrixXd& weights,
                                    const Matches& matches) {
  CHECK_EQ(source_cloud.points.cols(), target_cloud.points.cols());

  // const int dimCount(mPts.reading.features.rows());
  // const int ptsCount(mPts.reading.features.cols()); //Both point clouds have
  // now the same number of (matched) point

  const Vector w = weights.row(0);
  const double w_sum_inv = 1. / w.sum();
  const Vector meanReading =
      (source_cloud.points.array().rowwise() * w.array().transpose())
          .rowwise()
          .sum() *
      w_sum_inv;
  const Vector meanReference =
      (target_cloud.points.array().rowwise() * w.array().transpose())
          .rowwise()
          .sum() *
      w_sum_inv;

  // Remove the mean from the point clouds
  source_cloud.points.colwise() -= meanReading;
  target_cloud.points.colwise() -= meanReference;

  // Singular Value Decomposition
  const Matrix m(target_cloud.points * w.asDiagonal() *
                 source_cloud.points.transpose());
  const Eigen::JacobiSVD<Matrix> svd(m,
                                     Eigen::ComputeThinU | Eigen::ComputeThinV);
  Matrix rotMatrix(svd.matrixU() * svd.matrixV().transpose());
  // It is possible to get a reflection instead of a rotation. In this case, we
  // take the second best solution, guaranteed to be a rotation. For more
  // details, read the tech report: "Least-Squares Rigid Motion Using SVD", Olga
  // Sorkine http://igl.ethz.ch/projects/ARAP/svd_rot.pdf
  if (rotMatrix.determinant() < 0.) {
    Matrix tmpV = svd.matrixV().transpose();
    tmpV.row(kDim - 1) *= -1.;
    rotMatrix = svd.matrixU() * tmpV;
  }
  const Vector trVector(meanReference - rotMatrix * meanReading);

  Matrix result(Matrix::Identity(4, 4));
  result.topLeftCorner(kDim, kDim) = rotMatrix;
  result.topRightCorner(kDim, 1) = trVector;

  return result;
}

bool CheckConvergence(const std::vector<Eigen::Quaterniond>& rotations,
                      const std::vector<Eigen::Vector3d>& translations) {
  constexpr int kSmoothLength = 4;
  constexpr double kConvergeRotDist = 0.001;
  constexpr double kConvergeTransDist = 0.01;

  double rotation_dist = 0.;
  double translation_dist = 0.;
  bool has_converged = false;
  if (rotations.size() > kSmoothLength) {
    for (size_t i = rotations.size() - 1; i >= rotations.size() - kSmoothLength;
         i--) {
      // Compute the mean derivative
      rotation_dist +=
          std::fabs(rotations[i].angularDistance(rotations[i - 1]));
      translation_dist +=
          std::fabs((translations[i] - translations[i - 1]).norm());
    }

    rotation_dist /= kSmoothLength;
    translation_dist /= kSmoothLength;

    if (rotation_dist < kConvergeRotDist &&
        translation_dist < kConvergeTransDist)
      has_converged = true;
  }

  return has_converged;
}

template <typename PointT>
IcpFast<PointT>::IcpFast() : Interface<PointT>() {
  this->type_ = kFastIcp;
  REG_REGISTRATOR_INNER_OPTION("knn_normal_estimate",
                               OptionItemDataType::kInt32,
                               options_.knn_for_normal_estimate);
  REG_REGISTRATOR_INNER_OPTION("max_iteration", OptionItemDataType::kInt32,
                               options_.max_iteration);
  REG_REGISTRATOR_INNER_OPTION("dist_outlier_ratio",
                               OptionItemDataType::kFloat32,
                               options_.dist_outlier_ratio);
}

template <typename PointT>
void IcpFast<PointT>::SetInputSource(const PointCloudSourcePtr& cloud) {
  source_cloud_.reset(new BuildData);
  source_cloud_->FromPointCloud<PointT>(cloud);
}

template <typename PointT>
void IcpFast<PointT>::SetInputTarget(const PointCloudTargetPtr& cloud) {
  // it is the reference cloud
  target_cloud_.reset(new BuildData);
  target_cloud_->FromPointCloud<PointT>(cloud);

  // step1 normal estimation and filtering
  BuildNormals(target_cloud_.get(), 0, cloud->size(),
               target_cloud_->points.rowwise().minCoeff(),
               target_cloud_->points.rowwise().maxCoeff());

  // step2 remove useless points
  // Bring the data we keep to the front of the arrays then
  // wipe the leftover unused space.
  std::sort(target_cloud_->indices_to_keep.begin(),
            target_cloud_->indices_to_keep.end());
  const int points_count_remained = target_cloud_->indices_to_keep.size();
  for (int i = 0; i < points_count_remained; ++i) {
    const int k = target_cloud_->indices_to_keep[i];
    CHECK(i <= k);
    target_cloud_->points.col(i) = target_cloud_->points.col(k);
    target_cloud_->normals.col(i) = target_cloud_->normals.col(k);
  }
  target_cloud_->points.conservativeResize(Eigen::NoChange,
                                           points_count_remained);
  target_cloud_->normals.conservativeResize(Eigen::NoChange,
                                            points_count_remained);

  // for debug
  //
  // pcl::PointCloud<pcl::PointXYZINormal> normal_cloud;
  // const int cloud_size = target_cloud_->points.cols();
  // normal_cloud.reserve(cloud_size);
  // for (int i = 0; i < cloud_size; ++i) {
  //   pcl::PointXYZINormal point;
  //   point.x = target_cloud_->points(0, i);
  //   point.y = target_cloud_->points(1, i);
  //   point.z = target_cloud_->points(2, i);
  //   point.normal_x = target_cloud_->normals(0, i);
  //   point.normal_y = target_cloud_->normals(1, i);
  //   point.normal_z = target_cloud_->normals(2, i);
  //   normal_cloud.push_back(point);
  // }
  // pcl::io::savePCDFileBinary("/tmp/normal.pcd", normal_cloud);

  // step2 build KDTREE for closeset point searching
  // nns_kdtree_.reset(
  //     NNS::create(target_cloud_->points, kDim, NNS::KDTREE_LINEAR_HEAP));
}

template <typename PointT>
bool IcpFast<PointT>::Align(const Eigen::Matrix4d& guess,
                            Eigen::Matrix4d& result) {  // NOLINT
  const int target_points_count = target_cloud_->points.cols();
  const Eigen::Vector3d target_mean =
      target_cloud_->points.rowwise().sum() / target_points_count;
  Eigen::Matrix4d T_target_mean = Eigen::Matrix4d::Identity();
  T_target_mean.block(0, 3, 3, 1) = target_mean;

  // std::cout << "target mean: \n" << std::flush;
  // common::PrintTransform(T_target_mean);

  target_cloud_->points.colwise() -= target_mean;
  nns_kdtree_.reset(
      NNS::create(target_cloud_->points, kDim, NNS::KDTREE_LINEAR_HEAP));

  BuildData init_source_cloud = *source_cloud_;
  Eigen::Matrix4d T_target_mean_init_guess = T_target_mean.inverse() * guess;
  TransformData(&init_source_cloud, T_target_mean_init_guess);

  // initialise all iter status
  Eigen::Matrix4d T_iter = Eigen::Matrix4d::Identity();
  std::vector<Eigen::Quaterniond> rotations_iter;
  std::vector<Eigen::Vector3d> translations_iter;
  rotations_iter.reserve(10);
  translations_iter.reserve(10);
  rotations_iter.push_back(Eigen::Quaterniond::Identity());
  translations_iter.push_back(Eigen::Vector3d::Zero());

  int iterator = 0;
  while (true) {
    // step1 Find Closest points
    BuildData step_cloud(init_source_cloud);
    if (this->inner_compensation_) {
      MotionCompensation(T_iter, &step_cloud);
    } else {
      TransformData(&step_cloud, T_iter);
    }

    const Matches matches(FindClosests(nns_kdtree_, step_cloud));

    // step2 reject outliers (outliers with weight 0)
    const double limit = matches.GetDistsQuantile(options_.dist_outlier_ratio);
    const Matrix output_weights =
        (matches.dists.array() <= limit).cast<double>();

    // step3 solve point-to-plane and update result
    CHECK_EQ(output_weights.cols(), step_cloud.points.cols());

    ErrorElements error_elements(step_cloud, *target_cloud_, output_weights,
                                 matches);

    T_iter =
        ComputePointToPlane(error_elements.reading, error_elements.reference,
                            error_elements.weights, error_elements.matches,
                            this->inner_compensation_) *
        T_iter;

    // step4 check convergence and jump out
    iterator++;
    rotations_iter.emplace_back(Eigen::Matrix3d(T_iter.block(0, 0, 3, 3)));
    translations_iter.push_back(T_iter.block(0, 3, 3, 1));
    if (CheckConvergence(rotations_iter, translations_iter)) {
      break;
    }
    if (iterator >= options_.max_iteration) {
      break;
    }
  }

  // calculate the score
  {
    TransformData(&init_source_cloud, T_iter);
    const Matches matches(FindClosests(nns_kdtree_, init_source_cloud));

    const double limit = matches.GetDistsQuantile(options_.dist_outlier_ratio);
    const Matrix output_weights =
        (matches.dists.array() <= limit).cast<double>();
    ErrorElements error_elements(init_source_cloud, *target_cloud_,
                                 output_weights, matches);

    CHECK_GT(error_elements.matches.dists.cols(), 0);
    // @todo(edward) use dist instead of squared dists
    const double average_dist = error_elements.matches.dists.sum() /
                                error_elements.matches.dists.cols();
    this->final_score_ = std::exp(-average_dist);
  }

  result = T_target_mean * T_iter * T_target_mean_init_guess;
  return true;
}

template class IcpFast<pcl::PointXYZI>;
}  // namespace registrator
}  // namespace static_map
