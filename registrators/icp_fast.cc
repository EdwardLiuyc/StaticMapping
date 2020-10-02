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
using data::EigenPointCloud;

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
    // REGISTER_FUNC;
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
  EigenPointCloud reading;    //!< reading point cloud
  EigenPointCloud reference;  //!< reference point cloud
  OutlierWeights weights;     //!< weights for every association
  Matches matches;            //!< associations

  ErrorElements();
  ErrorElements(const EigenPointCloud& source_points,
                const EigenPointCloud& target_points,
                const OutlierWeights& weights, const Matches& matches) {
    REGISTER_FUNC;
    CHECK_GT(matches.ids.rows(), 0);
    CHECK_GT(matches.ids.cols(), 0);
    CHECK(matches.ids.cols() == source_points.points.cols());  // nbpts
    CHECK(weights.rows() == matches.ids.rows());               // knn
    CHECK_EQ(weights.rows(), 1);

    const int dim_points = source_points.points.rows();

    // Count points with no weights
    const int points_count = (weights.array() != 0.0).count();
    CHECK_GT(points_count, 0) << "no point to minimize";

    Matrix kept_points(dim_points, points_count);
    std::vector<double> kept_points_factor(points_count);
    Matches kept_matches(1, points_count);
    OutlierWeights kept_weights(1, points_count);

    int j = 0;
    for (int i = 0; i < source_points.points.cols(); ++i) {
      const auto match_dist = matches.dists(0, i);
      if (match_dist == NNS::InvalidValue) {
        continue;
      }

      if (weights(0, i) != 0.0) {
        kept_points.col(j) = source_points.points.col(i);
        kept_points_factor[j] = source_points.factors[i];
        kept_matches.ids(0, j) = matches.ids(0, i);
        kept_matches.dists(0, j) = match_dist;
        kept_weights(0, j) = weights(0, i);
        ++j;
      }
    }
    CHECK_EQ(j, points_count);
    CHECK_EQ(dim_points, target_points.points.rows());

    const int dim_target_normals = target_points.normals.rows();
    Matrix associated_target_normals;
    if (dim_target_normals > 0) {
      associated_target_normals = Matrix(dim_target_normals, points_count);
    }

    Matrix associated_points(dim_points, points_count);
    // Fetch matched points
    for (int i = 0; i < points_count; ++i) {
      const int ref_index(kept_matches.ids(i));
      associated_points.col(i) =
          target_points.points.block(0, ref_index, dim_points, 1);

      if (dim_target_normals > 0)
        associated_target_normals.col(i) =
            target_points.normals.block(0, ref_index, dim_target_normals, 1);
    }

    // Copy final data to structure
    reading.points = kept_points;
    reading.factors = std::move(kept_points_factor);
    reference.points = associated_points;
    reference.normals = associated_target_normals;

    this->weights = kept_weights;
    this->matches = kept_matches;
  }
};

Matches FindClosests(const std::shared_ptr<NNS>& nns_kdtree,
                     const EigenPointCloud& source_cloud) {
  REGISTER_FUNC;
  const int points_count(source_cloud.points.cols());
  const int search_count = 1;
  const double epsilon = 3.16;
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
    // Q2] and R = [ R1 ; R2 ] such that ||R2|| is small (or zero) and
    // therefore A = QR ~= Q1 * R1
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
    x = A.llt().solve(b);
  }
}

Eigen::Matrix4d ComputePointToPlane(const EigenPointCloud& source_cloud,
                                    const EigenPointCloud& target_cloud,
                                    const Eigen::MatrixXd& weights,
                                    const Matches& matches,
                                    bool compensation = false) {
  REGISTER_FUNC;
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
  Eigen::Matrix4d result;
  Eigen::Transform<double, 3, Eigen::Affine> transform;
  transform =
      Eigen::AngleAxis<double>(x.head(3).norm(), x.head(3).normalized());

  transform.translation() = x.segment(3, 3);
  result = transform.matrix();

  if (result.hasNaN()) {
    // Degenerate situation. This can happen when the source and reading
    // clouds are identical, and then b and x above are 0, and the rotation
    // matrix cannot be determined, it comes out full of NaNs. The correct
    // rotation is the identity.
    result.block(0, 0, kDim, kDim) = Matrix::Identity(kDim, kDim);
  }

  return result;
}

Eigen::Matrix4d ComputePointToPoint(EigenPointCloud& source_cloud,  // NOLINT
                                    EigenPointCloud& target_cloud,  // NOLINT
                                    const Eigen::MatrixXd& weights,
                                    const Matches& matches) {
  CHECK_EQ(source_cloud.points.cols(), target_cloud.points.cols());

  // const int dimCount(mPts.reading.features.rows());
  // const int ptsCount(mPts.reading.features.cols()); //Both point clouds
  // have now the same number of (matched) point

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
  // It is possible to get a reflection instead of a rotation. In this case,
  // we take the second best solution, guaranteed to be a rotation. For more
  // details, read the tech report: "Least-Squares Rigid Motion Using SVD",
  // Olga Sorkine http://igl.ethz.ch/projects/ARAP/svd_rot.pdf
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
void IcpFast<PointT>::SetInputSource(InnerCloudPtr cloud) {
  CHECK(cloud);
  CHECK(cloud->GetEigenCloud());
  source_cloud_.reset(new EigenPointCloud(*cloud->GetEigenCloud()));
}

template <typename PointT>
void IcpFast<PointT>::SetInputTarget(InnerCloudPtr cloud) {
  CHECK(cloud);
  CHECK(cloud->GetEigenCloud());
  CHECK(cloud->GetEigenCloud()->HasNormals());
  target_cloud_.reset(new EigenPointCloud(*cloud->GetEigenCloud()));

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

  target_cloud_->points.colwise() -= target_mean;
  {
    // REGISTER_BLOCK("BuildKdTree");
    nns_kdtree_.reset(
        NNS::create(target_cloud_->points, kDim, NNS::KDTREE_LINEAR_HEAP));
  }
  EigenPointCloud init_source_cloud = *source_cloud_;
  Eigen::Matrix4d T_target_mean_init_guess = T_target_mean.inverse() * guess;
  init_source_cloud.ApplyTransform(T_target_mean_init_guess);

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
    REGISTER_BLOCK("Iteration");
    // step1 Find Closest points
    EigenPointCloud step_cloud(init_source_cloud);
    if (this->inner_compensation_) {
      step_cloud.ApplyMotionCompensation(T_iter);
    } else {
      step_cloud.ApplyTransform(T_iter);
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
    if (CheckConvergence(rotations_iter, translations_iter) ||
        iterator >= options_.max_iteration) {
      const double average_dist = error_elements.matches.dists.sum() /
                                  error_elements.matches.dists.cols();
      this->final_score_ = std::exp(-average_dist);
      break;
    }
  }

  target_cloud_->points.colwise() += target_mean;
  result = T_target_mean * T_iter * T_target_mean_init_guess;
  return true;
}

template class IcpFast<pcl::PointXYZI>;
}  // namespace registrator
}  // namespace static_map
