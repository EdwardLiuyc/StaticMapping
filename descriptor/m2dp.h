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

#ifndef DESCRIPTOR_M2DP_H_
#define DESCRIPTOR_M2DP_H_

// third party
#include <pcl/ModelCoefficients.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <Eigen/SVD>

namespace static_map {
namespace descriptor {

// refer to the paper
// "M2DP: A Novel 3D Point Cloud Descriptor and Its Application in Loop
// Closure Detection"
template <typename PointType>
class M2dp {
 public:
  typedef pcl::PointCloud<PointType> PointCloudSource;
  typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
  typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

  typedef Eigen::VectorXf Descriptor;

  M2dp(double r = 0.1, double max_distance = 100., int32_t t = 16,
       int32_t p = 4, int32_t q = 16)
      : inner_cloud_(new PointCloudSource),
        t_(t),
        p_(p),
        q_(q),
        r_(r),
        max_distance_(max_distance) {}
  ~M2dp() = default;

  struct PolarCoordinate {
    double angle, length;
  };

  bool setInputCloud(const PointCloudSourcePtr& source) {
    if (source->empty()) {
      PCL_ERROR("source is empty.\n");
      return false;
    }

    // step1 pre processing
    preProcess(source);

    // step2 calculate A
    const double theta_step = M_PI / p_;
    const double phi_step = M_PI_2 / q_;
    for (int p = 0; p < p_; ++p)
      for (int q = 0; q < q_; ++q) {
        Eigen::VectorXi row = singleViewProcess(p * theta_step, q * phi_step);
        A_.block(p * q_ + q, 0, 1, l_ * t_) = row.transpose().cast<float>();
      }

    // SVD and get the final Descriptor
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(
        A_, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXf V = svd.matrixV(), U = svd.matrixU();
    Eigen::VectorXf v1 = V.col(0), u1 = U.col(0);  // paper algorithm1.15

    descriptor_.resize(u1.rows() + v1.rows(), 1);
    descriptor_ << u1, v1;
    return true;
  }

  inline Descriptor getFinalDescriptor() { return descriptor_; }

 private:
  // part III.B in paper
  void preProcess(const PointCloudSourcePtr& source);
  // part III.C in paper
  Eigen::VectorXi singleViewProcess(double theta, double phi);

  inline double getLength(const PointType& a) {
    return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
  }

  PointCloudSourcePtr inner_cloud_;

  Eigen::Vector3f mean_;

  // parameters in paper
  int32_t l_, t_;
  int32_t p_, q_;
  double r_;
  double max_distance_;

  Eigen::MatrixXf A_;
  Descriptor descriptor_;
};

// return the match score ( 0, 1 )
template <typename PointType>
double matchTwoM2dpDescriptors(const typename M2dp<PointType>::Descriptor& P,
                               const typename M2dp<PointType>::Descriptor& Q) {
  if (P.rows() != Q.rows() || P.rows() < 10) {
    PCL_ERROR("The Descriptors do not match.\n");
    return -1.;
  }

  // refer to paper
  // "Spin-Images: A Representation for 3-D Surface Matching"
  // section 2.4 page.28
  auto N = P.rows();
  double score = (N * P.dot(Q) - P.sum() * Q.sum()) /
                 std::sqrt((N * P.dot(P) - std::pow(P.sum(), 2)) *
                           (N * Q.dot(Q) - std::pow(Q.sum(), 2)));

  //   double lamda = 1.;
  //   score = std::pow( std::atanh(score), 2 ) - lamda / ( N - 3 );
  // 这里做了一个简单的取绝对值处理，因为相关性有时候会出现接近 -1 的值
  // 目前考虑点云不会有负相关的情况，绝对值够大就表示点云相似性够好
  return std::fabs(score);
}

}  // namespace descriptor
}  // namespace static_map

#endif  // DESCRIPTOR_M2DP_H_
