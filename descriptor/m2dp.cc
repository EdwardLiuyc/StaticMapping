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

#include "descriptor/m2dp.h"

namespace static_map {
namespace descriptor {

struct PolarCoordinate {
  double angle, length;
};

template <typename PointType>
double getLength(const PointType& a) {
  return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}

template <typename PointType>
M2dp<PointType>::M2dp(double r, double max_distance, int32_t t, int32_t p,
                      int32_t q)
    : inner_cloud_(new M2dp<PointType>::PointCloudSource),
      t_(t),
      p_(p),
      q_(q),
      r_(r),
      max_distance_(max_distance) {}

template <typename PointType>
void M2dp<PointType>::preProcess(
    const M2dp<PointType>::PointCloudSourcePtr& source) {
  // remove the shift and rotation
  pcl::PCA<PointType> pca;
  pca.setInputCloud(source);
  PointCloudSourcePtr tmp_inner_cloud(new PointCloudSource);
  pca.project(*source, *tmp_inner_cloud);

  for (auto& point : tmp_inner_cloud->points) {
    double d = getLength<PointType>(point);
    if (d <= max_distance_) inner_cloud_->push_back(point);
  }

  if (r_ < 1.e-6) {
    PCL_ERROR("r is too small.\n");
    return;
  }
  l_ = std::ceil(std::sqrt(max_distance_ / r_));
  A_.resize(p_ * q_, l_ * t_);
}

template <typename PointType>
Eigen::VectorXi M2dp<PointType>::singleViewProcess(double theta, double phi) {
  // normal
  Eigen::Vector3f m;
  m << std::cos(theta) * std::cos(phi), std::cos(theta) * std::sin(phi),
      std::sin(theta);

  // refer to the matlab code in git (according to the paper)
  Eigen::Vector3f projected_x_axis =
      Eigen::Vector3f(1., 0., 0.) -
      (Eigen::Vector3f(1., 0., 0.).transpose() * m).norm() * m;
  Eigen::Vector3f projected_y_axis = m.cross(projected_x_axis);

  const double m_2_pi = M_PI * 2.;
  auto calulate_polar = [&m_2_pi](const Eigen::Vector2f& p) {
    PolarCoordinate coord;
    coord.length = p.norm();
    coord.angle = std::atan2(p[1], p[0]);
    if (coord.angle < 0.) coord.angle += m_2_pi;

    return coord;
  };

  Eigen::VectorXi row;
  row.resize(l_ * t_, 1);
  for (int i = 0; i < l_ * t_; ++i) row[i] = 0;
  const double angle_step = m_2_pi / t_;
  for (auto& point : inner_cloud_->points) {
    Eigen::Vector3f p(point.x, point.y, point.z);
    Eigen::Vector2f projected_point((p.transpose() * projected_x_axis).norm(),
                                    (p.transpose() * projected_y_axis).norm());

    PolarCoordinate coord = calulate_polar(projected_point);
    int32_t l_index = std::floor(std::sqrt(coord.length / r_));
    // avoid over border
    if (l_index > l_ - 1) {
      l_index = l_ - 1;
    }
    int32_t t_index = std::floor(coord.angle / angle_step);
    if (t_index > t_ - 1) {
      t_index = t_ - 1;
    }
    int32_t index = l_index * t_ + t_index;
    //         std::cout << index << "  " << l_index << "  " << t_index << "  "
    //         << coord.angle << std::endl;
    row[index]++;
  }

  return row;
}

template <typename PointType>
bool M2dp<PointType>::setInputCloud(const PointCloudSourcePtr& source) {
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

template class M2dp<pcl::PointXYZI>;

template double matchTwoM2dpDescriptors<pcl::PointXYZI>(
    const typename M2dp<pcl::PointXYZI>::Descriptor& P,
    const typename M2dp<pcl::PointXYZI>::Descriptor& Q);

}  // namespace descriptor
}  // namespace static_map
