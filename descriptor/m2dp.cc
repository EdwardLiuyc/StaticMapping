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

template <typename PointType>
void M2dp<PointType>::preProcess(
    const M2dp<PointType>::PointCloudSourcePtr& source) {
  // remove the shift and rotation
  pcl::PCA<PointType> pca;
  pca.setInputCloud(source);
  PointCloudSourcePtr tmp_inner_cloud(new PointCloudSource);
  pca.project(*source, *tmp_inner_cloud);

  for (auto& point : tmp_inner_cloud->points) {
    double d = getLength(point);
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

template class M2dp<pcl::PointXYZI>;

}  // namespace descriptor
}  // namespace static_map
