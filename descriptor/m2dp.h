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

#include "builder/data/cloud_types.h"

namespace static_map {
namespace descriptor {

// refer to the paper
// "M2DP: A Novel 3D Point Cloud Descriptor and Its Application in Loop
// Closure Detection"
class M2dp {
 public:
  using Descriptor = Eigen::VectorXf;

  M2dp(double r = 0.1, double max_distance = 100., int32_t t = 16,
       int32_t p = 4, int32_t q = 16);
  ~M2dp() = default;

  bool setInputCloud(const data::InnerCloudType::Ptr& source);

  Descriptor getFinalDescriptor();

 private:
  // part III.B in paper
  void preProcess(const data::InnerCloudType::Ptr& source);
  // part III.C in paper
  Eigen::VectorXi singleViewProcess(double theta, double phi);

  data::InnerCloudType::Ptr inner_cloud_;
  Eigen::Vector3f mean_;

  // parameters in paper
  int32_t l_, t_;
  int32_t p_, q_;
  double r_;
  double max_distance_;

  Eigen::MatrixXf A_;
  Descriptor descriptor_;
};

inline M2dp::Descriptor M2dp::getFinalDescriptor() { return descriptor_; }

// return the match score ( 0, 1 )
double matchTwoM2dpDescriptors(const M2dp::Descriptor& P,
                               const M2dp::Descriptor& Q);

}  // namespace descriptor
}  // namespace static_map

#endif  // DESCRIPTOR_M2DP_H_
