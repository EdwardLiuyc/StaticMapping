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

#include "registrators/ndt.h"
#include "pclomp/voxel_grid_covariance_omp_impl.hpp"

namespace static_map {
namespace registrator {

Ndt::Ndt() : Interface() {
  this->type_ = kNdt;
  inner_matcher_.setResolution(1.);
  inner_matcher_.setNumThreads(6);
  inner_matcher_.setNeighborhoodSearchMethod(pclomp::KDTREE);
}

Ndt::~Ndt() {}

bool Ndt::Align(const Eigen::Matrix4d& guess,
                Eigen::Matrix4d& result) {  // NOLINT
  if (!this->source_cloud_ || !this->target_cloud_) {
    return false;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_pcl(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_pcl(
      new pcl::PointCloud<pcl::PointXYZ>);
  data::ToPclPointCloud(*this->source_cloud_->GetInnerCloud(),
                        source_cloud_pcl.get());
  data::ToPclPointCloud(*this->target_cloud_->GetInnerCloud(),
                        target_cloud_pcl.get());

  inner_matcher_.setInputSource(source_cloud_pcl);
  inner_matcher_.setInputTarget(target_cloud_pcl);

  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  inner_matcher_.align(*aligned_cloud, guess.cast<float>());

  this->final_score_ = inner_matcher_.getFitnessScore();
  result = inner_matcher_.getFinalTransformation().template cast<double>();

  return true;
}

}  // namespace registrator
}  // namespace static_map
