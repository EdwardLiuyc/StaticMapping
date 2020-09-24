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

#include "registrators/ndt_gicp.h"

namespace static_map {
namespace registrator {

template <typename PointType>
NdtWithGicp<PointType>::NdtWithGicp() : Interface<PointType>() {
  this->type_ = kNdtWithGicp;

  REG_REGISTRATOR_INNER_OPTION("use_ndt", OptionItemDataType::kBool,
                               options_.use_ndt);
  REG_REGISTRATOR_INNER_OPTION("using_voxel_filter", OptionItemDataType::kBool,
                               options_.using_voxel_filter);
  REG_REGISTRATOR_INNER_OPTION("voxel_resolution", OptionItemDataType::kFloat32,
                               options_.voxel_resolution);
}

template <typename PointType>
void NdtWithGicp<PointType>::InitWithOptions() {
  LOG(INFO) << "Init ";
  approximate_voxel_filter_.setLeafSize(options_.voxel_resolution,
                                        options_.voxel_resolution,
                                        options_.voxel_resolution);

  // TODO(edward) add more configs
  ndt_.setTransformationEpsilon(0.01);
  ndt_.setStepSize(0.1);
  ndt_.setResolution(1.);
  ndt_.setMaximumIterations(35);

  gicp_.setRotationEpsilon(1e-3);
  gicp_.setMaximumIterations(35);
}

template <typename PointType>
bool NdtWithGicp<PointType>::Align(const Eigen::Matrix4d& guess,
                                   Eigen::Matrix4d& result) {  // NOLINT
  if (options_.using_voxel_filter) {
    if (!down_sampled_source_cloud_) {
      down_sampled_source_cloud_ = boost::make_shared<PointCloudSource>();
    }
    if (!down_sampled_target_cloud_) {
      down_sampled_target_cloud_ = boost::make_shared<PointCloudTarget>();
    }

    approximate_voxel_filter_.setInputCloud(this->source_cloud_->GetPclCloud());
    approximate_voxel_filter_.filter(*down_sampled_source_cloud_);

    approximate_voxel_filter_.setInputCloud(this->target_cloud_->GetPclCloud());
    approximate_voxel_filter_.filter(*down_sampled_target_cloud_);
  } else {
    down_sampled_source_cloud_ = this->source_cloud_->GetPclCloud();
    down_sampled_target_cloud_ = this->target_cloud_->GetPclCloud();
  }

  PointCloudSourcePtr output_cloud(new PointCloudSource);
  Eigen::Matrix4f ndt_guess = guess.cast<float>();
  double ndt_score = 0.9;
  if (options_.use_ndt) {
    ndt_.setInputSource(down_sampled_source_cloud_);
    ndt_.setInputTarget(down_sampled_target_cloud_);
    ndt_.align(*output_cloud, guess.cast<float>());
    ndt_score = ndt_.getFitnessScore();

    ndt_guess = ndt_.getFinalTransformation();
  }

  double icp_score = 10.;
  Eigen::Matrix4f final_guess;
  if (ndt_score <= 1.) {
    icp_score = ndt_score;

    gicp_.setInputSource(down_sampled_source_cloud_);
    gicp_.setInputTarget(down_sampled_target_cloud_);
    gicp_.align(*output_cloud, ndt_guess);

    icp_score = gicp_.getFitnessScore();
    final_guess = gicp_.getFinalTransformation();

    this->final_score_ = std::exp(-icp_score);
    result = final_guess.cast<double>();
    return true;
  } else {
    result = guess;
    this->final_score_ = std::exp(-icp_score);
    return false;
  }

  return true;
}

template class NdtWithGicp<pcl::PointXYZI>;
template class NdtWithGicp<pcl::PointXYZ>;

}  // namespace registrator
}  // namespace static_map
