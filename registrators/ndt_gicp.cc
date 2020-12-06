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

NdtWithGicp::NdtWithGicp() : Interface() {
  this->type_ = kNdtWithGicp;

  REG_REGISTRATOR_INNER_OPTION("use_ndt", OptionItemDataType::kBool,
                               options_.use_ndt);
  REG_REGISTRATOR_INNER_OPTION("using_voxel_filter", OptionItemDataType::kBool,
                               options_.using_voxel_filter);
  REG_REGISTRATOR_INNER_OPTION("voxel_resolution", OptionItemDataType::kFloat32,
                               options_.voxel_resolution);
}

void NdtWithGicp::InitWithOptions() {
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

bool NdtWithGicp::Align(const Eigen::Matrix4d& guess,
                        Eigen::Matrix4d& result) {  // NOLINT
  down_sampled_source_cloud_.reset(new PointCloudSource);
  down_sampled_target_cloud_.reset(new PointCloudTarget);
  if (options_.using_voxel_filter) {
    PointCloudSourcePtr temp_source_pcl(new PointCloudSource);
    data::ToPclPointCloud(*this->source_cloud_->GetInnerCloud(),
                          temp_source_pcl.get());
    approximate_voxel_filter_.setInputCloud(temp_source_pcl);
    approximate_voxel_filter_.filter(*down_sampled_source_cloud_);

    PointCloudSourcePtr temp_target_pcl(new PointCloudSource);
    data::ToPclPointCloud(*this->target_cloud_->GetInnerCloud(),
                          temp_target_pcl.get());
    approximate_voxel_filter_.setInputCloud(temp_target_pcl);
    approximate_voxel_filter_.filter(*down_sampled_target_cloud_);
  } else {
    data::ToPclPointCloud(*this->source_cloud_->GetInnerCloud(),
                          down_sampled_source_cloud_.get());
    data::ToPclPointCloud(*this->target_cloud_->GetInnerCloud(),
                          down_sampled_target_cloud_.get());
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

}  // namespace registrator
}  // namespace static_map
