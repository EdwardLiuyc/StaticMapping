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

#ifndef REGISTRATORS_MULTIVIEW_REGISTRATOR_LUM_PCL_H_
#define REGISTRATORS_MULTIVIEW_REGISTRATOR_LUM_PCL_H_

#include <vector>

#include "pcl/registration/correspondence_estimation.h"
#include "pcl/registration/lum.h"
#include "registrators/multiview_registrator_interface.h"

namespace static_map {
namespace registrator {

template <typename PointT>
class MultiviewRegistratorLumPcl : public MultiviewInterface<PointT> {
 public:
  using PointCloudPtr = typename MultiviewInterface<PointT>::PointCloudPtr;

  MultiviewRegistratorLumPcl() : MultiviewInterface<PointT>() {
    lum_.setMaxIterations(5);
    lum_.setConvergenceThreshold(0.001);
  }
  ~MultiviewRegistratorLumPcl() = default;

  void AlignAll(const PointCloudPtr& output) override {
    auto& frames = MultiviewInterface<PointT>::inner_frames_;
    if (frames.empty()) {
      if (output) {
        output->clear();
      }
      return;
    }

    std::vector<PointCloudPtr> transformed_clouds;
    for (int iter_index = 0; iter_index < max_iteration_num_; ++iter_index) {
      for (size_t i = 0; i < frames.size(); ++i) {
        PointCloudPtr transformed_cloud(new pcl::PointCloud<PointT>);
        pcl::transformPointCloud(*frames[i].cloud, *transformed_cloud,
                                 frames[i].pose);
        lum_.addPointCloud(transformed_cloud);
        transformed_clouds.push_back(transformed_cloud);

        for (size_t j = 0; j < i; ++j) {
          pcl::registration::CorrespondenceEstimation<PointT, PointT> ce;
          ce.setInputTarget(transformed_clouds[i]);
          ce.setInputSource(transformed_clouds[j]);
          pcl::CorrespondencesPtr corr(new pcl::Correspondences);
          ce.determineCorrespondences(*corr, 0.25);

          if (corr->size() > 2) lum_.setCorrespondences(j, i, corr);
        }
      }
      lum_.compute();
    }

    if (output) {
      output->clear();
      for (int i = 0; i < frames.size(); ++i) {
        *output += *lum_.getTransformedCloud(i);
      }
    }
  }

 private:
  pcl::registration::LUM<PointT> lum_;
  int32_t max_iteration_num_ = 5;
};

}  // namespace registrator
}  // namespace static_map

#endif  // REGISTRATORS_MULTIVIEW_REGISTRATOR_LUM_PCL_H_
