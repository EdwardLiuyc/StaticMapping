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

#ifndef PRE_PROCESSORS_RANDOM_SAMPLE_WITH_PLANE_DETECT_H_
#define PRE_PROCESSORS_RANDOM_SAMPLE_WITH_PLANE_DETECT_H_

// @todo(edward) need to be refactored later!!!

// stl
#include <algorithm>
#include <vector>
// local
#include "pre_processors/plane_detector.h"

namespace static_map {

template <typename PointT>
class RandomSamplerWithPlaneDetect {
 public:
  using PointCloudType = typename pcl::PointCloud<PointT>;
  using PointCloudPtr = typename PointCloudType::Ptr;
  using PointCloudConstPtr = typename PointCloudType::ConstPtr;

  RandomSamplerWithPlaneDetect() : rate_(1.), ground_height_threshold_(0.2) {
    plane_detector_.SetMinPointNumInVoxel(10);
    plane_detector_.SetLeafSize(0.8);
  }

  ~RandomSamplerWithPlaneDetect() = default;

  inline void SetSamplingRate(float rate) {
    if (rate > 1.) {
      LOG(WARNING) << "rate should not be large than 1." << std::endl;
      rate = 1.;
    } else if (rate < 1.e-6) {
      LOG(WARNING) << "rate should not be less than 1.e-6" << std::endl;
      rate = 1.e-6;
    }
    rate_ = rate;
  }

  inline void SetLeafSize(float size) { plane_detector_.SetLeafSize(size); }

  inline void SetMinPointNumInVoxel(size_t num) {
    plane_detector_.SetMinPointNumInVoxel(num);
  }

  inline void SetInputCloud(const PointCloudPtr& cloud) {
    if (!cloud || cloud->empty()) {
      PRINT_ERROR("cloud is empty.");
      input_cloud_ = nullptr;
      return;
    }
    input_cloud_ = cloud;
  }

  inline void Filter(const PointCloudPtr& cloud) {
    if (!input_cloud_ || input_cloud_->empty()) {
      return;
    }
    if (rate_ > 0.999999) {
      *cloud = *input_cloud_;
      return;
    }

    cloud->clear();
    std::vector<int> plane_indices;
    plane_detector_.SetInputCloud(input_cloud_);
    plane_detector_.Detect(plane_indices, ground_height_threshold_);
    std::sort(plane_indices.begin(), plane_indices.end());

    int other_sum = 0;
    int other_sapmled_sum = 0;
    for (int i = 0; i < input_cloud_->size(); ++i) {
      if (i == plane_indices.front()) {
        cloud->push_back(input_cloud_->points[i]);
        plane_indices.erase(plane_indices.begin());
      } else {
        other_sum++;
        if (static_cast<float>(other_sapmled_sum) /
                static_cast<float>(other_sum) <
            rate_) {
          cloud->push_back(input_cloud_->points[i]);
          other_sapmled_sum++;
        }
      }
    }
  }

 private:
  float rate_;
  float ground_height_threshold_;
  PlaneDetector<PointT> plane_detector_;
  PointCloudPtr input_cloud_;
};

}  // namespace static_map

#endif  // PRE_PROCESSORS_RANDOM_SAMPLE_WITH_PLANE_DETECT_H_
