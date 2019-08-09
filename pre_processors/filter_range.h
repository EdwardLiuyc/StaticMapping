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

#pragma once

#include <memory>

#include "pre_processors/filter_interface.h"

namespace static_map {
namespace pre_processers {
namespace filter {

template <typename PointT>
class Range : public Interface<PointT> {
 public:
  USE_POINTCLOUD;

  Range() : Interface<PointT>(), min_range_(0.), max_range_(100.) {
    // float params
    INIT_INNER_PARAM(Interface<PointT>::kFloatParam, 0, "min_range",
                     min_range_);
    INIT_INNER_PARAM(Interface<PointT>::kFloatParam, 1, "max_range",
                     max_range_);
  }
  ~Range() {}
  Range(const Range &) = delete;
  Range &operator=(const Range &) = delete;

  std::shared_ptr<Interface<PointT>> CreateNewInstance() override {
    return std::make_shared<Range<PointT>>();
  }

  void Filter(const PointCloudPtr &cloud) override {
    if (!cloud || !Interface<PointT>::inner_cloud_) {
      LOG(WARNING) << "nullptr cloud, do nothing!" << std::endl;
      return;
    }

    this->FilterPrepare(cloud);
    const int size = this->inner_cloud_->size();
    bool is_inlier[size];
#ifdef _OPENMP
#pragma omp parallel for num_threads(LOCAL_OMP_THREADS_NUM)
#endif
    for (int i = 0; i < size; ++i) {
      auto &point = this->inner_cloud_->points[i];
      float range =
          std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
      if (range >= min_range_ && range <= max_range_) {
        is_inlier[i] = true;
      } else {
        is_inlier[i] = false;
      }
    }

    // first, reserve
    // then, push_back
    // finally, shrink to fit
    // to get best efficiency and space usage
    this->inliers_.reserve(size);
    this->outliers_.reserve(size);
    cloud->points.reserve(size);
    for (int i = 0; i < size; ++i) {
      if (is_inlier[i]) {
        this->inliers_.push_back(i);
        cloud->push_back(this->inner_cloud_->points[i]);
      } else {
        this->outliers_.push_back(i);
      }
    }
    this->inliers_.shrink_to_fit();
    this->outliers_.shrink_to_fit();
    cloud->points.shrink_to_fit();
  }

  void DisplayAllParams() override {
    PARAM_INFO(min_range_);
    PARAM_INFO(max_range_);
  }

 private:
  float min_range_;
  float max_range_;
};

}  // namespace filter
}  // namespace pre_processers
}  // namespace static_map
