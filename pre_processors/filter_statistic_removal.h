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

#include "pcl/filters/statistical_outlier_removal.h"
#include "pre_processors/filter_interface.h"

namespace static_map {
namespace pre_processers {
namespace filter {

template <typename PointT>
class StatisticRemoval : public Interface<PointT> {
 public:
  USE_POINTCLOUD;

  StatisticRemoval() : Interface<PointT>(), point_num_meank_(30), std_mul_(1.) {
    // float params
    INIT_FLOAT_PARAM("std_mul", std_mul_);
    // int32_t params
    INIT_INT32_PARAM("point_num_meank", point_num_meank_);
  }
  ~StatisticRemoval() {}
  StatisticRemoval(const StatisticRemoval &) = delete;
  StatisticRemoval &operator=(const StatisticRemoval &) = delete;

  std::shared_ptr<Interface<PointT>> CreateNewInstance() override {
    return std::make_shared<StatisticRemoval<PointT>>();
  }

  void Filter(const PointCloudPtr &cloud) override {
    if (!cloud || !Interface<PointT>::inner_cloud_) {
      LOG(WARNING) << "nullptr cloud, do nothing!" << std::endl;
      return;
    }

    this->FilterPrepare(cloud);
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(this->inner_cloud_);
    sor.setMeanK(point_num_meank_);
    sor.setStddevMulThresh(std_mul_);
    sor.filter(*cloud);

    // @todo inliers and outliers
  }

  void DisplayAllParams() override {
    PARAM_INFO(point_num_meank_);
    PARAM_INFO(std_mul_);
  }

 private:
  int32_t point_num_meank_;
  float std_mul_;
};

}  // namespace filter
}  // namespace pre_processers
}  // namespace static_map
