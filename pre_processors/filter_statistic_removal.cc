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

#include "pre_processors/filter_statistic_removal.h"

#include "pcl/filters/statistical_outlier_removal.h"

namespace static_map {
namespace pre_processers {
namespace filter {

StatisticRemoval::StatisticRemoval()
    : Interface(), point_num_meank_(30), std_mul_(1.) {
  // float params
  INIT_FLOAT_PARAM("std_mul", std_mul_);
  // int32_t params
  INIT_INT32_PARAM("point_num_meank", point_num_meank_);
}

void StatisticRemoval::DisplayAllParams() {
  PARAM_INFO(point_num_meank_);
  PARAM_INFO(std_mul_);
}

void StatisticRemoval::Filter(const data::InnerCloudType::Ptr &cloud) {
  if (!cloud || !Interface::inner_cloud_) {
    LOG(WARNING) << "nullptr cloud, do nothing!" << std::endl;
    return;
  }

  // this->FilterPrepare(cloud);
  // pcl::StatisticalOutlierRemoval sor;
  // sor.setInputCloud(this->inner_cloud_);
  // sor.setMeanK(point_num_meank_);
  // sor.setStddevMulThresh(std_mul_);
  // sor.filter(*cloud);
  // TODO(edward) implement statistic removal upon inner cloud type

  // @todo inliers and outliers
}

}  // namespace filter
}  // namespace pre_processers
}  // namespace static_map
