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

#include "pre_processors/filter_random_sample.h"

namespace static_map {
namespace pre_processers {
namespace filter {

RandomSampler::RandomSampler() : Interface(), sampling_rate_(1.) {
  INIT_FLOAT_PARAM("sampling_rate", sampling_rate_);
}

void RandomSampler::DisplayAllParams() { PARAM_INFO(sampling_rate_); }

bool RandomSampler::ConfigsValid() const {
  return sampling_rate_ >= 0.f && sampling_rate_ <= 1.f;
}

void RandomSampler::Filter(const data::InnerCloudType::Ptr& cloud) {
  if (!cloud || !Interface::inner_cloud_) {
    LOG(WARNING) << "nullptr cloud, do nothing!" << std::endl;
    return;
  }
  if (sampling_rate_ > 0.999) {
    *cloud = *this->inner_cloud_;
    for (int i = 0; i < this->inner_cloud_->points.size(); ++i) {
      this->inliers_.push_back(i);
    }
    this->outliers_.clear();
    return;
  }

  // std::rand() is not reentrant or thread-safe, so this filter
  // can not be optimized to multi-thread version
  this->FilterPrepare(cloud);

  std::random_device rand_dev;
  std::mt19937 eng(rand_dev());
  std::uniform_real_distribution<> distr(0., 1.);  // define the range

  auto& input = this->inner_cloud_;
  const int size = input->points.size();
  this->inliers_.reserve(size);
  this->outliers_.reserve(size);
  cloud->points.reserve(size);
  for (int i = 0; i < size; ++i) {
    if (distr(eng) <= sampling_rate_) {
      cloud->points.push_back(input->points[i]);
      this->inliers_.push_back(i);
    } else {
      this->outliers_.push_back(i);
    }
  }
  this->inliers_.shrink_to_fit();
  this->outliers_.shrink_to_fit();
  cloud->points.shrink_to_fit();
}

}  // namespace filter
}  // namespace pre_processers
}  // namespace static_map
