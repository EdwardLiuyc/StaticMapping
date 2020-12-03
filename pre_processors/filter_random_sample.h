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

#ifndef PRE_PROCESSORS_FILTER_RANDOM_SAMPLE_H_
#define PRE_PROCESSORS_FILTER_RANDOM_SAMPLE_H_

#include <memory>
#include <random>

#include "pre_processors/filter_interface.h"

namespace static_map {
namespace pre_processers {
namespace filter {

class RandomSampler : public Interface {
 public:
  // USE_POINTCLOUD;

  RandomSampler() : Interface(), sampling_rate_(1.) {
    INIT_FLOAT_PARAM("sampling_rate", sampling_rate_);
  }
  ~RandomSampler() = default;
  RandomSampler(const RandomSampler&) = delete;
  RandomSampler& operator=(const RandomSampler&) = delete;

  std::shared_ptr<Interface> CreateNewInstance() override {
    return std::make_shared<RandomSampler>();
  }

  void Filter(const data::InnerCloudType::Ptr& cloud) override {
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
    const int int_sample_rate = sampling_rate_ * 1000;
    std::random_device rand_dev;   // obtain a random number from hardware
    std::mt19937 eng(rand_dev());  // seed the generator
    std::uniform_int_distribution<> distr(0, 1000);  // define the range

    auto& input = this->inner_cloud_;
    const int size = input->points.size();
    this->inliers_.reserve(size);
    this->outliers_.reserve(size);
    cloud->points.reserve(size);
    // i += 4 to speed up
    // do the rand() only 1/4 times
    for (int i = 0; i < size; ++i) {
      if (distr(eng) <= int_sample_rate) {
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

  void DisplayAllParams() override { PARAM_INFO(sampling_rate_); }

 private:
  float sampling_rate_;
};

}  // namespace filter
}  // namespace pre_processers
}  // namespace static_map

#endif  // PRE_PROCESSORS_FILTER_RANDOM_SAMPLE_H_
