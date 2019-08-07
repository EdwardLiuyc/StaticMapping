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

#include <glog/logging.h>
#include <fstream>
#include <memory>
#include <string>
#include "builder/msg_conversion.h"
#include "registrators/registrator_interface.h"

namespace static_map {
namespace registrator {

template <typename PointType>
class IcpUsingPointMatcher : public Interface<PointType> {
 public:
  USE_REGISTRATOR_CLOUDS;
  using PM = PointMatcher<float>;

  explicit IcpUsingPointMatcher(const std::string& ymal_file = "")
      : Interface<PointType>(),
        reference_cloud_(new PM::DataPoints),
        reading_cloud_(new PM::DataPoints) {
    Interface<PointType>::type_ = kIcpPM;
    loadConfig(ymal_file);
  }
  ~IcpUsingPointMatcher() {
    reference_cloud_.reset();
    reading_cloud_.reset();
  }

  inline void setInputSource(const PointCloudSourcePtr& cloud) override {
    if (!cloud || cloud->empty()) {
      PRINT_ERROR("Empty cloud.");
      return;
    }
    *reading_cloud_ =
        sensors::pclPointCloudToLibPointMatcherPoints<PointType>(cloud);

    CHECK(reading_cloud_->getNbPoints() == cloud->points.size());
  }

  inline void setInputTarget(const PointCloudTargetPtr& cloud) override {
    if (!cloud || cloud->empty()) {
      PRINT_ERROR("Empty cloud.");
      return;
    }
    *reference_cloud_ =
        sensors::pclPointCloudToLibPointMatcherPoints<PointType>(cloud);

    CHECK(reference_cloud_->getNbPoints() == cloud->points.size());
  }

  bool align(const Eigen::Matrix4f& guess, Eigen::Matrix4f& result) override;

 protected:
  void loadDefaultConfig();
  void loadConfig(const std::string& yaml_filename);

 private:
  std::shared_ptr<PM::DataPoints> reference_cloud_;
  std::shared_ptr<PM::DataPoints> reading_cloud_;

  PM::ICP pm_icp_;
};

}  // namespace registrator
}  // namespace static_map
