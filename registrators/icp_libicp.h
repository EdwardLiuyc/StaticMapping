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

#include "libicp/icpPointToPlane.h"
#include "libicp/icpPointToPoint.h"

#include "registrators/registrator_interface.h"

namespace static_map {
namespace registrator {

template <typename PointType>
class IcpUsingLibicp : public Interface<PointType> {
 public:
  USE_REGISTRATOR_CLOUDS;

  enum IcpType { kPointToPoint, kPointToPlane };

  explicit IcpUsingLibicp(
      IcpType type = kPointToPlane /* default point-plane */)
      : Interface<PointType>(), icp_type_(type) {
    this->type_ = kLibicp;
  }

  ~IcpUsingLibicp();

  inline void setInputSource(const PointCloudSourcePtr& cloud) override {
    if (source_cloud_) {
      free(source_cloud_);
    }
    source_size_ = cloud->size();
    source_cloud_ =
        reinterpret_cast<double*>(calloc(3 * source_size_, sizeof(double)));
    for (size_t i = 0; i < source_size_; ++i) {
      source_cloud_[i * 3 + 0] = (*cloud)[i].x;
      source_cloud_[i * 3 + 1] = (*cloud)[i].y;
      source_cloud_[i * 3 + 2] = (*cloud)[i].z;
    }
  }

  inline void setInputTarget(const PointCloudTargetPtr& cloud) override {
    if (target_cloud_) {
      free(target_cloud_);
    }
    target_cloud_ =
        reinterpret_cast<double*>(calloc(3 * cloud->size(), sizeof(double)));
    for (int i = 0; i < cloud->size(); ++i) {
      target_cloud_[i * 3 + 0] = (*cloud)[i].x;
      target_cloud_[i * 3 + 1] = (*cloud)[i].y;
      target_cloud_[i * 3 + 2] = (*cloud)[i].z;
    }

    if (icp_ != nullptr) {
      icp_.reset();
    }

    if (icp_type_ == kPointToPoint) {
      icp_ = std::make_shared<IcpPointToPoint>(target_cloud_, cloud->size(), 3);
    } else {
      icp_ = std::make_shared<IcpPointToPlane>(target_cloud_, cloud->size(), 3);
    }
  }

  bool align(const Eigen::Matrix4f& guess, Eigen::Matrix4f& result) override;

 private:
  IcpType icp_type_;

  double* target_cloud_ = NULL;
  double* source_cloud_ = NULL;
  size_t source_size_ = 0;
  std::shared_ptr<Icp> icp_ = nullptr;
};

}  // namespace registrator
}  // namespace static_map
