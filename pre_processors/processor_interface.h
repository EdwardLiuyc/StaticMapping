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

#ifndef PRE_PROCESSORS_PROCESSOR_INTERFACE_H_
#define PRE_PROCESSORS_PROCESSOR_INTERFACE_H_

#include <glog/logging.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

#include "builder/data/cloud_types.h"
#include "pre_processors/xml_interface.h"

namespace static_map {
namespace pre_processers {

class ProcesserInterface : public XmlInterface {
 public:
  ProcesserInterface() : XmlInterface(), inner_cloud_(nullptr) {}
  virtual ~ProcesserInterface() {}

  ProcesserInterface(const ProcesserInterface&) = delete;
  ProcesserInterface& operator=(const ProcesserInterface&) = delete;

  virtual void SetInputCloud(const data::InnerCloudType::Ptr& cloud) {
    inliers_.clear();
    outliers_.clear();
    if (cloud == nullptr || cloud->points.empty()) {
      LOG(WARNING) << "cloud empty, do nothing!" << std::endl;
      inner_cloud_ = nullptr;
      return;
    }
    inner_cloud_ = cloud;
  }

  virtual const std::vector<int>& Inliers() const { return inliers_; }
  virtual const std::vector<int>& Outliers() const { return outliers_; }

 protected:
  /// @brief it is just a pointer, no memory allocated
  data::InnerCloudType::Ptr inner_cloud_;
  /// @brief should be sorted from small to large
  std::vector<int> inliers_;
  /// @brief should be sorted from small to large
  std::vector<int> outliers_;
};

}  // namespace pre_processers
}  // namespace static_map

#endif  // PRE_PROCESSORS_PROCESSOR_INTERFACE_H_
