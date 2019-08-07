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

#ifndef REGISTRATORS_MULTIVIEW_REGISTRATOR_INTERFACE_H_
#define REGISTRATORS_MULTIVIEW_REGISTRATOR_INTERFACE_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

namespace static_map {
namespace registrator {

enum MultiviewType { kLumPCL, kMultiviewTypeCount };

template <typename PointT>
class MultiviewInterface {
 public:
  using PointCloudType = pcl::PointCloud<PointT>;
  using PointCloudPtr = typename PointCloudType::Ptr;
  using PointCloudConstPtr = typename PointCloudType::ConstPtr;

  MultiviewInterface() = default;
  ~MultiviewInterface() = default;

  struct InnerFrame {
    PointCloudPtr cloud;
    Eigen::Matrix4f pose;
  };

  inline void AddNewCloud(const PointCloudPtr& cloud,
                          const Eigen::Matrix4f& pose) {
    if (cloud) {
      inner_frames_.push_back({cloud, pose});
    }
  }

  size_t Size() { return inner_frames_.size(); }

  virtual void AlignAll(const PointCloudPtr& output) = 0;

 protected:
  MultiviewType type_;
  std::vector<InnerFrame> inner_frames_;

 private:
};

}  // namespace registrator
}  // namespace static_map

#endif  // REGISTRATORS_MULTIVIEW_REGISTRATOR_INTERFACE_H_
