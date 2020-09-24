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

#ifndef BUILDER_FRAME_H_
#define BUILDER_FRAME_H_

// stl
#include <string>
// local
#include "builder/frame_base.h"

namespace static_map {

struct FrameId {
  int32_t trajectory_index;
  int32_t submap_index;
  int32_t frame_index;  // in submap

  bool operator==(const FrameId& other) const {
    return (trajectory_index == other.trajectory_index &&
            submap_index == other.submap_index &&
            frame_index == other.frame_index);
  }

  bool operator!=(const FrameId& other) const { return !operator==(other); }

  bool operator<(const FrameId& other) const {
    return std::forward_as_tuple(trajectory_index, submap_index, frame_index) <
           std::forward_as_tuple(other.trajectory_index, other.submap_index,
                                 other.frame_index);
  }

  std::string DebugString() const {
    std::ostringstream out;
    out << "frame" << trajectory_index << "_" << submap_index << "_"
        << frame_index;
    return out.str();
  }
};

template <typename PointType>
class Frame : public FrameBase<PointType> {
 public:
  using PointCloudType = pcl::PointCloud<PointType>;
  using PointCloudPtr = typename PointCloudType::Ptr;
  using PointCloudConstPtr = typename PointCloudType::ConstPtr;
  using InnerCloudPtr = typename sensors::InnerPointCloudData<PointType>::Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Frame() : FrameBase<PointType>() {}
  ~Frame() {}

  Frame(const Frame&) = delete;
  Frame& operator=(const Frame&) = delete;

  FrameId id_;

  void ToPcdFile(const std::string& filename) override;

  InnerCloudPtr Cloud() override;
};

}  // namespace static_map

#endif  // BUILDER_FRAME_H_
