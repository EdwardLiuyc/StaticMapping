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

  // Operators
  bool operator==(const FrameId& other) const;
  bool operator!=(const FrameId& other) const;
  bool operator<(const FrameId& other) const;

  std::string DebugString() const;
};

class Frame : public FrameBase {
 public:
  using InnerCloudPtr = typename data::InnerPointCloudData::Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Frame();
  ~Frame() = default;

  PROHIBIT_COPY_AND_ASSIGN(Frame);

  void ToPcdFile(const std::string& filename) override;

  InnerCloudPtr Cloud() override;

 public:
  FrameId id_;
};

}  // namespace static_map

#endif  // BUILDER_FRAME_H_
