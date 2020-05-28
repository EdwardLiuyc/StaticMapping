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
#include <vector>

#include "nabo/nabo.h"
#include "registrators/registrator_interface.h"

namespace static_map {
namespace registrator {

struct BuildData;
using NNS = Nabo::NearestNeighbourSearch<double>;

template <typename PointType>
class IcpFast : public Interface<PointType> {
 public:
  USE_REGISTRATOR_CLOUDS;

  IcpFast() { this->type_ = kFastIcp; }

  void setInputSource(const PointCloudSourcePtr& cloud) override;
  void setInputTarget(const PointCloudTargetPtr& cloud) override;
  bool align(const Eigen::Matrix4f& guess, Eigen::Matrix4f& result) override;

 private:
  std::shared_ptr<BuildData> source_cloud_;
  std::shared_ptr<BuildData> target_cloud_;
  std::shared_ptr<NNS> nns_kdtree_;
};

}  // namespace registrator
}  // namespace static_map
