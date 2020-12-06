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

#ifndef PRE_PROCESSORS_FILTER_GROUND_REMOVAL_H_
#define PRE_PROCESSORS_FILTER_GROUND_REMOVAL_H_

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "pre_processors/filter_interface.h"

namespace static_map {
namespace pre_processers {
namespace filter {

class GroundRemoval : public Interface {
 public:
  GroundRemoval();
  ~GroundRemoval() {}

  PROHIBIT_COPY_AND_ASSIGN(GroundRemoval);

  std::shared_ptr<Interface> CreateNewInstance() override {
    return std::make_shared<GroundRemoval>();
  }

  void SetInputCloud(const data::InnerCloudType::Ptr& cloud) override;

  void Filter(const data::InnerCloudType::Ptr& cloud) override;

  void DisplayAllParams() override;

 private:
  struct VectorCompare {
    bool operator()(const Eigen::Vector3i a, const Eigen::Vector3i b) const {
      return std::forward_as_tuple(a[0], a[1], a[2]) <
             std::forward_as_tuple(b[0], b[1], b[2]);
    }
  };
  std::map<Eigen::Vector3i, std::vector<int>, VectorCompare> voxels_;

  // parameters
  int32_t min_point_num_in_voxel_;
  float leaf_size_;
  float height_threshold_;
};

}  // namespace filter
}  // namespace pre_processers
}  // namespace static_map

#endif  // PRE_PROCESSORS_FILTER_GROUND_REMOVAL_H_
