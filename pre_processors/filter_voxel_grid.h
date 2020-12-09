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

#ifndef PRE_PROCESSORS_FILTER_VOXEL_GRID_H_
#define PRE_PROCESSORS_FILTER_VOXEL_GRID_H_

#include <memory>
#include <unordered_map>
#include <vector>

#include "common/eigen_hash.h"
#include "pre_processors/filter_interface.h"

namespace static_map {
namespace pre_processers {
namespace filter {

class VoxelGrid : public Interface {
 public:
  VoxelGrid();
  ~VoxelGrid() = default;

  PROHIBIT_COPY_AND_ASSIGN(VoxelGrid);

  std::shared_ptr<Interface> CreateNewInstance() override {
    return std::make_shared<VoxelGrid>();
  }

  bool ConfigsValid() const override;

  void Filter(const data::InnerCloudType::Ptr& cloud) override;

  void DisplayAllParams() override;

 private:
  float voxel_size_ = 0.1f;
};

}  // namespace filter
}  // namespace pre_processers
}  // namespace static_map

#endif  // PRE_PROCESSORS_FILTER_VOXEL_GRID_H_
