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

#include "builder/memory_manager.h"

#include <pcl/point_types.h>

#include "builder/trajectory.h"
#include "common/macro_defines.h"
#include "common/simple_time.h"
#include "glog/logging.h"

namespace static_map {

constexpr int kTimeStepSec = 1;

template <typename PointType>
MemoryManager<PointType>::MemoryManager(
    std::vector<std::shared_ptr<Trajectory<PointType>>>* const trajectories)
    : trajectories_(trajectories), quit_(false) {
  CHECK(trajectories_);

  memory_managing_thread_ =
      std::thread(std::bind(&MemoryManager::Processing, this));
}

template <typename PointType>
MemoryManager<PointType>::~MemoryManager() {
  quit_ = true;
  if (memory_managing_thread_.joinable()) {
    memory_managing_thread_.join();
  }
}

template <typename PointType>
void MemoryManager<PointType>::Processing() {
  PRINT_INFO("Start managing memory.");
  while (!quit_.load()) {
    for (auto& trajectory : *trajectories_) {
      for (auto& submap : *trajectory) {
        submap->UpdateInactiveTime(kTimeStepSec);
      }
    }
    SimpleTime::from_sec(static_cast<double>(kTimeStepSec)).sleep();
  }
  PRINT_INFO("End managing memory.");
}

template class MemoryManager<pcl::PointXYZI>;

}  // namespace static_map
