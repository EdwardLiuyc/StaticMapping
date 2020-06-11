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

#ifndef BUILDER_MEMORY_MANAGER_H_
#define BUILDER_MEMORY_MANAGER_H_

#include <atomic>
#include <memory>
#include <thread>
#include <vector>

namespace static_map {

template <typename PointType>
class Trajectory;

template <typename PointType>
class MemoryManager {
 public:
  explicit MemoryManager(
      std::vector<std::shared_ptr<Trajectory<PointType>>>* const trajectories);
  ~MemoryManager();

 private:
  /// @brief managing submaps between RAM and Disk
  void Processing();
  // todo(edward) use tbb::vector instead of std::vector
  std::vector<std::shared_ptr<Trajectory<PointType>>>* const trajectories_;
  std::thread memory_managing_thread_;
  std::atomic_bool quit_;
};

}  // namespace static_map

#endif  // BUILDER_MEMORY_MANAGER_H_
