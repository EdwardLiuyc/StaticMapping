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

#ifndef BUILDER_TRAJECTORY_H_
#define BUILDER_TRAJECTORY_H_
// stl
#include <memory>
#include <string>
#include <vector>
// local
#include "builder/submap.h"
#include "common/pugixml.hpp"

namespace static_map {

/*
 * @class Trajectory
 * it is a thread safe class for submaps accessing and modifying
 *
 * Layers :
 * |- Trajectoies
 *   |- submaps
 *     |- frames
 *
 * @notice you can use a Trajectory just like a vector of submap
 * for instance :
 *   1. for(auto& submap : trajectory) {}
 *   2. traejctory.push_back(submap)
 *   ...
 */
template <typename PointT>
class Trajectory {
 public:
  Trajectory() {}
  ~Trajectory() {}

  Trajectory(const Trajectory<PointT>&) = delete;
  Trajectory& operator=(const Trajectory<PointT>&) = delete;

  using Ptr = std::shared_ptr<Trajectory<PointT>>;

  using ReadWriteMutex = boost::shared_mutex;
  using ReadMutexLocker = boost::shared_lock<ReadWriteMutex>;
  using WriteMutexLocker = boost::upgrade_to_unique_lock<ReadWriteMutex>;

  using iterator =
      typename std::vector<std::shared_ptr<Submap<PointT>>>::iterator;
  using const_iterator =
      typename std::vector<std::shared_ptr<Submap<PointT>>>::const_iterator;
  // Element access
  inline std::shared_ptr<Submap<PointT>> at(size_t n) {
    ReadMutexLocker locker(mutex_);
    return submaps_.at(n);
  }
  inline std::shared_ptr<Submap<PointT>>& operator[](size_t n) {
    ReadMutexLocker locker(mutex_);
    return submaps_[n];
  }
  inline std::shared_ptr<Submap<PointT>>& front() {
    ReadMutexLocker locker(mutex_);
    return submaps_.front();
  }
  inline std::shared_ptr<Submap<PointT>>& back() {
    ReadMutexLocker locker(mutex_);
    return submaps_.back();
  }

  // Iterators
  inline iterator begin() {
    ReadMutexLocker locker(mutex_);
    return submaps_.begin();
  }
  inline iterator end() {
    ReadMutexLocker locker(mutex_);
    return submaps_.end();
  }
  // Capacity
  size_t size() {
    ReadMutexLocker locker(mutex_);
    return submaps_.size();
  }
  bool empty() {
    ReadMutexLocker locker(mutex_);
    return submaps_.empty();
  }
  inline void reserve(size_t n) {
    ReadMutexLocker locker(mutex_);
    submaps_.reserve(n);
  }

  // Modifiers
  inline void push_back(const std::shared_ptr<Submap<PointT>>& submap) {
    CHECK(submap);
    CHECK(submap->GetId().trajectory_index == id_);

    boost::upgrade_lock<ReadWriteMutex> locker(mutex_);
    WriteMutexLocker write_locker(locker);
    submaps_.push_back(submap);
  }
  inline void shrink_to_fit() {
    boost::upgrade_lock<ReadWriteMutex> locker(mutex_);
    WriteMutexLocker write_locker(locker);
    submaps_.shrink_to_fit();
  }

  // Trajectory APIs
  inline void SetId(int id) { id_ = id; }
  inline int GetId() { return id_; }
  void SetUtmOffset(const double x, const double y);
  void ToXmlNode(pugi::xml_node* map_node);
  void SetSavePath(const std::string& path);

 private:
  // use a read&write mutex locker to ensure efficiency of submap
  // accessment and modifying
  ReadWriteMutex mutex_;
  std::vector<std::shared_ptr<Submap<PointT>>> submaps_;

  int id_;
  double utm_offset_x_ = 0.;
  double utm_offset_y_ = 0.;
};

}  // namespace static_map

#endif  // BUILDER_TRAJECTORY_H_
