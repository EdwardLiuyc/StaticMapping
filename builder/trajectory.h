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

#include <boost/thread/pthread/shared_mutex.hpp>

#include "builder/submap.h"
#include "common/macro_defines.h"

namespace pugi {
class xml_node;
}

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

  PROHIBIT_COPY_AND_ASSIGN(Trajectory);

  using Ptr = std::shared_ptr<Trajectory<PointT>>;

  using ReadWriteMutex = boost::shared_mutex;
  using ReadMutexLocker = boost::shared_lock<ReadWriteMutex>;
  using WriteMutexLocker = boost::upgrade_to_unique_lock<ReadWriteMutex>;

  using iterator =
      typename std::vector<std::shared_ptr<Submap<PointT>>>::iterator;
  using const_iterator =
      typename std::vector<std::shared_ptr<Submap<PointT>>>::const_iterator;

  // Element access
  std::shared_ptr<Submap<PointT>> at(size_t n);
  std::shared_ptr<Submap<PointT>>& operator[](size_t n);
  std::shared_ptr<Submap<PointT>>& front();
  std::shared_ptr<Submap<PointT>>& back();
  // Iterators
  iterator begin();
  iterator end();
  // Capacity
  size_t size();
  bool empty();
  void reserve(size_t n);
  // Modifiers
  void push_back(const std::shared_ptr<Submap<PointT>>& submap);
  void shrink_to_fit();

  // Trajectory APIs
  void SetId(int id) { id_ = id; }
  int GetId() const { return id_; }
  void SetEnuOffset(const double x, const double y);
  void ToXmlNode(pugi::xml_node* map_node);
  void SetSavePath(const std::string& path);
  void OutputPathToPointcloud(const std::string& path);

 private:
  // use a read&write mutex locker to ensure efficiency of submap
  // accessment and modifying
  ReadWriteMutex mutex_;
  std::vector<std::shared_ptr<Submap<PointT>>> submaps_;

  int id_;
  // @todo(edward) change to enu offset
  double enu_offset_x_ = 0.;
  double enu_offset_y_ = 0.;
};

template <typename PointT>
inline std::shared_ptr<Submap<PointT>> Trajectory<PointT>::at(size_t n) {
  ReadMutexLocker locker(mutex_);
  return submaps_.at(n);
}

template <typename PointT>
inline std::shared_ptr<Submap<PointT>>& Trajectory<PointT>::operator[](
    size_t n) {
  ReadMutexLocker locker(mutex_);
  return submaps_[n];
}

template <typename PointT>
inline std::shared_ptr<Submap<PointT>>& Trajectory<PointT>::front() {
  ReadMutexLocker locker(mutex_);
  return submaps_.front();
}

template <typename PointT>
inline std::shared_ptr<Submap<PointT>>& Trajectory<PointT>::back() {
  ReadMutexLocker locker(mutex_);
  return submaps_.back();
}

template <typename PointT>
inline typename Trajectory<PointT>::iterator Trajectory<PointT>::begin() {
  ReadMutexLocker locker(mutex_);
  return submaps_.begin();
}

template <typename PointT>
inline typename Trajectory<PointT>::iterator Trajectory<PointT>::end() {
  ReadMutexLocker locker(mutex_);
  return submaps_.end();
}

template <typename PointT>
inline size_t Trajectory<PointT>::size() {
  ReadMutexLocker locker(mutex_);
  return submaps_.size();
}

template <typename PointT>
inline bool Trajectory<PointT>::empty() {
  ReadMutexLocker locker(mutex_);
  return submaps_.empty();
}

template <typename PointT>
inline void Trajectory<PointT>::reserve(size_t n) {
  ReadMutexLocker locker(mutex_);
  submaps_.reserve(n);
}

template <typename PointT>
inline void Trajectory<PointT>::push_back(
    const std::shared_ptr<Submap<PointT>>& submap) {
  CHECK(submap);
  // CHECK(submap->GetId().trajectory_index == id_);

  boost::upgrade_lock<ReadWriteMutex> locker(mutex_);
  WriteMutexLocker write_locker(locker);
  submaps_.push_back(submap);
}

template <typename PointT>
inline void Trajectory<PointT>::shrink_to_fit() {
  boost::upgrade_lock<ReadWriteMutex> locker(mutex_);
  WriteMutexLocker write_locker(locker);
  submaps_.shrink_to_fit();
}

}  // namespace static_map

#endif  // BUILDER_TRAJECTORY_H_
