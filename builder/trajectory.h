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
#include "tbb/concurrent_vector.h"

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
class Trajectory : public tbb::concurrent_vector<std::shared_ptr<Submap>> {
 public:
  Trajectory() = default;
  ~Trajectory() = default;

  PROHIBIT_COPY_AND_ASSIGN(Trajectory);

  using Ptr = std::shared_ptr<Trajectory>;

  // Trajectory APIs
  void SetId(int id) { id_ = id; }
  int GetId() const { return id_; }
  void SetEnuOffset(const double x, const double y);
  void ToXmlNode(pugi::xml_node* map_node);
  void SetSavePath(const std::string& path);
  void OutputPathToPointcloud(const std::string& path);

 private:
  int id_;
  // @todo(edward) change to enu offset
  double enu_offset_x_ = 0.;
  double enu_offset_y_ = 0.;
};

}  // namespace static_map

#endif  // BUILDER_TRAJECTORY_H_
