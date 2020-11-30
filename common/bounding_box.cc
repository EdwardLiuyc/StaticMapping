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

#include "common/bounding_box.h"

#include "glog/logging.h"

namespace static_map {
namespace common {

CubeBoundingBox::CubeBoundingBox(const Eigen::Vector3d& center, double size)
    : center_(center),
      min_(center - Eigen::Vector3d::Constant(size * 0.5)),
      max_(center + Eigen::Vector3d::Constant(size * 0.5)),
      size_(size),
      half_size_(0.5 * size_) {}

CubeBoundingBox::CubeBoundingBox(const Eigen::Vector3d& min,
                                 const Eigen::Vector3d& max)
    : center_(0.5 * (min + max)),
      min_(min),
      max_(max),
      size_(max[0] - min[0]),
      half_size_(0.5 * size_) {
  CHECK_DOUBLE_EQ(max[1] - min[1], size_);
  CHECK_DOUBLE_EQ(max[2] - min[2], size_);
}

std::vector<CubeBoundingBox> CubeBoundingBox::GenerateSubBoxes() const {
  /// How do we separate the space(voxel):
  ///  x > 0, y > 0, z > 0: 0
  ///  x < 0, y > 0, z > 0: 1
  ///  x < 0, y < 0, z > 0: 2
  ///  x > 0, y < 0, z > 0: 3
  ///  ... for z < 0 : [4, 7]
  /// "0" means the center of the voxel which we want to split into 8 parts.
  return {CubeBoundingBox(center_, max_),
          CubeBoundingBox(center_ - Eigen::Vector3d(half_size_, 0, 0),
                          max_ - Eigen::Vector3d(half_size_, 0, 0)),
          CubeBoundingBox(center_ - Eigen::Vector3d(half_size_, half_size_, 0),
                          max_ - Eigen::Vector3d(half_size_, half_size_, 0)),
          CubeBoundingBox(center_ - Eigen::Vector3d(0, half_size_, 0),
                          max_ - Eigen::Vector3d(0, half_size_, 0)),
          CubeBoundingBox(center_ - Eigen::Vector3d(0, 0, half_size_),
                          max_ - Eigen::Vector3d(0, 0, half_size_)),
          CubeBoundingBox(center_ - Eigen::Vector3d(half_size_, 0, half_size_),
                          max_ - Eigen::Vector3d(half_size_, 0, half_size_)),
          CubeBoundingBox(
              center_ - Eigen::Vector3d(half_size_, half_size_, half_size_),
              max_ - Eigen::Vector3d(half_size_, half_size_, half_size_)),
          CubeBoundingBox(center_ - Eigen::Vector3d(0, half_size_, half_size_),
                          max_ - Eigen::Vector3d(0, half_size_, half_size_))};
}

int CubeBoundingBox::GetSubBoxIndexForPoint(
    const Eigen::Vector3d& point) const {
  if (!ContainsPoint(point)) {
    return -1;
  }

  const Eigen::Vector3d offseted_point = point - center_;
  int index = 0;
  if (offseted_point[0] >= 0.) {
    // 0 or 3
    if (offseted_point[1] >= 0) {
      index = 0;
    } else {
      index = 3;
    }
  } else {
    // 1 or 2
    if (offseted_point[1] >= 0) {
      index = 1;
    } else {
      index = 2;
    }
  }
  if (offseted_point[2] < 0.) {
    index += 4;
  }
  return index;
}

bool CubeBoundingBox::ContainsPoint(const Eigen::Vector3d& point) const {
  const Eigen::Vector3d offseted_point = (point - center_).cwiseAbs();
  return offseted_point[0] <= half_size_ && offseted_point[1] <= half_size_ &&
         offseted_point[2] <= half_size_;
}

BoundingBox::BoundingBox()
    : min_(Eigen::Vector3d::Zero()), max_(Eigen::Vector3d::Zero()) {}

BoundingBox::BoundingBox(const Eigen::Vector3d& min, const Eigen::Vector3d& max)
    : min_(min), max_(max) {
  CHECK(min_[0] < max_[0] && min_[1] < max_[1] && min_[2] < max_[2]);
}

bool BoundingBox::ContainsPoint(const Eigen::Vector3d& point) const {
  return ((point[0] >= min_[0] && point[0] <= max_[0]) &&
          (point[1] >= min_[1] && point[1] <= max_[1]) &&
          (point[2] >= min_[2] && point[2] <= max_[2]));
}

}  // namespace common
}  // namespace static_map
