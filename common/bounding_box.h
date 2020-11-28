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

#ifndef COMMON_BOUNDING_BOX_H_
#define COMMON_BOUNDING_BOX_H_

#include <vector>

#include "Eigen/Eigen"

namespace static_map {
namespace common {

/// @class CubeBoundingBox: axis aligned bouding box.
/// Notice: this bounding box must be an exact cube.
class CubeBoundingBox {
 public:
  /// @brief CubeBoundingBox: Init the bbox with its center and length of its
  /// edges.
  CubeBoundingBox(const Eigen::Vector3d& center, double length);

  /// @brief CubeBoundingBox: Init the bbox with its left_bottom point and
  /// right_up point.
  CubeBoundingBox(const Eigen::Vector3d& min, const Eigen::Vector3d& max);

  /// @brief GenerateSubBoxes: split the bounding box into 8 sub boxes. for
  /// detailed information of how we sort these boxes, refer to the .cc file.
  std::vector<CubeBoundingBox> GenerateSubBoxes() const;

  /// @brief GetSubBoxIndexForPoint: return the index of the sub CubeBoundingBox
  /// for the given point.
  int GetSubBoxIndexForPoint(const Eigen::Vector3d& point) const;

  /// @brief ContainsPoint: return whether the given point is in the box.
  bool ContainsPoint(const Eigen::Vector3d& point) const;

  /// @brief GetCenter: return center point of the box.
  Eigen::Vector3d GetCenter() const { return center_; }

 private:
  Eigen::Vector3d center_;
  Eigen::Vector3d min_;
  Eigen::Vector3d max_;
  double size_;
  double half_size_;
};

class BoundingBox {
 public:
  BoundingBox();
  /// @brief BoundingBox: Init the bbox with its left_bottom point and
  /// right_up point.
  BoundingBox(const Eigen::Vector3d& min, const Eigen::Vector3d& max);

  /// @brief ContainsPoint: return whether the given point is in the box.
  bool ContainsPoint(const Eigen::Vector3d& point) const;

 private:
  Eigen::Vector3d min_;
  Eigen::Vector3d max_;
};

}  // namespace common
}  // namespace static_map

#endif  // COMMON_BOUNDING_BOX_H_
