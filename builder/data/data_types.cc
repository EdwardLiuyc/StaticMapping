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

#include "builder/data/data_types.h"

#include <algorithm>
#include <utility>

#include "common/math.h"
#include "common/performance/simple_prof.h"

namespace static_map {
namespace data {

Eigen::Matrix4d OdomMsg::PoseInMatrix() const {
  Eigen::Matrix4d pose_in_matrix = Eigen::Matrix4d::Identity();
  pose_in_matrix.block(0, 3, 3, 1) = pose.pose.position;
  pose_in_matrix.block(0, 0, 3, 3) = pose.pose.orientation.toRotationMatrix();
  return pose_in_matrix;
}

void OdomMsg::SetPose(const Eigen::Matrix4d& pose_mat) {
  pose.pose.position = pose_mat.block(0, 3, 3, 1);
  pose.pose.orientation =
      Eigen::Quaterniond(Eigen::Matrix3d(pose_mat.block(0, 0, 3, 3)));
}

}  // namespace data
}  // namespace static_map
