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

#include "registrators/icp_libicp.h"
#include "common/macro_defines.h"

namespace static_map {
namespace registrator {

template <typename PointType>
bool IcpUsingLibicp<PointType>::align(const Eigen::Matrix4f& guess,
                                      Eigen::Matrix4f& result) {
  if (!target_cloud_ || !source_cloud_) {
    PRINT_ERROR("Empty cloud.");
    return false;
  }

  Matrix R = Matrix::eye(3);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      R.val[i][j] = guess(i, j);
    }
  }

  Matrix t(3, 1);
  t.val[0][0] = guess(0, 3);
  t.val[1][0] = guess(1, 3);
  t.val[2][0] = guess(2, 3);

  double icp_score = icp_->fit(source_cloud_, source_size_, R, t, 0.1);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) result(i, j) = R.val[i][j];

  for (int i = 0; i < 3; ++i) {
    result(i, 3) = t.val[i][0];
    result(3, i) = 0.;
  }
  result(3, 3) = 1.;

  this->final_score_ = std::exp(-icp_score);
  return true;
}

template <typename PointType>
IcpUsingLibicp<PointType>::~IcpUsingLibicp() {
  if (target_cloud_) {
    free(target_cloud_);
  }
  if (source_cloud_) {
    free(source_cloud_);
  }
}

template class IcpUsingLibicp<pcl::PointXYZI>;
template class IcpUsingLibicp<pcl::PointXYZ>;

}  // namespace registrator
}  // namespace static_map
