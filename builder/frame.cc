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

#include "builder/frame.h"

namespace static_map {

template <typename PointType>
void Frame<PointType>::ToPcdFile(const std::string& filename) {
  const std::string actual_filename =
      filename.empty() ? id_.DebugString() + ".pcd" : filename;
  pcl::io::savePCDFileASCII(actual_filename,
                            *this->inner_cloud_->GetPclCloud());
}

template <typename PointType>
typename Frame<PointType>::InnerCloudPtr Frame<PointType>::Cloud() {
  return this->inner_cloud_;
}

template class Frame<pcl::PointXYZI>;

}  // namespace static_map
