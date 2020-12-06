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

#ifndef REGISTRATORS_NDT_H_
#define REGISTRATORS_NDT_H_

#include "pcl/point_types.h"
#include "pclomp/ndt_omp_impl.hpp"
#include "registrators/interface.h"

namespace static_map {
namespace registrator {

class Ndt : public Interface {
 public:
  USE_REGISTRATOR_CLOUDS;

  using NdtRegistrator =
      pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;

  Ndt();
  ~Ndt();

  PROHIBIT_COPY_AND_ASSIGN(Ndt);

  bool Align(const Eigen::Matrix4d& guess, Eigen::Matrix4d& result) override;

 private:
  NdtRegistrator inner_matcher_;
};

}  // namespace registrator
}  // namespace static_map

#endif  // REGISTRATORS_NDT_H_
