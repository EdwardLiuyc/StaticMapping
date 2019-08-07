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

#ifndef COMMON_EIGEN_PITFALLS_AVOIDENCE_H_
#define COMMON_EIGEN_PITFALLS_AVOIDENCE_H_

// for more information, refer to
// "https://eigen.tuxfamily.org/dox/TopicPitfalls.html"

#include "Eigen/Eigen"
#include "Eigen/StdVector"

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector2d);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector4f);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector4d);

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix4f);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix4d);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix2f);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix2d);

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Quaterniond);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Quaternionf);

namespace static_map {
namespace common {}
}  // namespace static_map

#endif  // COMMON_EIGEN_PITFALLS_AVOIDENCE_H_
