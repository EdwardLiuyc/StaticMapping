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

#ifndef REGISTRATORS_ICP_FACTOR_H_
#define REGISTRATORS_ICP_FACTOR_H_

#include <ceres/ceres.h>

namespace static_map {
namespace registrator {

class IcpPointToPlaneFactor {
 public:
  IcpPointToPlaneFactor(const Eigen::Vector3d& query_point,
                        const Eigen::Vector3d& target_point,
                        const Eigen::Vector3d& normal, const double factor)
      : query_(query_point),
        target_(target_point),
        normal_(normal),
        factor_(factor) {}

  template <typename T>
  bool operator()(const T* const t, const T* const q,
                  T* residuals_ptr /* point to plane distance */) const {
    Eigen::Matrix<T, 3, 1> whole_t(t[0], t[1], t[2]);

    Eigen::Quaternion<T> whole_q{q[0], q[1], q[2], q[3]};
    Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};

    Eigen::Quaternion<T> q_current_point =
        q_identity.slerp(T(factor_), whole_q);
    Eigen::Matrix<T, 3, 1> t_current_point = whole_t * T(factor_);

    residuals_ptr[0] =
        (t_current_point + q_current_point * query_.template cast<T>() -
         target_.template cast<T>())
            .template dot(normal_.template cast<T>());
    return true;
  }

  static ceres::CostFunction* Create(const Eigen::Vector3d& query_point,
                                     const Eigen::Vector3d& target_point,
                                     const Eigen::Vector3d& normal,
                                     const double factor) {
    return new ceres::AutoDiffCostFunction<IcpPointToPlaneFactor, 1, 3, 4>(
        new IcpPointToPlaneFactor(query_point, target_point, normal, factor));
  }

 private:
  Eigen::Vector3d query_;
  Eigen::Vector3d target_;
  Eigen::Vector3d normal_;
  double factor_;
};

}  // namespace registrator

}  // namespace static_map

#endif  // REGISTRATORS_ICP_FACTOR_H_
