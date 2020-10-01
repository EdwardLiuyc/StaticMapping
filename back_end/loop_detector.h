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

#ifndef BACK_END_LOOP_DETECTOR_H_
#define BACK_END_LOOP_DETECTOR_H_

#include <Eigen/Eigen>
// stl
#include <memory>
#include <utility>
#include <vector>

#include "back_end/loop_detector_options.h"
#include "tbb/atomic.h"
#include "tbb/concurrent_vector.h"

namespace static_map {

template <typename PointType>
class Submap;

namespace back_end {

template <typename PointT>
class LoopDetector {
 public:
  explicit LoopDetector(const LoopDetectorSettings &l_d_settings)
      : settings_(l_d_settings), tf_odom_lidar_(Eigen::Matrix4d::Identity()) {}
  ~LoopDetector() {}

  LoopDetector(const LoopDetector &) = delete;
  LoopDetector &operator=(const LoopDetector &) = delete;

  enum LoopStatus {
    kNoLoop,
    kTryingToCloseLoop,
    kEnteringLoop,
    kContinousLoop,
    kLeavingLoop,
    kLoopStatusCount
  };

  struct DetectResult {
    int current_frame_index;
    LoopStatus status;
    // Is it or not a good loop detection result.
    bool close_succeed = false;
    // Pairs of indices of frame that matched.
    tbb::concurrent_vector<std::pair<int, int>> close_pair;
    // The transforms of pairs.
    tbb::concurrent_vector<Eigen::Matrix4d,
                           Eigen::aligned_allocator<Eigen::Matrix4d>>
        transform;
    // Matching scores.
    tbb::concurrent_vector<double> constraint_score;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  DetectResult AddFrame(const std::shared_ptr<Submap<PointT>> &submap,
                        bool do_loop_detect = true);
  void SetSearchWindow(const int start_index, const int end_index);
  std::vector<std::shared_ptr<Submap<PointT>>> &GetFrames();

  /// @brief set static tf link from odom to lidar(cloud frame)
  void SetTransformOdomToLidar(const Eigen::Matrix4d &t);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  bool CloseLoop(const int first_id, const int last_id,
                 Eigen::Matrix4d *const result, double *score);

  bool CheckResult(const DetectResult &result);

 private:
  std::vector<std::shared_ptr<Submap<PointT>>> all_frames_;
  std::vector<Eigen::Vector3d> all_frames_translation_;

  int loop_detection_;
  int accumulate_loop_detected_count_;
  float current_min_distance_;

  LoopStatus current_status_ = kNoLoop;
  LoopDetectorSettings settings_;

  Eigen::Matrix4d tf_odom_lidar_;

  int search_window_start_ = -1;
  int search_window_end_ = -1;
};

template <typename PointT>
inline std::vector<std::shared_ptr<Submap<PointT>>>
    &LoopDetector<PointT>::GetFrames() {
  return all_frames_;
}

}  // namespace back_end
}  // namespace static_map

#endif  // BACK_END_LOOP_DETECTOR_H_
