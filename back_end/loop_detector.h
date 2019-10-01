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

// stl
#include <memory>
#include <utility>
#include <vector>
// local
#include "builder/submap.h"
#include "registrators/icp_pointmatcher.h"

#ifdef _USE_TBB_
#include <tbb/atomic.h>
#include <tbb/concurrent_vector.h>
#endif

namespace static_map {
namespace back_end {

struct LoopDetectorSettings {
  bool use_gps = false;
  bool use_descriptor = false;
  bool output_matched_cloud = false;
  int loop_ignore_threshold = 15;
  int trying_detect_loop_count = 1;
  int nearest_history_pos_num = 4;
  float max_close_loop_distance = 25.f;
  float m2dp_match_score = 0.99f;
  float accept_scan_match_score = 0.75f;
};

template <typename PointT>
class LoopDetector {
 public:
  explicit LoopDetector(const LoopDetectorSettings &l_d_settings)
      : settings_(l_d_settings), tf_odom_lidar_(Eigen::Matrix4f::Identity()) {}
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
    DetectResult() : close_succeed(false) {}
    int current_frame_index;
    LoopStatus status;
    bool close_succeed;
#ifndef _USE_TBB_
    std::vector<std::pair<int, int>> close_pair;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
        transform;
    std::vector<double> constraint_score;
#else
    tbb::concurrent_vector<std::pair<int, int>> close_pair;
    tbb::concurrent_vector<Eigen::Matrix4f,
                           Eigen::aligned_allocator<Eigen::Matrix4f>>
        transform;
    tbb::concurrent_vector<double> constraint_score;
#endif
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  DetectResult AddFrame(const std::shared_ptr<Submap<PointT>> &submap,
                        bool do_loop_detect = true);
  void SetSearchWindow(const int start_index, const int end_index);
  inline std::vector<std::shared_ptr<Submap<PointT>>> &GetFrames() {
    return all_frames_;
  }
  /// @brief set static tf link from odom to lidar(cloud frame)
  void SetTransformOdomToLidar(const Eigen::Matrix4f &t);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  bool CloseLoop(const int first_id, const int last_id, Eigen::Matrix4f *result,
                 double *score);

 private:
  std::vector<std::shared_ptr<Submap<PointT>>> all_frames_;
  std::vector<Eigen::Vector3d> all_frames_translation_;

  int loop_detection_;
  int accumulate_loop_detected_count_;
  float current_min_distance_;

  LoopStatus current_status_ = kNoLoop;
  LoopDetectorSettings settings_;

  Eigen::Matrix4f tf_odom_lidar_;

  int search_window_start_ = -1;
  int search_window_end_ = -1;
};

}  // namespace back_end
}  // namespace static_map

#endif  // BACK_END_LOOP_DETECTOR_H_
