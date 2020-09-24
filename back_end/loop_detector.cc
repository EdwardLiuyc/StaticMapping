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

#include "back_end/loop_detector.h"

#include <algorithm>
#include <limits>

#include "builder/submap.h"
#include "common/simple_thread_pool.h"
#include "registrators/icp_fast.h"
#include "tbb/task_group.h"

namespace static_map {
namespace back_end {

template <typename PointT>
typename LoopDetector<PointT>::DetectResult LoopDetector<PointT>::AddFrame(
    const std::shared_ptr<Submap<PointT>>& frame, bool do_loop_detect) {
  all_frames_.push_back(frame);
  const Eigen::Vector3d translation = frame->GlobalTranslation();
  all_frames_translation_.push_back(translation);

  int32_t current_index = all_frames_.size() - 1;
  const char* mode_name[kLoopStatusCount] = {"No Loop", "Trying To Close Loop",
                                             "Entering Loop", "Continous Loop",
                                             "Leaving Loop"};

  typename LoopDetector<PointT>::DetectResult result;
  result.status = current_status_;
  result.current_frame_index = current_index;
  if (!do_loop_detect ||
      all_frames_.size() <= settings_.loop_ignore_threshold) {
    return result;
  }

  const int max_index = static_cast<int>(all_frames_.size()) - 1;
  std::vector<int> indices_in_distance;
  indices_in_distance.reserve(10);

  Eigen::Vector2d cur_trans_2d =
      all_frames_translation_[current_index].topRows(2);
  const double cur_trans_z = all_frames_translation_[current_index][2];
  double min_distance = std::numeric_limits<double>::max();
  int closest_index = -1;

  int start_index = 0;
  int end_index = max_index - settings_.loop_ignore_threshold;
  if (search_window_end_ >= 0 && search_window_start_ >= 0) {
    start_index = common::Clamp(search_window_start_, 0, max_index);
    end_index = common::Clamp(search_window_end_, 0, max_index);
  }
  for (int i = start_index; i < end_index; ++i) {
    Eigen::Vector2d frame_trans_2d = all_frames_translation_[i].topRows(2);
    double xy_distance = (cur_trans_2d - frame_trans_2d).norm();
    double z_distance = std::fabs(cur_trans_z - all_frames_translation_[i][2]);
    if (xy_distance <= settings_.max_close_loop_distance &&
        z_distance <= settings_.max_close_loop_z_distance) {
      indices_in_distance.push_back(i);
      if (xy_distance < min_distance) {
        closest_index = i;
        min_distance = xy_distance;
      }
    }
  }
  if (min_distance >= settings_.max_close_loop_distance * 0.4) {
    closest_index = -1;
  }

  std::vector<int> indices_well_matched;
  if (!indices_in_distance.empty()) {
    if (!settings_.use_descriptor) {
      // if not use descriptor, we do not match the descriptor
      // instead, we get the closest frame
      // indices_well_matched.swap(indices_in_distance);
      std::copy(indices_in_distance.begin(), indices_in_distance.end(),
                std::back_inserter(indices_well_matched));
    } else {
      for (auto& i : indices_in_distance) {
        CHECK_NE(i, current_index);
        auto& target_frame = all_frames_.at(i);
        if (descriptor::matchTwoM2dpDescriptors<PointT>(
                frame->GetDescriptor(), target_frame->GetDescriptor()) >
            settings_.m2dp_match_score) {
          indices_well_matched.push_back(i);
        }
      }
    }
  }

  loop_detection_ = 0;
  std::vector<std::pair<int, int>> maybe_close_pair;
  maybe_close_pair.reserve(settings_.nearest_history_pos_num + 1);
  if (!indices_well_matched.empty()) {
    loop_detection_ = 1;
    std::sort(indices_well_matched.begin(), indices_well_matched.end());
    if (indices_well_matched.size() >= settings_.nearest_history_pos_num * 2) {
      const int step = static_cast<int>(indices_well_matched.size()) /
                       settings_.nearest_history_pos_num;
      for (int i = 0; i < settings_.nearest_history_pos_num; ++i) {
        maybe_close_pair.push_back(
            std::make_pair(indices_well_matched[i * step], current_index));
      }
    } else {
      const int size =
          indices_well_matched.size() > settings_.nearest_history_pos_num
              ? settings_.nearest_history_pos_num
              : indices_well_matched.size();
      for (int i = 0; i < size; ++i) {
        maybe_close_pair.push_back(
            std::make_pair(indices_well_matched[i], current_index));
      }
    }
    if (closest_index >= 0) {
      auto closest_pair = std::make_pair(closest_index, current_index);
      if (std::find(maybe_close_pair.begin(), maybe_close_pair.end(),
                    closest_pair) == maybe_close_pair.end()) {
        maybe_close_pair.push_back(closest_pair);
      }
    }
  }

  switch (current_status_) {
    case kNoLoop:
      accumulate_loop_detected_count_ = 0;
      if (loop_detection_ == 1) {
        current_status_ = kTryingToCloseLoop;
        accumulate_loop_detected_count_++;
        if (accumulate_loop_detected_count_ >=
            settings_.trying_detect_loop_count) {
          current_status_ = kEnteringLoop;
        }
      }
      break;

    case kTryingToCloseLoop:
      if (loop_detection_ == 1) {
        accumulate_loop_detected_count_++;
        if (accumulate_loop_detected_count_ >=
            settings_.trying_detect_loop_count) {
          current_status_ = kEnteringLoop;
        }
      } else if (loop_detection_ == 0) {
        current_status_ = kNoLoop;
      }
      break;

    case kEnteringLoop:
      if (loop_detection_ == 0) {
        current_status_ = kTryingToCloseLoop;
      } else if (loop_detection_ == 1) {
        current_status_ = kContinousLoop;
      }
      break;

    case kContinousLoop:
      if (loop_detection_ == 0) {
        current_status_ = kLeavingLoop;
        accumulate_loop_detected_count_ = 0;
        current_min_distance_ = -1;
      }
      break;

    case kLeavingLoop:
      if (loop_detection_ == 0) {
        current_status_ = kNoLoop;
      } else if (loop_detection_ == 1) {
        current_status_ = kTryingToCloseLoop;
      }
      break;

    default:
      break;
  }

  if (loop_detection_ != 0) {
    PRINT_INFO_FMT("Current Mode : %s", mode_name[(int)current_status_]);
  }

  if (current_status_ == kContinousLoop) {
    CHECK(!maybe_close_pair.empty());

    auto pair_close_loop = [&](const std::pair<int, int>& p) {
      double constraint_score = 0.;
      Eigen::Matrix4d transform;
      if (CloseLoop(p.first, p.second, &transform, &constraint_score)) {
        result.close_pair.push_back(p);
        result.transform.push_back(transform);
        result.constraint_score.push_back(constraint_score);
      }
    };
    // using threadpool
    // common::ThreadPool pool(pair_size);
    // const int pair_size = maybe_close_pair.size();
    // for (auto& pair : maybe_close_pair) {
    //   pool.enqueue(pair_close_loop, pair);
    // }
    tbb::task_group tasks;
    for (auto& pair : maybe_close_pair) {
      tasks.run([&] { pair_close_loop(pair); });
    }
    tasks.wait();

    if (!result.close_pair.empty()) {
      result.close_succeed = true;
    }
  }

  return result;
}

template <typename PointT>
void LoopDetector<PointT>::SetSearchWindow(const int start_index,
                                           const int end_index) {
  CHECK_GE(start_index, 0);
  CHECK_GE(end_index, 0);
  CHECK_GE(end_index, start_index);

  search_window_start_ = start_index;
  search_window_end_ = end_index;
}

template <typename PointT>
bool LoopDetector<PointT>::CloseLoop(const int target_id, const int source_id,
                                     Eigen::Matrix4d* const result,
                                     double* score) {
  // PRINT_INFO("Trying to close loop ...");
  CHECK(all_frames_.size() > target_id && all_frames_.size() > source_id);
  Eigen::Matrix4d init_guess = all_frames_[target_id]->GlobalPose().inverse() *
                               all_frames_[source_id]->GlobalPose();
  // @todo it is a trick, remove it
  init_guess(2, 3) = 0.f;
  if (settings_.use_gps && all_frames_[target_id]->HasGps() &&
      all_frames_[source_id]->HasGps()) {
    // if use gps, update the translation part of guess
    const EnuPosition delta_enu = all_frames_[source_id]->GetRelatedGpsInENU() -
                                  all_frames_[target_id]->GetRelatedGpsInENU();
    init_guess.block(0, 3, 3, 1) = delta_enu;
  }

  // TODO(edward) Load the config for submap matching.
  registrator::IcpFast<PointT> scan_matcher;
  scan_matcher.SetInputSource(all_frames_[source_id]->Cloud());
  scan_matcher.SetInputTarget(all_frames_[target_id]->Cloud());
  scan_matcher.Align(init_guess, *result);
  const double match_score = scan_matcher.GetFitnessScore();
  if (match_score > settings_.accept_scan_match_score) {
    // match score = exp(-score)
    // so, score = -log_e(match_score)
    *score = -std::log(match_score);
    PRINT_INFO_FMT("++++ Got good match from source %d to target %d ++++",
                   source_id, target_id);

    if (settings_.output_matched_cloud) {
      // for debug
      std::cout << "\nlast: " << source_id << std::endl
                << all_frames_.at(source_id)->GlobalPose()
                << "\nfirst: " << target_id << std::endl
                << all_frames_.at(target_id)->GlobalPose() << std::endl;
      PRINT_DEBUG_FMT("match score: %lf, score: %lf", match_score, *score);
      typename pcl::PointCloud<PointT>::Ptr matched_cloud(
          new typename pcl::PointCloud<PointT>);
      pcl::transformPointCloud(
          *all_frames_.at(source_id)->Cloud()->GetPclCloud(), *matched_cloud,
          *result);
      *matched_cloud += *all_frames_.at(target_id)->Cloud()->GetPclCloud();
      pcl::io::savePCDFile("pcd/matched_" + std::to_string(target_id) + "_" +
                               std::to_string(source_id) + ".pcd",
                           *matched_cloud);
    }
    return true;
  }
  return false;
}

template <typename PointT>
void LoopDetector<PointT>::SetTransformOdomToLidar(const Eigen::Matrix4d& t) {
  tf_odom_lidar_ = t;
}

template class LoopDetector<pcl::PointXYZI>;

}  // namespace back_end
}  // namespace static_map
