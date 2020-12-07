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
#include <map>
#include <string>

#include "builder/submap.h"
#include "common/math.h"
#include "common/simple_thread_pool.h"
#include "registrators/icp_fast.h"
#include "registrators/icp_pointmatcher.h"
#include "tbb/task_group.h"

namespace static_map {
namespace back_end {
namespace {
const std::map<LoopStatus, std::string> kLoopStatusToStr = {
    {LoopStatus::kNoLoop, "No Loop"},
    {LoopStatus::kTryingToCloseLoop, "Trying To Close Loop"},
    {LoopStatus::kEnteringLoop, "Entering Loop"},
    {LoopStatus::kContinousLoop, "Continous Loop"},
    {LoopStatus::kLeavingLoop, "Leaving Loop"}};
}

DetectResult LoopDetector::AddFrame(const std::shared_ptr<Submap>& frame,
                                    bool do_loop_detect) {
  all_frames_.push_back(frame);

  // Update all positions since they could have been changed.
  all_frames_translation_.emplace_back();
  CHECK_EQ(all_frames_.size(), all_frames_translation_.size());
  for (size_t i = 0; i < all_frames_.size(); ++i) {
    all_frames_translation_[i] = all_frames_[i]->GlobalTranslation();
  }

  int32_t current_index = all_frames_.size() - 1;

  DetectResult result;
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
        if (descriptor::matchTwoM2dpDescriptors(frame->GetDescriptor(),
                                                target_frame->GetDescriptor()) >
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
    case LoopStatus::kNoLoop:
      accumulate_loop_detected_count_ = 0;
      if (loop_detection_ == 1) {
        current_status_ = LoopStatus::kTryingToCloseLoop;
        accumulate_loop_detected_count_++;
        if (accumulate_loop_detected_count_ >=
            settings_.trying_detect_loop_count) {
          current_status_ = LoopStatus::kEnteringLoop;
        }
      }
      break;

    case LoopStatus::kTryingToCloseLoop:
      if (loop_detection_ == 1) {
        accumulate_loop_detected_count_++;
        if (accumulate_loop_detected_count_ >=
            settings_.trying_detect_loop_count) {
          current_status_ = LoopStatus::kEnteringLoop;
        }
      } else if (loop_detection_ == 0) {
        current_status_ = LoopStatus::kNoLoop;
      }
      break;

    case LoopStatus::kEnteringLoop:
      if (loop_detection_ == 0) {
        current_status_ = LoopStatus::kTryingToCloseLoop;
      } else if (loop_detection_ == 1) {
        current_status_ = LoopStatus::kContinousLoop;
      }
      break;

    case LoopStatus::kContinousLoop:
      if (loop_detection_ == 0) {
        current_status_ = LoopStatus::kLeavingLoop;
        accumulate_loop_detected_count_ = 0;
        current_min_distance_ = -1;
      }
      break;

    case LoopStatus::kLeavingLoop:
      if (loop_detection_ == 0) {
        current_status_ = LoopStatus::kNoLoop;
      } else if (loop_detection_ == 1) {
        current_status_ = LoopStatus::kTryingToCloseLoop;
      }
      break;

    default:
      break;
  }

  if (loop_detection_ != 0) {
    PRINT_INFO_FMT("Current Mode : %s",
                   kLoopStatusToStr.at(current_status_).c_str());
  }

  if (current_status_ == LoopStatus::kContinousLoop) {
    CHECK(!maybe_close_pair.empty());

    auto pair_close_loop = [&](const std::pair<int, int>& p) {
      DetectResult::LoopEdge loop_edge;
      loop_edge.close_pair_index = p;
      if (CloseLoop(&loop_edge)) {
        result.edges.push_back(loop_edge);
      }
    };

    tbb::task_group tasks;
    for (auto& pair : maybe_close_pair) {
      tasks.run([&] { pair_close_loop(pair); });
    }
    tasks.wait();

    if (!result.edges.empty()) {
      result.close_succeed = CheckResult(result);
      if (settings_.output_matched_cloud && !result.close_succeed) {
        // For debug.
        for (const auto& edge : result.edges) {
          const int source_id = edge.close_pair_index.second;
          const int target_id = edge.close_pair_index.first;
          {
            // TODO(edward) Add api of innercloud to write pcd directly or add
            // tool to view .bin just like pcl_viewer.
            // Output combined cloud using init pose.
            data::InnerCloudType::Ptr none_matched_cloud(
                new data::InnerCloudType);
            all_frames_.at(source_id)
                ->Cloud()
                ->GetInnerCloud()
                ->ApplyTransformToOutput(edge.init_guess,
                                         none_matched_cloud.get());
            *none_matched_cloud +=
                *all_frames_.at(target_id)->Cloud()->GetInnerCloud();

            pcl::PointCloud<pcl::PointXYZI> output_cloud;
            data::ToPclPointCloud(*none_matched_cloud, &output_cloud);
            pcl::io::savePCDFile("pcd/matched_" + std::to_string(target_id) +
                                     "_" + std::to_string(source_id) +
                                     "_init.pcd",
                                 output_cloud);
          }
          {
            // Output combined cloud using init pose.
            data::InnerCloudType::Ptr matched_cloud(new data::InnerCloudType);
            all_frames_.at(source_id)
                ->Cloud()
                ->GetInnerCloud()
                ->ApplyTransformToOutput(edge.transform, matched_cloud.get());
            *matched_cloud +=
                *all_frames_.at(target_id)->Cloud()->GetInnerCloud();

            pcl::PointCloud<pcl::PointXYZI> output_cloud;
            data::ToPclPointCloud(*matched_cloud, &output_cloud);
            pcl::io::savePCDFile("pcd/matched_" + std::to_string(target_id) +
                                     "_" + std::to_string(source_id) +
                                     "_error.pcd",
                                 output_cloud);
          }
        }
      }
    }
  }

  return result;
}

void LoopDetector::SetSearchWindow(const int start_index, const int end_index) {
  CHECK_GE(start_index, 0);
  CHECK_GE(end_index, 0);
  CHECK_GE(end_index, start_index);

  search_window_start_ = start_index;
  search_window_end_ = end_index;
}

bool LoopDetector::CloseLoop(DetectResult::LoopEdge* edge) const {
  const int target_id = edge->close_pair_index.first;
  const int source_id = edge->close_pair_index.second;

  CHECK(all_frames_.size() > target_id && all_frames_.size() > source_id);
  Eigen::Matrix4d init_guess = all_frames_[target_id]->GlobalPose().inverse() *
                               all_frames_[source_id]->GlobalPose();
  // @todo it is a trick, remove it
  init_guess(2, 3) = 0.f;
  // TODO(edward) Fix this bug!! It is quite wrong to simply calculating like
  // this.
  // if (settings_.use_gps && all_frames_[target_id]->HasGps() &&
  //     all_frames_[source_id]->HasGps()) {
  //   // if use gps, update the translation part of guess
  //   const EnuPosition delta_enu =
  //   all_frames_[source_id]->GetRelatedGpsInENU() -
  //                                 all_frames_[target_id]->GetRelatedGpsInENU();
  //   init_guess.block(0, 3, 3, 1) = delta_enu;
  // }
  edge->init_guess = init_guess;

  // TODO(edward) Load the config for submap matching.
  registrator::IcpUsingPointMatcher scan_matcher;
  scan_matcher.SetInputSource(all_frames_[source_id]->Cloud());
  scan_matcher.SetInputTarget(all_frames_[target_id]->Cloud());
  scan_matcher.Align(init_guess, edge->transform);
  const double match_score = scan_matcher.GetFitnessScore();
  if (match_score > settings_.accept_scan_match_score) {
    // match score = exp(-score)
    // so, score = -log_e(match_score)
    edge->score = -std::log(match_score);
    PRINT_INFO_FMT("++++ Got good match from source %d to target %d ++++",
                   source_id, target_id);
    return true;
  }
  return false;
}

bool LoopDetector::CheckResult(const DetectResult& result) const {
  // TODO(edward) CUrrently, this check is rather poor, so disable it.
  // return true;

  if (result.edges.size() <= 1) {
    return false;
  }

  const auto& first_edge = result.edges[0];
  const Eigen::Matrix4d first_source_pose =
      all_frames_[first_edge.close_pair_index.first]->GlobalPose() *
      first_edge.transform;
  for (size_t i = 1; i < result.edges.size(); ++i) {
    const auto& edge = result.edges[i];
    const Eigen::Matrix4d source_edge =
        all_frames_[edge.close_pair_index.first]->GlobalPose() * edge.transform;

    const Eigen::Matrix4d diff = first_source_pose.inverse() * source_edge;
    const double trans_diff = diff.block(0, 3, 3, 1).norm();
    const double rotation_diff = common::RotationMatrixToEulerAngles(
                                     Eigen::Matrix3d(diff.block(0, 0, 3, 3)))
                                     .norm();
    if (trans_diff > 0.25 || rotation_diff > 0.02) {
      PRINT_DEBUG_FMT("Invalid result, %lf, %lf", trans_diff, rotation_diff);
      return false;
    }
  }
  return true;
}

void LoopDetector::SetTransformOdomToLidar(const Eigen::Matrix4d& t) {
  tf_odom_lidar_ = t;
}

}  // namespace back_end
}  // namespace static_map
