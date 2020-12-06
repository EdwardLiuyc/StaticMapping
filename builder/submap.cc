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

#include <memory>

#include "builder/multi_resolution_voxel_map.h"
#include "builder/submap.h"
#include "common/file_utils.h"
#include "common/performance/simple_prof.h"
#include "common/point_utils.h"
#include "common/simple_time.h"
#include "pcl/filters/random_sample.h"
#include "pcl/io/pcd_io.h"
#include "pcl/io/vtk_io.h"
#include "pre_processors/filter_voxel_grid.h"
#include "pre_processors/random_sample_with_plane_detect.h"
#include "registrators/multiview_registrator_lum_pcl.h"

namespace static_map {

using data::InnerPointCloudData;

using common::ReadMutexLocker;
using common::ReadWriteMutex;
using common::WriteMutexLocker;

bool SubmapId::operator==(const SubmapId& other) const {
  return std::forward_as_tuple(trajectory_index, submap_index) ==
         std::forward_as_tuple(other.trajectory_index, other.submap_index);
}

bool SubmapId::operator!=(const SubmapId& other) const {
  return !operator==(other);
}

bool SubmapId::operator<(const SubmapId& other) const {
  return std::forward_as_tuple(trajectory_index, submap_index) <
         std::forward_as_tuple(other.trajectory_index, other.submap_index);
}

std::string SubmapId::DebugString() const {
  std::ostringstream out;
  out << "(" << trajectory_index << "," << submap_index << ")";
  return out.str();
}

using static_map::registrator::MultiviewRegistratorLumPcl;

Submap::Submap(const SubmapOptions& options)
    : FrameBase(),
      options_(options),
      save_filename_(""),
      full_(false),
      got_matched_transform_to_next_(false),
      cloud_inactive_time_(0) {}

void Submap::InsertFrame(const std::shared_ptr<Frame>& frame) {
  CHECK(frame != nullptr);
  CHECK(!full_.load());

  if (frames_.empty()) {
    // This is the first time that we set the global pose of the submap, we do
    // not need a mutex the protect the pose.
    this->global_pose_ = frame->GlobalPose();
    frame->SetLocalPose(Eigen::Matrix4d::Identity());
    this->SetTimeStamp(frame->GetTimeStamp());
  } else {
    frame->SetLocalPose(this->global_pose_.inverse() * frame->GlobalPose());
  }

  frame->id_.frame_index = frames_.size();
  frame->id_.submap_index = id_.submap_index;
  frame->id_.trajectory_index = id_.trajectory_index;
  frames_.push_back(frame);
  if (frames_.size() == options_.frame_count) {
    full_ = true;
  }

  if (full_.load()) {
    data::InnerCloudType::Ptr processed_cloud(new data::InnerCloudType);
    if (options_.enable_inner_mrvm) {
      REGISTER_BLOCK("MRVM in submap");
      MultiResolutionVoxelMap map;
      for (auto& frame : frames_) {
        data::InnerCloudType::Ptr transformed_cloud(new data::InnerCloudType);
        frame->Cloud()->GetInnerCloud()->ApplyTransformToOutput(
            frame->LocalPose(), transformed_cloud.get());
        if (options_.enable_check) {
          FATAL_CHECK_CLOUD(transformed_cloud);
        }
        map.InsertPointCloud(transformed_cloud,
                             frame->LocalTranslation().cast<float>());
      }
      // TODO(edward) Output to the cloud.
    } else {
      data::InnerCloudType::Ptr added_cloud(new data::InnerCloudType);
      for (auto& frame : frames_) {
        data::InnerCloudType::Ptr transformed_cloud(new data::InnerCloudType);
        frame->Cloud()->GetInnerCloud()->ApplyTransformToOutput(
            frame->LocalPose(), transformed_cloud.get());
        if (options_.enable_check) {
          FATAL_CHECK_CLOUD(added_cloud);
        }
        *(added_cloud) += *transformed_cloud;
      }
      added_cloud->stamp = frames_[0]->Cloud()->GetInnerCloud()->stamp;
      *processed_cloud = *added_cloud;
    }

    // check if the submap is valid
    if (options_.enable_check) {
      FATAL_CHECK_CLOUD(processed_cloud);
    }

    // TODO(edward) enable this block after refactoring this filter.
    // if (options_.enable_random_sampleing) {
    //   RandomSamplerWithPlaneDetect<PointType> random_sampler;
    //   random_sampler.SetSamplingRate(options_.random_sampling_rate);
    //   random_sampler.SetInputCloud(processed_cloud);
    //   PointCloudPtr filter_cloud(new PointCloudType);
    //   random_sampler.Filter(filter_cloud);
    //   *processed_cloud = *filter_cloud;
    // }

    if (options_.enable_voxel_filter && !processed_cloud->points.empty()) {
      data::InnerCloudType::Ptr filtered_cloud(new data::InnerCloudType);
      pre_processers::filter::VoxelGrid voxel_filter;

      const std::string filter_config_text =
          "<filter name=\"RandomSampler\" >"
          "<param type=\"1\" name=\"voxel_size\">" +
          std::to_string(options_.voxel_size) +
          "</param>"
          "</filter>";
      voxel_filter.InitFromXmlText(filter_config_text.c_str());
      voxel_filter.SetInputCloud(processed_cloud);
      voxel_filter.Filter(filtered_cloud);
      *processed_cloud = *filtered_cloud;
    }

    this->inner_cloud_.reset(new InnerPointCloudData(processed_cloud));
    this->inner_cloud_->CalculateNormals();
  }
}

void Submap::SetId(const SubmapId& id) {
  id_ = id;
  if (save_filename_.empty()) {
    save_filename_ = options_.saving_name_prefix +
                     std::to_string(id_.trajectory_index) + "_" +
                     std::to_string(id_.submap_index) + ".bin";
  }
}

void Submap::SetSavePath(const std::string& path) {
  save_path_ = path;
  CHECK(common::FileExist(save_path_));
}

std::vector<std::shared_ptr<Frame>>& Submap::GetFrames() {
  ReadMutexLocker locker(mutex_);
  return frames_;
}

std::shared_ptr<Frame> Submap::GetFrame(const FrameId& frame_id) {
  if (frame_id.submap_index != id_.submap_index ||
      frame_id.trajectory_index != id_.trajectory_index) {
    return nullptr;
  }

  ReadMutexLocker locker(mutex_);
  if (frame_id.frame_index >= frames_.size()) {
    return nullptr;
  }
  return frames_.at(frame_id.frame_index);
}

void Submap::SetMatchedTransformedToNext(const Eigen::Matrix4d& t) {
  CHECK(!got_matched_transform_to_next_.load());
  boost::upgrade_lock<ReadWriteMutex> locker(mutex_);
  {
    WriteMutexLocker write_locker(locker);
    this->SetTransformToNext(t);
  }
  if (options_.enable_disk_saving) {
    CHECK(!save_filename_.empty());
    ToPcdFile(save_path_ + save_filename_);

    // Clear frame cloud as well.
    for (const auto& frame : frames_) {
      frame->ToPcdFile(save_path_ + frame->id_.DebugString() + ".pcd");
      frame->ClearCloud();
    }
  }
  got_matched_transform_to_next_ = true;
}

typename Submap::InnerCloudPtr Submap::Cloud() {
  ReadMutexLocker locker(mutex_);
  cloud_inactive_time_ = 0;
  CHECK(this->inner_cloud_ && this->inner_cloud_->CloudInMemory());
  return this->inner_cloud_;
}

bool Submap::UpdateInactiveTime(const int update_time_in_sec) {
  if (!options_.enable_disk_saving || !got_matched_transform_to_next_.load()) {
    return true;
  }

  boost::upgrade_lock<ReadWriteMutex> locker(mutex_);
  cloud_inactive_time_ += update_time_in_sec;
  if (cloud_inactive_time_ > options_.disk_saving_delay) {
    WriteMutexLocker write_locker(locker);
    this->inner_cloud_->Clear();
  }
  return true;
}

void Submap::ToPcdFile(const std::string& filename) {
  CHECK(this->inner_cloud_ && !this->inner_cloud_->Empty());
  this->inner_cloud_->SaveToFile(filename);
}

void Submap::ToInfoFile(const std::string& filename) {
  std::ofstream info_file(filename, std::ios::out | std::ios::binary);
  if (info_file.is_open()) {
    info_file.close();
  } else {
    PRINT_ERROR_FMT("Cannot open file: %s", filename.c_str());
  }
}

void Submap::ClearCloudInFrames() {
  CHECK(full_.load());
  for (auto& frame : frames_) {
    frame->ClearCloud();
  }
}

void Submap::AddConnectedSubmap(const SubmapId& id) {
  // do not need locker now
  // boost::upgrade_lock<ReadWriteMutex> locker(mutex_);
  // WriteMutexLocker write_locker(locker);
  connected_submaps_.push_back(id);
}

std::vector<SubmapId>& Submap::GetConnected() { return connected_submaps_; }

void Submap::UpdateInnerFramePose() {
  boost::upgrade_lock<ReadWriteMutex> locker(mutex_);
  WriteMutexLocker write_locker(locker);
  for (auto& frame : frames_) {
    frame->SetGlobalPose(this->global_pose_ * frame->LocalPose());
  }
}

std::string Submap::SavedFileName() {
  if (options_.enable_disk_saving) {
    return save_filename_;
  }
  return "";
}

void Submap::SetSavedFileName(const std::string& filename) {
  save_filename_ = filename;
}

}  // namespace static_map
