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

#include "builder/submap.h"
#include "builder/multi_resolution_voxel_map.h"
#include "builder/random_sample_with_plane_detect.h"
#include "common/make_unique.h"
#include "common/point_utils.h"
#include "common/simple_time.h"
#include "registrators/multiview_registrator_lum_pcl.h"

namespace static_map {

using static_map::registrator::MultiviewRegistratorLumPcl;

template <typename PointType>
void Submap<PointType>::InsertFrame(
    const std::shared_ptr<Frame<PointType>>& frame) {
  CHECK(frame != nullptr);
  CHECK(!full_.load());

  if (frames_.empty()) {
    this->global_pose_ = frame->GlobalPose();
    frame->SetLocalPose(Eigen::Matrix4f::Identity());
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
    PointCloudPtr output_cloud(new PointCloudType);

    MultiResolutionVoxelMap<PointType> map;
    // todo add a parameter : z offset
    // map.SetOffsetZ(1.2);
    for (auto& frame : frames_) {
      pcl::transformPointCloud(*frame->Cloud(), *output_cloud,
                               frame->LocalPose());
      if (options_.enable_check) {
        FATAL_CHECK_CLOUD(output_cloud);
      }
      map.InsertPointCloud(output_cloud, frame->LocalTranslation());
    }
    map.OutputToPointCloud(0.51, this->cloud_);

    // check if the submap is valid
    if (options_.enable_check) {
      FATAL_CHECK_CLOUD(this->cloud_);
    }

    if (options_.enable_inner_multiview_icp) {
      MultiviewRegistratorLumPcl<PointType> multi_matcher;
      for (auto& frame : frames_) {
        multi_matcher.AddNewCloud(frame->Cloud(), frame->LocalPose());
      }
      multi_matcher.AlignAll(this->cloud_);
    }

    if (options_.enable_random_sampleing) {
      RandomSamplerWithPlaneDetect<PointType> random_sampler;
      random_sampler.SetSamplingRate(options_.random_sampling_rate);
      random_sampler.SetInputCloud(this->cloud_);
      PointCloudPtr filtered_final_cloud(new PointCloudType);
      random_sampler.Filter(filtered_final_cloud);
      *this->cloud_ = *filtered_final_cloud;
      filtered_final_cloud.reset();
    }

    if (options_.enable_voxel_filter && !this->cloud_->empty()) {
      pcl::ApproximateVoxelGrid<PointType> approximate_voxel_filter;
      approximate_voxel_filter.setLeafSize(0.1, 0.1, 0.1);
      PointCloudPtr filtered_final_cloud(new PointCloudType);
      approximate_voxel_filter.setInputCloud(this->cloud_);
      approximate_voxel_filter.filter(*filtered_final_cloud);
      *this->cloud_ = *filtered_final_cloud;
      filtered_final_cloud.reset();
    }

    is_cloud_in_memory_ = true;
  }
}

template <typename PointType>
void Submap<PointType>::SetId(const SubmapId& id) {
  id_ = id;
  if (save_filename_.empty()) {
    save_filename_ = options_.saving_name_prefix +
                     std::to_string(id_.trajectory_index) + "_" +
                     std::to_string(id_.submap_index) + ".pcd";
  }
}

template <typename PointType>
void Submap<PointType>::SetSavePath(const std::string& path) {
  save_path_ = path;
}

template <typename PointType>
std::vector<std::shared_ptr<Frame<PointType>>>& Submap<PointType>::GetFrames() {
  ReadMutexLocker locker(mutex_);
  return frames_;
}

template <typename PointType>
std::shared_ptr<Frame<PointType>> Submap<PointType>::GetFrame(
    const FrameId& frame_id) {
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

template <typename PointType>
void Submap<PointType>::SetMatchedTransformedToNext(const Eigen::Matrix4f& t) {
  CHECK(!got_matched_transform_to_next_.load());
  boost::upgrade_lock<ReadWriteMutex> locker(mutex_);
  {
    WriteMutexLocker write_locker(locker);
    this->SetTransformToNext(t);
  }
  if (options_.enable_disk_saving) {
    CHECK(!save_filename_.empty());
    ToPcdFile(save_path_ + save_filename_);
  }
  got_matched_transform_to_next_ = true;
}

template <typename PointType>
typename Submap<PointType>::PointCloudPtr Submap<PointType>::Cloud() {
  cloud_inactive_time_ = 0;
  boost::upgrade_lock<ReadWriteMutex> locker(mutex_);
  if (!is_cloud_in_memory_.load()) {
    WriteMutexLocker write_locker(locker);
    CHECK(pcl::io::loadPCDFile<PointType>(save_path_ + save_filename_,
                                          *this->cloud_) == 0);
    is_cloud_in_memory_ = true;
    // PRINT_DEBUG_FMT("get submap data %d from disk.", id_.submap_index);
  }
  return this->cloud_;
}

template <typename PointType>
bool Submap<PointType>::UpdateInactiveTime(const int update_time_in_sec) {
  if (!options_.enable_disk_saving) {
    return true;
  }
  if (!got_matched_transform_to_next_.load()) {
    return true;
  }

  boost::upgrade_lock<ReadWriteMutex> locker(mutex_);
  if (is_cloud_in_memory_.load()) {
    cloud_inactive_time_ += update_time_in_sec;
    if (cloud_inactive_time_ > options_.disk_saving_delay) {
      WriteMutexLocker write_locker(locker);
      this->cloud_->points.clear();
      this->cloud_->points.shrink_to_fit();
      is_cloud_in_memory_ = false;
      // PRINT_DEBUG_FMT("Remove submap %d from RAM.", id_.submap_index);
    }
  }
  return is_cloud_in_memory_.load();
}

template <typename PointType>
void Submap<PointType>::ToPcdFile(const std::string& filename) {
  if (!full_.load() || this->cloud_ == nullptr || this->cloud_->empty()) {
    return;
  }

  // PRINT_INFO("Export submap pcd file.");
  if (!filename.empty()) {
    pcl::io::savePCDFileBinaryCompressed(filename, *this->cloud_);
  } else {
    pcl::io::savePCDFileBinaryCompressed(
        "submap_" + std::to_string(id_.submap_index) + ".pcd", *this->cloud_);
  }
}

template <typename PointType>
void Submap<PointType>::ToVtkFile(const std::string& filename) {
  if (!full_.load() || this->cloud_ == nullptr || this->cloud_->empty()) {
    return;
  }

  PRINT_INFO("Export submap vtk file.");
  pcl::PCLPointCloud2 cloud2;
  pcl::toPCLPointCloud2(*this->cloud_, cloud2);
  if (!filename.empty()) {
    pcl::io::saveVTKFile(filename, cloud2);
  } else {
    pcl::io::saveVTKFile("submap_" + std::to_string(id_.submap_index) + ".vtk",
                         cloud2);
  }
}

template <typename PointType>
void Submap<PointType>::ToInfoFile(const std::string& filename) {
  std::ofstream info_file(filename, std::ios::out | std::ios::binary);
  if (info_file.is_open()) {
    info_file.close();
  } else {
    PRINT_ERROR_FMT("Cannot open file: %s", filename.c_str());
  }
}

template <typename PointType>
void Submap<PointType>::ClearCloudInFrames() {
  CHECK(full_.load());
  for (auto& frame : frames_) {
    frame->ClearCloud();
  }
}

template <typename PointType>
void Submap<PointType>::AddConnectedSubmap(const SubmapId& id) {
  // do not need locker now
  // boost::upgrade_lock<ReadWriteMutex> locker(mutex_);
  // WriteMutexLocker write_locker(locker);
  connected_submaps_.push_back(id);
}

template <typename PointType>
std::vector<SubmapId>& Submap<PointType>::GetConnected() {
  return connected_submaps_;
}

template <typename PointType>
void Submap<PointType>::UpdateInnerFramePose() {
  boost::upgrade_lock<ReadWriteMutex> locker(mutex_);
  WriteMutexLocker write_locker(locker);
  for (auto& frame : frames_) {
    frame->SetGlobalPose(this->global_pose_ * frame->LocalPose());
  }
}

template <typename PointType>
std::string Submap<PointType>::SavedFileName() {
  if (options_.enable_disk_saving) {
    return save_filename_;
  }
  return "";
}

template <typename PointType>
void Submap<PointType>::SetSavedFileName(const std::string& filename) {
  save_filename_ = filename;
}

template <typename PointType>
Submap<PointType>::~Submap() {}

template class Submap<pcl::PointXYZI>;

}  // namespace static_map
