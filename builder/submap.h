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

#ifndef BUILDER_SUBMAP_H_
#define BUILDER_SUBMAP_H_

// stl
#include <atomic>
#include <memory>
#include <ostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
// local
#include "builder/frame.h"
#include "builder/submap_options.h"
#include "common/mutex.h"

#include <boost/thread/pthread/shared_mutex.hpp>

namespace static_map {

struct SubmapId {
  int32_t trajectory_index;
  int32_t submap_index;

  bool operator==(const SubmapId& other) const;
  bool operator!=(const SubmapId& other) const;
  bool operator<(const SubmapId& other) const;
  std::string DebugString() const;
};

template <typename PointType>
class Submap : public FrameBase<PointType> {
 public:
  using PointCloudType = pcl::PointCloud<PointType>;
  using PointCloudPtr = typename PointCloudType::Ptr;
  using PointCloudConstPtr = typename PointCloudType::ConstPtr;
  using InnerCloudPtr = typename data::InnerPointCloudData<PointType>::Ptr;

  /// read write mutex
  /// when there is no writing in progress, several thread can access (read) the
  /// memory at the same time
  using ReadWriteMutex = boost::shared_mutex;
  using ReadMutexLocker = boost::shared_lock<ReadWriteMutex>;
  using WriteMutexLocker = boost::upgrade_to_unique_lock<ReadWriteMutex>;

  explicit Submap(const SubmapOptions& options);
  ~Submap();

  PROHIBIT_COPY_AND_ASSIGN(Submap);

  /// @brief Set the submap ID include trajectory index and submap index
  void SetId(const SubmapId& id);
  /// @brief Get the submap ID include trajectory index and submap index
  SubmapId GetId() { return id_; }
  /// @brief save the inner cloud into a pcd file
  void ToPcdFile(const std::string& filename) override;
  /// @brief save the inner cloud into a vtk file
  void ToVtkFile(const std::string& filename);
  /// @brief save all information including cloud data into a give file
  void ToInfoFile(const std::string& filename);
  /// @brief insert single cloud frame into the submap
  void InsertFrame(const std::shared_ptr<Frame<PointType>>& frame);
  /// @brief clean the cloud data in frames (for saving RAM) only if the
  /// submap cloud data is stable
  void ClearCloudInFrames();
  /// @brief if the submap's global pose has been changed,
  /// you should call this func to update the inner frames's global pose
  void UpdateInnerFramePose();
  /// @brief set the matrix to next and set flag to true as well
  void SetMatchedTransformedToNext(const Eigen::Matrix4d& t);
  /// @brief when the submap is full, you can insert no more frames into it
  /// the full_ flag is an atomic variable so use .load()
  inline bool Full() { return full_.load(); }
  inline bool GotMatchedToNext() {
    return got_matched_transform_to_next_.load();
  }
  /// @brief get all frames inserted into the submap
  std::vector<std::shared_ptr<Frame<PointType>>>& GetFrames();
  /// @brief get single frame in submap according to its id
  std::shared_ptr<Frame<PointType>> GetFrame(const FrameId& frame_id);
  /// @brief we do not just return the cloud
  /// @notice we update the active status and inactive time in this function
  InnerCloudPtr Cloud() override;
  /// @brief we manage all submaps' memory in a single thread
  /// and this managing thread tells each submap how long has it been waited
  bool UpdateInactiveTime(const int update_time_in_sec);
  /// @brief connect to another submap with this id
  /// @todo one submap should be connected directly to another submap
  /// but not a SubmapId (a submap's id can be changed, but it self will stay
  /// from the beginning to the end of process)
  void AddConnectedSubmap(const SubmapId& id);
  /// @brief get ids of all submaps connected to this one
  std::vector<SubmapId>& GetConnected();
  /// @brief set the pcd filename without path
  void SetSavedFileName(const std::string& filename);
  /// @brief set the path without filename
  void SetSavePath(const std::string& path);
  std::string SavedFileName();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  double match_score_to_previous_submap_ = 0.;

 private:
  ReadWriteMutex mutex_;
  std::vector<std::shared_ptr<Frame<PointType>>> frames_;

  SubmapOptions options_;
  SubmapId id_;
  std::string save_filename_;
  std::string save_path_;
  std::atomic<bool> full_;
  std::atomic<bool> is_cloud_in_memory_;
  std::atomic<bool> got_matched_transform_to_next_;
  std::atomic<int> cloud_inactive_time_;

  std::vector<SubmapId> connected_submaps_;
};

}  // namespace static_map

#endif  // BUILDER_SUBMAP_H_
