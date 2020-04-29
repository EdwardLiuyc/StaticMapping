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

#ifndef ROS_NODE_PLAYABLE_BAG_H_
#define ROS_NODE_PLAYABLE_BAG_H_

#include <deque>
#include <functional>
#include <memory>
#include <queue>
#include <set>
#include <string>
#include <tuple>

// #include <cartographer_ros_msgs/BagfileProgress.h>
#include "ros/node_handle.h"

#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "tf2_ros/buffer.h"

namespace static_map_ros {

class PlayableBag {
 public:
  // Handles messages early, i.e. when they are about to enter the buffer.
  // Returns a boolean indicating whether the message should enter the buffer.
  using FilteringEarlyMessageHandler =
      std::function<bool(const rosbag::MessageInstance&)>;

  PlayableBag(const std::string& bag_filename, int bag_id, ros::Time start_time,
              ros::Time end_time, ros::Duration buffer_delay,
              FilteringEarlyMessageHandler filtering_early_message_handler);

  ros::Time PeekMessageTime() const;
  rosbag::MessageInstance GetNextMessage();
  bool IsMessageAvailable() const;
  std::tuple<ros::Time, ros::Time> GetBeginEndTime() const;

  int bag_id() const;
  std::set<std::string> topics() const { return topics_; }
  double duration_in_seconds() const { return duration_in_seconds_; }
  bool finished() const { return finished_; }
  std::string get_bag_filename() const { return bag_filename_; }

 private:
  void AdvanceOneMessage();
  void AdvanceUntilMessageAvailable();

  std::unique_ptr<rosbag::Bag> bag_;
  std::unique_ptr<rosbag::View> view_;
  rosbag::View::const_iterator view_iterator_;
  bool finished_;
  const int bag_id_;
  const std::string bag_filename_;
  const double duration_in_seconds_;
  int message_counter_;
  std::deque<rosbag::MessageInstance> buffered_messages_;
  const ::ros::Duration buffer_delay_;
  FilteringEarlyMessageHandler filtering_early_message_handler_;
  std::set<std::string> topics_;
};

}  // namespace static_map_ros

#endif  // ROS_NODE_PLAYABLE_BAG_H_
