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

#include <utility>

#include "common/make_unique.h"
#include "glog/logging.h"
#include "ros_node/playable_bag.h"
#include "tf2_msgs/TFMessage.h"

namespace static_map_ros {

PlayableBag::PlayableBag(
    const std::string& bag_filename, const int bag_id,
    const ros::Time start_time, const ros::Time end_time,
    const ros::Duration buffer_delay,
    FilteringEarlyMessageHandler filtering_early_message_handler)
    : bag_(static_map::common::make_unique<rosbag::Bag>(bag_filename,
                                                        rosbag::bagmode::Read)),
      view_(static_map::common::make_unique<rosbag::View>(*bag_, start_time,
                                                          end_time)),
      view_iterator_(view_->begin()),
      finished_(false),
      bag_id_(bag_id),
      bag_filename_(bag_filename),
      duration_in_seconds_(
          (view_->getEndTime() - view_->getBeginTime()).toSec()),
      message_counter_(0),
      buffer_delay_(buffer_delay),
      filtering_early_message_handler_(
          std::move(filtering_early_message_handler)) {
  AdvanceUntilMessageAvailable();
  for (const auto* connection_info : view_->getConnections()) {
    topics_.insert(connection_info->topic);
  }
}

ros::Time PlayableBag::PeekMessageTime() const {
  CHECK(IsMessageAvailable());
  return buffered_messages_.front().getTime();
}

std::tuple<ros::Time, ros::Time> PlayableBag::GetBeginEndTime() const {
  return std::make_tuple(view_->getBeginTime(), view_->getEndTime());
}

rosbag::MessageInstance PlayableBag::GetNextMessage() {
  CHECK(IsMessageAvailable());
  const rosbag::MessageInstance msg = buffered_messages_.front();
  buffered_messages_.pop_front();
  AdvanceUntilMessageAvailable();
  double processed_seconds = (msg.getTime() - view_->getBeginTime()).toSec();
  if ((message_counter_ % 10000) == 0) {
    LOG(INFO) << "Processed " << processed_seconds << " of "
              << duration_in_seconds_ << " seconds of bag " << bag_filename_;
  }

  return msg;
}

bool PlayableBag::IsMessageAvailable() const {
  return !buffered_messages_.empty() &&
         (buffered_messages_.front().getTime() <
          buffered_messages_.back().getTime() - buffer_delay_);
}

int PlayableBag::bag_id() const { return bag_id_; }

void PlayableBag::AdvanceOneMessage() {
  CHECK(!finished_);
  if (view_iterator_ == view_->end()) {
    finished_ = true;
    return;
  }
  rosbag::MessageInstance& msg = *view_iterator_;
  if (!filtering_early_message_handler_ ||
      filtering_early_message_handler_(msg)) {
    buffered_messages_.push_back(msg);
  }
  ++view_iterator_;
  ++message_counter_;
}

void PlayableBag::AdvanceUntilMessageAvailable() {
  if (finished_) {
    return;
  }
  do {
    AdvanceOneMessage();
  } while (!finished_ && !IsMessageAvailable());
}

}  // namespace static_map_ros
