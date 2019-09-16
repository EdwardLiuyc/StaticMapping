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

#include "ros_node/urdf_reader.h"

#include <string>
#include <vector>

#include "glog/logging.h"
#include "urdf/model.h"

namespace static_map_ros {

std::vector<geometry_msgs::TransformStamped> ReadStaticTransformsFromUrdf(
    const std::string& urdf_filename, tf2_ros::Buffer* const tf_buffer) {
  urdf::Model model;
  CHECK(model.initFile(urdf_filename));
#if URDFDOM_HEADERS_HAS_SHARED_PTR_DEFS
  std::vector<urdf::LinkSharedPtr> links;
#else
  std::vector<boost::shared_ptr<urdf::Link> > links;
#endif
  model.getLinks(links);
  std::vector<geometry_msgs::TransformStamped> transforms;
  for (const auto& link : links) {
    if (!link->getParent() || link->parent_joint->type != urdf::Joint::FIXED) {
      continue;
    }

    const urdf::Pose& pose =
        link->parent_joint->parent_to_joint_origin_transform;
    geometry_msgs::TransformStamped transform;
    transform.transform.rotation.w = pose.rotation.w;
    transform.transform.rotation.x = pose.rotation.x;
    transform.transform.rotation.y = pose.rotation.y;
    transform.transform.rotation.z = pose.rotation.z;

    transform.transform.translation.x = pose.position.x;
    transform.transform.translation.y = pose.position.y;
    transform.transform.translation.z = pose.position.z;

    transform.child_frame_id = link->name;
    transform.header.frame_id = link->getParent()->name;
    tf_buffer->setTransform(transform, "urdf", true /* is_static */);
    transforms.push_back(transform);
  }
  return transforms;
}

}  // namespace static_map_ros
