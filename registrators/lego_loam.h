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

#pragma once

#include <string>

#include "pre_processors/filter_ground_removal2.h"
#include "pre_processors/filter_range_image.h"
#include "registrators/interface.h"

namespace static_map {
namespace registrator {

constexpr uint32_t kGroundLabel = 999999;

template <typename PointT>
class LegoLoam : public Interface<PointT> {
 public:
  using PointCloudType = pcl::PointCloud<PointT>;
  using PointCloudPtr = typename PointCloudType::Ptr;
  using PointCloudConstPtr = typename PointCloudType::ConstPtr;

  using PointCloudSourcePtr = PointCloudPtr;
  using PointCloudTargetPtr = PointCloudPtr;

  using LabeledPointType = pcl::PointXYZL;
  using LabeledPointCloudPtr = pcl::PointCloud<LabeledPointType>::Ptr;

  LegoLoam() : Interface<PointT>(), source_data_(), target_data_() {
    this->type_ = kLegoLoam;
  }
  ~LegoLoam() {}
  LegoLoam(const LegoLoam &) = delete;
  LegoLoam &operator=(const LegoLoam &) = delete;

  inline void InitialiseFiltersFromXmlNode(const pugi::xml_node &node) {
    if (node.empty()) {
      PRINT_INFO("Init filters with default parameters.");
      return;
    }
    CHECK_EQ(std::string(node.name()), "inner_filters");
    for (auto filter_node = node.child("filter"); filter_node;
         filter_node = filter_node.next_sibling("filter")) {
      std::string filter_name = filter_node.attribute("name").as_string();
      if (filter_name == "GroundRemoval2") {
        ground_removal_filter_.InitFromXmlNode(filter_node);
      } else if (filter_name == "RangeImage") {
        range_image_filter_.InitFromXmlNode(filter_node);
      }
    }

    ground_removal_filter_.DisplayAllParams();
    range_image_filter_.DisplayAllParams();
  }

  inline void setInputSource(const PointCloudSourcePtr &cloud) override {
    if (cloud) {
      // perhaps, the cloud has been segmented
      if (cloud == source_data_.raw_cloud) {
        return;
      } else if (cloud == target_data_.raw_cloud) {
        source_data_.raw_cloud = target_data_.raw_cloud;
        *source_data_.ground_removed_cloud = *target_data_.ground_removed_cloud;
        *source_data_.segmented_cloud = *target_data_.segmented_cloud;
        return;
      }
    }
    UpdateData(source_data_, cloud);
  }

  inline void setInputTarget(const PointCloudTargetPtr &cloud) override {
    if (cloud) {
      // perhaps, the cloud has been segmented
      if (cloud == target_data_.raw_cloud) {
        return;
      } else if (cloud == source_data_.raw_cloud) {
        target_data_.raw_cloud = source_data_.raw_cloud;
        *target_data_.ground_removed_cloud = *source_data_.ground_removed_cloud;
        *target_data_.segmented_cloud = *source_data_.segmented_cloud;
        return;
      }
    }

    // PRINT_DEBUG("new data.");
    UpdateData(target_data_, cloud);
  }

  inline bool align(const Eigen::Matrix4f &guess,
                    Eigen::Matrix4f &result) override {
    return true;
  }

 protected:
  struct CloudData {
    PointCloudPtr raw_cloud;
    PointCloudPtr ground_removed_cloud;
    LabeledPointCloudPtr segmented_cloud;

    CloudData()
        : raw_cloud(nullptr),
          ground_removed_cloud(new PointCloudType),
          segmented_cloud(new pcl::PointCloud<LabeledPointType>) {}
  };

  inline void UpdateData(CloudData &data, const PointCloudSourcePtr &cloud) {
    data.raw_cloud = cloud;
    if (!cloud || cloud->empty()) {
      PRINT_ERROR("Empty cloud.");
      return;
    }
    // clear the former data
    data.ground_removed_cloud->clear();
    data.segmented_cloud->clear();

    ground_removal_filter_.SetInputCloud(cloud);
    ground_removal_filter_.Filter(data.ground_removed_cloud);
    range_image_filter_.SetInputCloud(cloud);
    range_image_filter_.DepthCluster(data.segmented_cloud);
    for (auto i : ground_removal_filter_.Outliers()) {
      pcl::PointXYZL ground_point;
      ground_point.x = cloud->points[i].x;
      ground_point.y = cloud->points[i].y;
      ground_point.z = cloud->points[i].z;
      ground_point.label = kGroundLabel;
      source_data_.segmented_cloud->push_back(ground_point);
    }
  }

 private:
  pre_processers::filter::GroundRemoval2<PointT> ground_removal_filter_;
  pre_processers::filter::RangeImage<PointT> range_image_filter_;

  CloudData source_data_;
  CloudData target_data_;

  // todo cache some data for future use, avoid repeat calculating
};

}  // namespace registrator
}  // namespace static_map
