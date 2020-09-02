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

#ifndef PRE_PROCESSORS_FILTER_FACTORY_H_
#define PRE_PROCESSORS_FILTER_FACTORY_H_

#include <map>
#include <memory>
#include <string>
#include <vector>

// filters
#include "pre_processors/filter_axis_range.h"
#include "pre_processors/filter_ground_removal.h"
#include "pre_processors/filter_ground_removal2.h"
#include "pre_processors/filter_random_sample.h"
#include "pre_processors/filter_range.h"
#include "pre_processors/filter_range_image.h"
#include "pre_processors/filter_statistic_removal.h"
#include "pre_processors/filter_voxel_grid.h"
#include "pre_processors/filter_semantic.h"

namespace static_map {
namespace pre_processers {
namespace filter {

template <typename PointT>
class Factory : public Interface<PointT> {
 public:
  USE_POINTCLOUD;

  Factory() : Interface<PointT>() { RegisterSupportedFilters(); }
  ~Factory() = default;

  Factory(const Factory&) = delete;
  Factory& operator=(const Factory&) = delete;

  std::shared_ptr<Interface<PointT>> CreateNewInstance() override {
    return std::make_shared<Factory<PointT>>();
  }

  /* example
  <filters>
    <filter name="RandomSampler">
      <param type="1" name="sampling_rate"> 0.8 </param>
    </filter>
    ...
  </filters>
  */
  inline void InitFromXmlText(const char* text) {
    InitFromXmlNode(pugi::xml_node(text));
  }

  void InitFromXmlNode(const pugi::xml_node& filters_node) {
    for (auto filter_node = filters_node.child("filter"); filter_node;
         filter_node = filter_node.next_sibling("filter")) {
      const std::string filter_name = filter_node.attribute("name").as_string();
      auto it = supported_filters_.find(filter_name);
      if (it == supported_filters_.end()) {
        XML_INFO << filter_name << " not supported yet." << std::endl;
        continue;
      }

      std::string debug_msg = "Creating filter: " + filter_name;
      XML_INFO << debug_msg << std::endl;

      auto filter = it->second->CreateNewInstance();
      filter->InitFromXmlNode(filter_node);
      filters_.push_back(filter);
      filter->DisplayAllParams();
    }
  }

  void Filter(const PointCloudPtr& cloud) override {
    if (!cloud || !Interface<PointT>::inner_cloud_) {
      LOG(WARNING) << "nullptr cloud, do nothing!" << std::endl;
      return;
    }

    this->FilterPrepare(cloud);
    if (filters_.empty()) {
      *cloud = *this->inner_cloud_;
      return;
    }

    // use all filters in order
    PointCloudPtr middle_poindcloud1(new PointCloudType);
    PointCloudPtr middle_poindcloud2(new PointCloudType);
    *middle_poindcloud1 = *(this->inner_cloud_);
    for (auto filter : filters_) {
      filter->SetInputCloud(middle_poindcloud1);
      filter->Filter(middle_poindcloud2);
      // @todo copy but not directly =
      *middle_poindcloud1 = *middle_poindcloud2;
    }
    *cloud = *middle_poindcloud2;
  }

 protected:
  void RegisterSupportedFilters();

 private:
  std::vector<std::shared_ptr<Interface<PointT>>> filters_;
  std::map<std::string, std::shared_ptr<Interface<PointT>>> supported_filters_;
};

template <typename PointT>
void Factory<PointT>::RegisterSupportedFilters() {
  supported_filters_.emplace("RandomSampler",
                             std::make_shared<RandomSampler<PointT>>());
  supported_filters_.emplace("RangeImage",
                             std::make_shared<RangeImage<PointT>>());
  supported_filters_.emplace("GroundRemoval",
                             std::make_shared<GroundRemoval<PointT>>());
  supported_filters_.emplace("GroundRemoval2",
                             std::make_shared<GroundRemoval2<PointT>>());
  supported_filters_.emplace("Range", std::make_shared<Range<PointT>>());
  supported_filters_.emplace("StatisticRemoval",
                             std::make_shared<StatisticRemoval<PointT>>());
  supported_filters_.emplace("VoxelGrid",
                             std::make_shared<VoxelGrid<PointT>>());
  supported_filters_.emplace("AxisRange",
                             std::make_shared<AxisRange<PointT>>());
  /*supported_filters_.emplace("Semantic",
                             std::make_shared<SemanticFilter<PointT>>());*/


}

}  // namespace filter
}  // namespace pre_processers
}  // namespace static_map

#endif  // PRE_PROCESSORS_FILTER_FACTORY_H_
