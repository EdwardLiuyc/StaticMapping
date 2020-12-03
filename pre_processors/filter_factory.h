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

#include "pugixml/pugixml.hpp"
// filters
#include "pre_processors/filter_axis_range.h"
#include "pre_processors/filter_bounding_box.h"
#include "pre_processors/filter_ground_removal.h"
#include "pre_processors/filter_ground_removal2.h"
#include "pre_processors/filter_random_sample.h"
#include "pre_processors/filter_range.h"
#include "pre_processors/filter_range_image.h"
#include "pre_processors/filter_statistic_removal.h"
#include "pre_processors/filter_voxel_grid.h"

namespace static_map {
namespace pre_processers {
namespace filter {

class Factory : public Interface {
 public:
  Factory() : Interface() { RegisterSupportedFilters(); }
  ~Factory() = default;

  Factory(const Factory&) = delete;
  Factory& operator=(const Factory&) = delete;

  std::shared_ptr<Interface> CreateNewInstance() override {
    return std::make_shared<Factory>();
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
    pugi::xml_document doc;
    if (doc.load_string(text)) {
      InitFromXmlNode(doc.first_child());
    } else {
      LOG(FATAL) << "invalid xml text.";
    }
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

  void Filter(const data::InnerCloudType::Ptr& cloud) override {
    if (!cloud || !Interface::inner_cloud_) {
      LOG(WARNING) << "nullptr cloud, do nothing!" << std::endl;
      return;
    }

    this->FilterPrepare(cloud);
    if (filters_.empty()) {
      *cloud = *this->inner_cloud_;
      return;
    }

    // use all filters in order
    data::InnerCloudType::Ptr middle_poindcloud1(new data::InnerCloudType);
    data::InnerCloudType::Ptr middle_poindcloud2(new data::InnerCloudType);
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
  void RegisterSupportedFilters() {
    supported_filters_.emplace("RandomSampler",
                               std::make_shared<RandomSampler>());
    supported_filters_.emplace("RangeImage", std::make_shared<RangeImage>());
    supported_filters_.emplace("GroundRemoval",
                               std::make_shared<GroundRemoval>());
    supported_filters_.emplace("GroundRemoval2",
                               std::make_shared<GroundRemoval2>());
    supported_filters_.emplace("Range", std::make_shared<Range>());
    supported_filters_.emplace("StatisticRemoval",
                               std::make_shared<StatisticRemoval>());
    supported_filters_.emplace("VoxelGrid", std::make_shared<VoxelGrid>());
    supported_filters_.emplace("AxisRange", std::make_shared<AxisRange>());
    supported_filters_.emplace("BoundingBoxRemoval",
                               std::make_shared<BoundingBoxRemoval>());
  }

 private:
  std::vector<std::shared_ptr<Interface>> filters_;
  std::map<std::string, std::shared_ptr<Interface>> supported_filters_;
};

}  // namespace filter
}  // namespace pre_processers
}  // namespace static_map

#endif  // PRE_PROCESSORS_FILTER_FACTORY_H_
