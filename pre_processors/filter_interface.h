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

#ifndef PRE_PROCESSORS_FILTER_INTERFACE_H_
#define PRE_PROCESSORS_FILTER_INTERFACE_H_

#include <memory>
#include <string>

#include "pre_processors/processor_interface.h"

#include "pugixml/pugixml.hpp"

namespace static_map {
namespace pre_processers {
namespace filter {

template <typename PointT>
class Interface : public ProcesserInterface<PointT> {
 public:
  using PointCloudPtr = typename ProcesserInterface<PointT>::PointCloudPtr;

  Interface() : ProcesserInterface<PointT>() {}
  ~Interface() {}
  Interface(const Interface&) = delete;
  Interface& operator=(const Interface&) = delete;

  void InitFromXmlNode(const pugi::xml_node& node) {
    CHECK_EQ(std::string(node.name()), "filter");
    bool all_right = true;
    for (auto param_node = node.child("param"); param_node;
         param_node = param_node.next_sibling("param")) {
      XmlInterface::ParamType param_type = XmlInterface::kParamTypeCount;
      if (param_node.attribute("type")) {
        param_type =
            (XmlInterface::ParamType)param_node.attribute("type").as_int();
      }
      std::string param_name = param_node.attribute("name").as_string();
      switch (param_type) {
        case XmlInterface::kInt32Param:
          all_right = this->SetValue(param_name, param_node.text().as_int());
          break;
        case XmlInterface::kFloatParam:
          all_right = this->SetValue(param_name, param_node.text().as_float());
          break;
        default:
          all_right = false;
          break;
      }
      CHECK(all_right);
    }
  }

  void InitFromXmlText(const char* xml_text) {
    pugi::xml_document doc;
    if (doc.load_string(xml_text)) {
      InitFromXmlNode(doc.first_child());
    } else {
      LOG(FATAL) << "invalid xml text.";
    }
  }

  virtual std::shared_ptr<Interface<PointT>> CreateNewInstance() = 0;
  virtual void Filter(const PointCloudPtr& cloud) = 0;

  virtual void FilterPrepare(const PointCloudPtr& cloud) {
    auto& input = Interface<PointT>::inner_cloud_;
    cloud->clear();
    cloud->header = input->header;
    this->inliers_.clear();
    this->outliers_.clear();
  }
};

}  // namespace filter
}  // namespace pre_processers
}  // namespace static_map

#define USE_POINTCLOUD                                             \
  using PointCloudType = pcl::PointCloud<PointT>;                  \
  using PointCloudPtr = typename Interface<PointT>::PointCloudPtr; \
  using PointCloudConstPtr = typename PointCloudType::ConstPtr;

#endif  // PRE_PROCESSORS_FILTER_INTERFACE_H_
