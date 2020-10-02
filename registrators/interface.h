
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

#ifndef REGISTRATORS_INTERFACE_H_
#define REGISTRATORS_INTERFACE_H_

#include <glog/logging.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <memory>
#include <string>
#include <unordered_map>

#include "builder/data/cloud_types.h"
#include "common/macro_defines.h"
#include "pugixml/pugixml.hpp"

namespace static_map {
namespace registrator {

enum Type {
  kNoType,
  kIcpPM,
  kLibicp,  // deprecated
  kNdtWithGicp,
  kLegoLoam,  // deprecated
  kNdt,
  kFastIcp,
  kTypeCount
};

enum class OptionItemDataType : uint8_t { kInt32, kFloat32, kBool };

struct InnerOptionItem {
  OptionItemDataType data_type;
  void* data_ptr = nullptr;
};

struct MatcherOptions {
  Type type = static_map::registrator::Type::kIcpPM;
  float accepted_min_score = 0.7;
  // for inner filters
  pugi::xml_node registrator_options_node;
  pugi::xml_node inner_filters_node;
};

template <typename PointType>
class Interface {
 public:
  using PointCloudSource = pcl::PointCloud<PointType>;
  using PointCloudSourcePtr = typename PointCloudSource::Ptr;
  using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;

  using PointCloudTarget = PointCloudSource;
  using PointCloudTargetPtr = typename PointCloudTarget::Ptr;
  using PointCloudTargetConstPtr = typename PointCloudTarget::ConstPtr;

  using InnerCloudPtr = typename data::InnerPointCloudData<PointType>::Ptr;

  Interface() = default;
  virtual ~Interface() {}

  PROHIBIT_COPY_AND_ASSIGN(Interface);

  // the xml node should be like this
  /*
  <registrator>
    <param name="max_range"> 75. </param>
    <param name="min_range"> 4.5 </param>
  </registrator>
  */
  void InitWithXml(const pugi::xml_node& node);

  /// @brief Some times we need a set of filters inside of the icp algorithm,
  /// apart from the pre-processor.
  void InitInnerFiltersWithXml(const pugi::xml_node& node);

  /// @brief Enable/Disable the motion compensation.
  void EnableInnerCompensation();
  void DisableInnerCompensation();

  /// @brief Using the configs from xml to init inner functions.
  virtual void InitWithOptions() {}
  /// @brief Print configs' value.
  void PrintOptions();

  virtual void SetInputSource(InnerCloudPtr source_cloud);
  virtual void SetInputTarget(InnerCloudPtr target_cloud);
  virtual double GetFitnessScore() { return final_score_; }

  // need to be implemented by child class
  virtual bool Align(const Eigen::Matrix4d& guess,
                     Eigen::Matrix4d& result) = 0;  // NOLINT

 protected:
  double final_score_;
  Type type_ = kNoType;
  InnerCloudPtr source_cloud_ = nullptr;
  InnerCloudPtr target_cloud_ = nullptr;
  std::unordered_map<std::string, InnerOptionItem> inner_options_;

  bool inner_compensation_ = false;
};

template <typename PointType>
void Interface<PointType>::EnableInnerCompensation() {
  inner_compensation_ = true;
}

template <typename PointType>
void Interface<PointType>::DisableInnerCompensation() {
  inner_compensation_ = false;
}

template <typename PointType>
void Interface<PointType>::SetInputSource(
    typename Interface<PointType>::InnerCloudPtr source_cloud) {
  if (!source_cloud) {
    source_cloud_ = nullptr;
    return;
  }
  if (source_cloud->Empty()) {
    PRINT_WARNING("cloud is empty.");
    return;
  }
  source_cloud_ = source_cloud;
}

template <typename PointType>
void Interface<PointType>::SetInputTarget(
    typename Interface<PointType>::InnerCloudPtr target_cloud) {
  if (!target_cloud) {
    target_cloud_ = nullptr;
    return;
  }
  if (target_cloud->Empty()) {
    PRINT_WARNING("cloud is empty.");
    return;
  }
  target_cloud_ = target_cloud;
}

template <typename PointType>
void Interface<PointType>::InitWithXml(const pugi::xml_node& node) {
  for (auto param_node = node.child("param"); param_node;
       param_node = param_node.next_sibling("param")) {
    const std::string param_name = param_node.attribute("name").as_string();
    CHECK_GT(inner_options_.count(param_name), 0)
        << "Init an unknown option of this matcher!";
    // CHECK(inner_options_.at(param_name).data_ptr);

    switch (inner_options_.at(param_name).data_type) {
      case OptionItemDataType::kInt32:
        *reinterpret_cast<int32_t*>(inner_options_.at(param_name).data_ptr) =
            param_node.text().as_int();
        break;

      case OptionItemDataType::kFloat32:
        *reinterpret_cast<float*>(inner_options_.at(param_name).data_ptr) =
            param_node.text().as_float();
        break;

      case OptionItemDataType::kBool:
        *reinterpret_cast<bool*>(inner_options_.at(param_name).data_ptr) =
            param_node.text().as_bool();
        break;

      default:
        break;
    }
  }
}

template <typename PointType>
void Interface<PointType>::InitInnerFiltersWithXml(const pugi::xml_node& node) {
  // TODO(edward) inner filters
  if (node.empty()) {
    PRINT_INFO("Init filters with default parameters.");
    return;
  }
  // CHECK_EQ(std::string(node.name()), "inner_filters");
  // for (auto filter_node = node.child("filter"); filter_node;
  //      filter_node = filter_node.next_sibling("filter")) {
  //   std::string filter_name = filter_node.attribute("name").as_string();
  //   if (filter_name == "GroundRemoval2") {
  //     ground_removal_filter_.InitFromXmlNode(filter_node);
  //   } else if (filter_name == "RangeImage") {
  //     range_image_filter_.InitFromXmlNode(filter_node);
  //   }
  // }

  // ground_removal_filter_.DisplayAllParams();
  // range_image_filter_.DisplayAllParams();
}

template <typename PointType>
void Interface<PointType>::PrintOptions() {
  for (const auto& name_to_inner_option_item : inner_options_) {
    std::cout << std::setw(25) << name_to_inner_option_item.first << " -> ";
    const auto& option_item = name_to_inner_option_item.second;
    switch (option_item.data_type) {
      case OptionItemDataType::kInt32:
        std::cout << *reinterpret_cast<int32_t*>(option_item.data_ptr);
        break;

      case OptionItemDataType::kFloat32:
        std::cout << std::setprecision(6)
                  << *reinterpret_cast<float*>(option_item.data_ptr);
        break;

      case OptionItemDataType::kBool:
        std::cout << std::boolalpha
                  << *reinterpret_cast<bool*>(option_item.data_ptr);
        break;

      default:
        break;
    }
    std::cout << std::endl;
  }
}

template <typename PointType>
std::shared_ptr<Interface<PointType>> CreateMatcher(
    const MatcherOptions& options, bool verbose = false);

}  // namespace registrator
}  // namespace static_map

#define USE_REGISTRATOR_CLOUDS                              \
  using typename Interface<PointType>::PointCloudSource;    \
  using typename Interface<PointType>::PointCloudTarget;    \
  using typename Interface<PointType>::PointCloudSourcePtr; \
  using typename Interface<PointType>::PointCloudTargetPtr; \
  using typename Interface<PointType>::InnerCloudPtr;

#define REG_REGISTRATOR_INNER_OPTION(NAME, TYPE, VARIABLE) \
  this->inner_options_[NAME].data_type = TYPE;             \
  this->inner_options_[NAME].data_ptr = &VARIABLE;

#endif  // REGISTRATORS_INTERFACE_H_
