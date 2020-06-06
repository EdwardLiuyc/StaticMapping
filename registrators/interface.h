
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

#include "common/macro_defines.h"
#include "common/pugixml.hpp"

namespace static_map {
namespace registrator {

enum Type {
  kNoType,
  kIcpPM,
  kLibicp,  // deprecated
  kNdtWithGicp,
  kLegoLoam,
  kNdt,
  kFastIcp,
  kTypeCount
};

struct InlierPointPairs {
  Eigen::Matrix<float, Eigen::Dynamic, 3> ref_points, read_points;
  size_t pairs_num;
};

enum class OptionItemDataType : uint8_t { kInt32, kFloat32 };

struct InnerOptionItem {
  OptionItemDataType data_type;
  void* data_ptr = nullptr;
};

struct MatcherOptions {
  Type type = static_map::registrator::Type::kIcpPM;
  bool enable_ndt = false;
  bool use_voxel_filter = true;
  double voxel_filter_resolution = 0.1;
  float accepted_min_score = 0.7;
  // for inner filters
  pugi::xml_node registrator_options;
  pugi::xml_node inner_filters_node;
};

template <typename PointType>
class Interface {
 public:
  typedef pcl::PointCloud<PointType> PointCloudSource;
  typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
  typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

  typedef pcl::PointCloud<PointType> PointCloudTarget;
  typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
  typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;

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

  void PrintOptions();

  // @todo(edward) change the function names ( naming rules )
  virtual void setInputSource(const PointCloudSourcePtr& cloud);
  virtual void setInputTarget(const PointCloudTargetPtr& cloud);
  virtual double getFitnessScore() { return final_score_; }
  virtual InlierPointPairs getInlierPointPairs() { return point_pairs_; }

  // need to be implemented by child class
  virtual bool align(const Eigen::Matrix4f& guess,
                     Eigen::Matrix4f& result) = 0;  // NOLINT

 protected:
  double final_score_;

  PointCloudSourcePtr source_cloud_ = nullptr;
  PointCloudTargetPtr target_cloud_ = nullptr;

  Type type_ = kNoType;
  InlierPointPairs point_pairs_;

  std::unordered_map<std::string, InnerOptionItem> inner_options_;
};

template <typename PointType>
void Interface<PointType>::setInputSource(
    const typename Interface<PointType>::PointCloudSourcePtr& cloud) {
  if (!cloud) {
    source_cloud_ = nullptr;
    return;
  }
  if (cloud->empty()) {
    PRINT_WARNING("cloud is empty.");
    return;
  }
  source_cloud_ = cloud;
}

template <typename PointType>
void Interface<PointType>::setInputTarget(
    const typename Interface<PointType>::PointCloudTargetPtr& cloud) {
  if (!cloud) {
    target_cloud_ = nullptr;
    return;
  }
  if (cloud->empty()) {
    PRINT_WARNING("cloud is empty.");
    return;
  }
  target_cloud_ = cloud;
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

      default:
        break;
    }
  }
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
  using typename Interface<PointType>::PointCloudTargetPtr;

#define REG_REGISTRATOR_INNER_OPTION(NAME, TYPE, VARIABLE) \
  this->inner_options_[NAME].data_type = TYPE;             \
  this->inner_options_[NAME].data_ptr = &VARIABLE;

#endif  // REGISTRATORS_INTERFACE_H_
