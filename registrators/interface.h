
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

class Interface {
 public:
  using InnerCloudPtr = typename data::InnerPointCloudData::Ptr;

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

  virtual Type GetType() const { return type_; }

 protected:
  double final_score_;
  Type type_ = kNoType;
  InnerCloudPtr source_cloud_ = nullptr;
  InnerCloudPtr target_cloud_ = nullptr;
  std::unordered_map<std::string, InnerOptionItem> inner_options_;

  bool inner_compensation_ = false;
};

std::shared_ptr<Interface> CreateMatcher(const MatcherOptions& options,
                                         bool verbose = false);

}  // namespace registrator
}  // namespace static_map

#define USE_REGISTRATOR_CLOUDS using typename Interface::InnerCloudPtr;

#define REG_REGISTRATOR_INNER_OPTION(NAME, TYPE, VARIABLE) \
  this->inner_options_[NAME].data_type = TYPE;             \
  this->inner_options_[NAME].data_ptr = &VARIABLE;

#endif  // REGISTRATORS_INTERFACE_H_
