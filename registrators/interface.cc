
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

#include "registrators/interface.h"

#include "registrators/icp_fast.h"
#include "registrators/icp_pointmatcher.h"
#include "registrators/ndt.h"
#include "registrators/ndt_gicp.h"

namespace static_map {
namespace registrator {

void Interface::EnableInnerCompensation() { inner_compensation_ = true; }

void Interface::DisableInnerCompensation() { inner_compensation_ = false; }

void Interface::SetInputSource(Interface::InnerCloudPtr source_cloud) {
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

void Interface::SetInputTarget(Interface::InnerCloudPtr target_cloud) {
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

void Interface::InitWithXml(const pugi::xml_node& node) {
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

void Interface::InitInnerFiltersWithXml(const pugi::xml_node& node) {
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

void Interface::PrintOptions() {
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

std::shared_ptr<Interface> CreateMatcher(const MatcherOptions& options,
                                         bool verbose) {
  std::shared_ptr<Interface> matcher;
  switch (options.type) {
    case registrator::kIcpPM:
      matcher.reset(new IcpUsingPointMatcher);
      break;
    case registrator::kNdtWithGicp:
      matcher.reset(new NdtWithGicp);
      break;
    case registrator::kNdt:
      matcher.reset(new Ndt);
      break;
    case registrator::kFastIcp:
      matcher.reset(new registrator::IcpFast());
      break;
    case registrator::kLibicp:
    case registrator::kLegoLoam:
      LOG(FATAL) << "The registrator using libicp & lego-loam is deprecated. "
                    "please choose another type";
      return nullptr;
    default:
      PRINT_ERROR("Wrong type");
      return nullptr;
  }

  if (!options.registrator_options_node.empty()) {
    matcher->InitWithXml(options.registrator_options_node);
  }
  if (verbose) {
    matcher->PrintOptions();
  }
  matcher->InitWithOptions();
  return matcher;
}

}  // namespace registrator
}  // namespace static_map
