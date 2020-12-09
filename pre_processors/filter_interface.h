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
#include <unordered_map>

#include "pre_processors/processor_interface.h"

#include "pugixml/pugixml.hpp"

namespace static_map {
namespace pre_processers {
namespace filter {

class Interface : public ProcesserInterface {
 public:
  Interface() : ProcesserInterface() {}
  virtual ~Interface() {}

  PROHIBIT_COPY_AND_ASSIGN(Interface);

  // @brief Use xml node to init inner config parameters.
  bool InitFromXmlNode(const pugi::xml_node& node);
  // @brief Use xml test to construct xml node, then init inner configs.
  bool InitFromXmlText(const char* xml_text);
  // @brief return whether all configs are valid.
  virtual bool ConfigsValid() const { return true; }
  // TODO(edward) should be a static function.
  virtual std::shared_ptr<Interface> CreateNewInstance() = 0;
  // @brief Filter and output the inlier points to cloud. should be implemented
  // by child classes.
  virtual void Filter(const data::InnerCloudType::Ptr& cloud) = 0;
  // @brief return current filter's class name.
  virtual std::string GetName() const;

 protected:
  // @brief Will be called in the beginning of Filter(cloud) to ensure the cloud
  // ready to go.
  virtual void FilterPrepare(const data::InnerCloudType::Ptr& cloud);
};

extern const std::unordered_map<std::string, std::string> kFilterNameMap;

}  // namespace filter
}  // namespace pre_processers
}  // namespace static_map

#endif  // PRE_PROCESSORS_FILTER_INTERFACE_H_
