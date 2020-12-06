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

class Interface : public ProcesserInterface {
 public:
  Interface() : ProcesserInterface() {}
  virtual ~Interface() {}

  Interface(const Interface&) = delete;
  Interface& operator=(const Interface&) = delete;

  void InitFromXmlNode(const pugi::xml_node& node);

  void InitFromXmlText(const char* xml_text);

  virtual std::shared_ptr<Interface> CreateNewInstance() = 0;
  virtual void Filter(const data::InnerCloudType::Ptr& cloud) = 0;

  virtual void FilterPrepare(const data::InnerCloudType::Ptr& cloud);
};

}  // namespace filter
}  // namespace pre_processers
}  // namespace static_map

#endif  // PRE_PROCESSORS_FILTER_INTERFACE_H_
