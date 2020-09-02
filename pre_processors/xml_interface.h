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

#ifndef PRE_PROCESSORS_XML_INTERFACE_H_
#define PRE_PROCESSORS_XML_INTERFACE_H_

#include <deque>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>

#include "common/macro_defines.h"

namespace static_map {
namespace pre_processers {

class XmlInterface {
 public:
  enum ParamType { kInt32Param, kFloatParam, kParamTypeCount };
  struct ParamItem {
    ParamType type;
    void* target;

    ParamItem() : type(kParamTypeCount), target(nullptr) {}
    ParamItem(ParamType t, void* p) : type(t), target(p) {}
  };

  XmlInterface() = default;

  virtual ~XmlInterface() {}

  PROHIBIT_COPY_AND_ASSIGN(XmlInterface);

  template <typename T>
  bool SetValue(const std::string& name, const T& value) {
    if (name_to_param_.count(name) == 0) {
      return false;
    }

    auto& param_item = name_to_param_[name];
    switch (param_item.type) {
      case kInt32Param:
        *(reinterpret_cast<int32_t*>(param_item.target)) =
            static_cast<int32_t>(value);
        break;
      case kFloatParam:
        *(reinterpret_cast<float*>(param_item.target)) =
            static_cast<float>(value);
        break;
      default:
        return false;
    }

    return true;
  }

  virtual void DisplayAllParams() {}

 protected:
  std::map<std::string, ParamItem> name_to_param_;
};

}  // namespace pre_processers
}  // namespace static_map

#define INIT_INNER_PARAM(type, name, target) \
  XmlInterface::name_to_param_.emplace(      \
      name, XmlInterface::ParamItem(type, reinterpret_cast<void*>(&target)));

#define INIT_FLOAT_PARAM(name, target) \
  INIT_INNER_PARAM(XmlInterface::ParamType::kFloatParam, name, target)

#define INIT_INT32_PARAM(name, target) \
  INIT_INNER_PARAM(XmlInterface::ParamType::kInt32Param, name, target)

#ifndef XML_INFO
#define XML_INFO std::cout << "  " << std::setw(50)
#endif

#ifndef PARAM_INFO
#define PARAM_INFO(name) XML_INFO << #name << " -> " << name << std::endl;
#endif

#endif  // PRE_PROCESSORS_XML_INTERFACE_H_
