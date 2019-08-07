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

#include <map>
#include <string>

namespace static_map {
namespace pre_processers {

constexpr uint32_t kParamCount = 20;

class XmlInterface {
 public:
  enum ParamType { kInt32Param, kFloatParam, kParamTypeCount };
  struct ParamIndex {
    ParamType type;
    uint32_t index;

    ParamIndex() : type(kParamTypeCount), index(0) {}
    ParamIndex(ParamType t, uint32_t i) : type(t), index(i) {}
  };

  XmlInterface() {
    for (uint32_t i = 0; i < kParamCount; ++i) {
      int32_params_[i] = NULL;
      float_params_[i] = NULL;
    }
  }
  ~XmlInterface() {}
  XmlInterface(const XmlInterface&) = delete;
  XmlInterface& operator=(const XmlInterface&) = delete;

  template <typename T>
  bool SetValue(const std::string& name, const T& value) {
    auto it = name_to_index_.find(name);
    if (it == name_to_index_.end()) {
      return false;
    }
    auto& param_index = name_to_index_[name];
    SetParamValue(param_index.type, param_index.index, value);
    return true;
  }

  template <typename T>
  bool SetParamValue(ParamType param_type, const uint32_t& index,
                     const T& value) {
    if (index >= kParamCount) {
      return false;
    }
    switch (param_type) {
      case kInt32Param:
        if (!int32_params_[index]) {
          return false;
        }
        *int32_params_[index] = (int32_t)value;
        break;
      case kFloatParam:
        if (!float_params_[index]) {
          return false;
        }
        *float_params_[index] = static_cast<float>(value);
        break;
      default:
        return false;
    }
    return true;
  }

  bool RemapParam(ParamType param_type, const uint32_t& index,
                  void* target_memory) {
    if (index >= kParamCount && target_memory == NULL) {
      return false;
    }
    switch (param_type) {
      case kInt32Param:
        if (int32_params_[index] != NULL) {
          return false;
        }
        int32_params_[index] = reinterpret_cast<int32_t*>(target_memory);
        break;
      case kFloatParam:
        if (float_params_[index] != NULL) {
          return false;
        }
        float_params_[index] = reinterpret_cast<float*>(target_memory);
        break;
      default:
        return false;
    }
    return true;
  }

  virtual void DisplayAllParams() {}

 protected:
  std::map<std::string, ParamIndex> name_to_index_;

 private:
  int32_t* int32_params_[kParamCount];
  float* float_params_[kParamCount];
};

}  // namespace pre_processers
}  // namespace static_map

#define INIT_INNER_PARAM(type, index, name, target)                  \
  CHECK(XmlInterface::RemapParam(type, index,                        \
                                 reinterpret_cast<void*>(&target))); \
  XmlInterface::name_to_index_.emplace(name,                         \
                                       XmlInterface::ParamIndex(type, index));

#ifndef XML_INFO
#define XML_INFO std::cout << "  " << std::setw(50)
#endif

#ifndef PARAM_INFO
#define PARAM_INFO(name) XML_INFO << #name << " -> " << name << std::endl;
#endif

#endif  // PRE_PROCESSORS_XML_INTERFACE_H_
