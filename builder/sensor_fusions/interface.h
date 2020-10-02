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

#ifndef BUILDER_SENSOR_FUSIONS_INTERFACE_H_
#define BUILDER_SENSOR_FUSIONS_INTERFACE_H_

#include "glog/logging.h"

namespace static_map {

// forward declare to speed compiling
namespace sensors {
struct ImuMsg;
struct GpsEnuMsg;
struct OdomMsg;
};  // namespace sensors

namespace sensor_fusions {

class Interface {
 public:
  Interface() {}
  virtual ~Interface() {}

  virtual void AddImuData(const data::ImuMsg&) = 0;
  virtual void AddGpsData(const data::GpsEnuMsg&) = 0;
  virtual void AddOdomData(const data::OdomMsg&) = 0;
};

}  // namespace sensor_fusions
}  // namespace static_map

#endif  // BUILDER_SENSOR_FUSIONS_INTERFACE_H_
