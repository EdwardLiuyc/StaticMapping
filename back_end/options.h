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

#ifndef BACK_END_OPTIONS_H_
#define BACK_END_OPTIONS_H_

#include "back_end/isam_optimizer.h"
#include "back_end/loop_detector.h"
#include "builder/submap.h"
#include "registrators/interface.h"

namespace static_map {
namespace back_end {

struct Options {
  struct SubmapMatcherOptions {
    static_map::registrator::Type type = static_map::registrator::Type::kIcpPM;
    bool use_voxel_filter = true;
    bool enable_ndt = false;
    float voxel_filter_resolution = 0.1;
    float accepted_min_score = 0.8;
  } submap_matcher_options;

  SubmapOptions submap_options;
  IsamOptimizerOptions isam_optimizer_options;
  LoopDetectorSettings loop_detector_setting;
};

}  // namespace back_end
}  // namespace static_map

#endif  // BACK_END_OPTIONS_H_
