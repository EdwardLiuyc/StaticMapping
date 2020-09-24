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

#ifndef BUILDER_SUBMAP_OPTIONS_H_
#define BUILDER_SUBMAP_OPTIONS_H_

#include <string>

namespace static_map {

struct SubmapOptions {
  bool enable_inner_mrvm = false;
  bool enable_voxel_filter = false;
  bool enable_random_sampleing = false;
  bool enable_check = true;
  float random_sampling_rate = 0.5;
  float voxel_size = 0.1f;
  int32_t frame_count = 5;

  // disk saving function
  bool enable_disk_saving = false;
  int32_t disk_saving_delay = 10;  // seconds
  std::string saving_name_prefix = "submap_";
};

}  // namespace static_map

#endif  // BUILDER_SUBMAP_OPTIONS_H_
