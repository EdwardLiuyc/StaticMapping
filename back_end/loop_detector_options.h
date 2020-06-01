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

#ifndef BACK_END_LOOP_DETECTOR_OPTIONS_H_
#define BACK_END_LOOP_DETECTOR_OPTIONS_H_

namespace static_map {
namespace back_end {

struct LoopDetectorSettings {
  bool use_gps = false;
  bool use_descriptor = false;
  bool output_matched_cloud = false;
  int loop_ignore_threshold = 15;
  int trying_detect_loop_count = 1;
  int nearest_history_pos_num = 4;
  float max_close_loop_distance = 25.f;
  float max_close_loop_z_distance = 1.f;
  float m2dp_match_score = 0.99f;
  float accept_scan_match_score = 0.75f;
};

}  // namespace back_end
}  // namespace static_map

#endif  // BACK_END_LOOP_DETECTOR_OPTIONS_H_
