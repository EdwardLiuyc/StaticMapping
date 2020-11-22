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

#include <chrono>
#include <cstring>
#include <stack>

#include "common/macro_defines.h"
#include "common/simple_time.h"

namespace static_map {

static char filename[128] = {'\0'};
char* splited_file_name(const char* file) {
  memset(filename, 0, sizeof(filename));
  std::string str = file;
  int index = str.find_last_of('/');
  if (index != std::string::npos) {
    snprintf(filename, sizeof(filename), "%s", str.substr(index + 1).c_str());
  } else {
    snprintf(filename, sizeof(filename), "%s", str.c_str());
  }
  return filename;
}

std::stack<SimpleTime> debug_times;
// @todo make it thread-safe
void start_clock() { debug_times.push(SimpleTime::GetCurrentTime()); }
void end_clock(const char* filename, const char* func_name, const int line) {
  if (debug_times.empty()) {
    PRINT_ERROR_FMT("[ %s: %s: %d ] no clock.", splited_file_name(filename),
                    func_name, line);
    return;
  }
  SimpleTime current_time = debug_times.top();
  debug_times.pop();
  PRINT_COLOR_FMT(BOLD, "[ %s: %s: %d ] Cost : %lf s",
                  splited_file_name(filename), func_name, line,
                  (SimpleTime::GetCurrentTime() - current_time).ToSec());
}

// using c++ std
std::chrono::high_resolution_clock::time_point debug_start_time_std;
void start_clock_std() {
  debug_start_time_std = std::chrono::high_resolution_clock::now();
}

void end_clock_std() {
  std::chrono::high_resolution_clock::time_point end =
      std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::ratio<1, 1>> duration_s(
      end - debug_start_time_std);
  PRINT_DEBUG_FMT("Cost: %lf s", duration_s.count());
}

void end_clock_min_time_std(double min_time) {
  std::chrono::high_resolution_clock::time_point end =
      std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::ratio<1, 1>> duration_s(
      end - debug_start_time_std);
  double spent_time = duration_s.count();
  if (spent_time > min_time) PRINT_DEBUG_FMT("Cost: %lf s", spent_time);
}

}  // namespace static_map
