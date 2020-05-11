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

#ifndef COMMON_PERFORMANCE_PROCESS_INFO_H_
#define COMMON_PERFORMANCE_PROCESS_INFO_H_

#include <pwd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <queue>
#include <string>
#include <thread>

namespace static_map {
namespace common {
namespace performance {

struct ProcStatus;

class ProcessProfiler {
 public:
  ProcessProfiler();

  ~ProcessProfiler();

 private:
  const char* GetItems(const char* buffer, unsigned int item);

  uint64_t GetCpuTotalOccpy();

  uint64_t GetCpuProcOccpy(unsigned int pid);

  float GetProcCpu(unsigned int pid);

  bool GetProcStatus(ProcStatus* status);

  void Run();

  pid_t pid_;
  std::thread profiling_thread_;
  bool run_ = true;
};

}  // namespace performance
}  // namespace common
}  // namespace static_map

#endif  // COMMON_PERFORMANCE_PROCESS_INFO_H_
