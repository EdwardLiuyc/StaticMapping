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

#include "common/performance/process_info.h"

namespace static_map {
namespace common {
namespace performance {

struct TotalCpuOccpy {
  uint64_t user;
  uint64_t nice;
  uint64_t system;
  uint64_t idle;
};

struct ProcCpuOccpy {
  unsigned int pid;
  uint64_t utime;   // user time
  uint64_t stime;   // kernel time
  uint64_t cutime;  // all user time
  uint64_t cstime;  // all dead time
};

struct ProcStatus {
  char name[256];
  float cpu_occpy;
  int32_t vm_size;
  int32_t mem_size;
  int32_t threads_num;
};

ProcessProfiler::ProcessProfiler() {
  pid_ = getpid();
  profiling_thread_ = std::thread(&ProcessProfiler::Run, this);
}

ProcessProfiler::~ProcessProfiler() {
  run_ = false;
  if (profiling_thread_.joinable()) {
    profiling_thread_.join();
  }
}

const char* ProcessProfiler::GetItems(const char* buffer, unsigned int item) {
  const char* p = buffer;
  int len = strlen(buffer);
  int count = 0;

  for (int i = 0; i < len; i++) {
    if (' ' == *p) {
      count++;
      if (count == item - 1) {
        p++;
        break;
      }
    }
    p++;
  }
  return p;
}

uint64_t ProcessProfiler::GetCpuTotalOccpy() {
  FILE* fd;
  char buff[1024] = {0};
  TotalCpuOccpy t;

  fd = fopen("/proc/stat", "r");
  if (nullptr == fd) {
    return 0;
  }

  char name[64] = {0};
  if (fgets(buff, sizeof(buff), fd)) {
    sscanf(buff, "%s %ld %ld %ld %ld", name, &t.user, &t.nice, &t.system,
           &t.idle);
  }
  fclose(fd);
  return (t.user + t.nice + t.system + t.idle);
}

uint64_t ProcessProfiler::GetCpuProcOccpy(unsigned int pid) {
  constexpr int kProcessItem = 14;
  char file_name[64] = {0};
  ProcCpuOccpy t;
  FILE* fd;
  char line_buff[1024] = {0};
  snprintf(file_name, sizeof(file_name), "/proc/%d/stat", pid);

  fd = fopen(file_name, "r");
  if (nullptr == fd) {
    return 0;
  }

  if (fgets(line_buff, sizeof(line_buff), fd)) {
    sscanf(line_buff, "%u", &t.pid);
    const char* q = GetItems(line_buff, kProcessItem);
    sscanf(q, "%ld %ld %ld %ld", &t.utime, &t.stime, &t.cutime, &t.cstime);
  }
  fclose(fd);
  return (t.utime + t.stime + t.cutime + t.cstime);
}

float ProcessProfiler::GetProcCpu(unsigned int pid) {
  uint64_t totalcputime1, totalcputime2;
  uint64_t procputime1, procputime2;

  totalcputime1 = GetCpuTotalOccpy();
  procputime1 = GetCpuProcOccpy(pid);

  sleep(1);

  totalcputime2 = GetCpuTotalOccpy();
  procputime2 = GetCpuProcOccpy(pid);

  float pcpu = 0.0;
  if (0 != totalcputime2 - totalcputime1) {
    pcpu =
        100.0 * (procputime2 - procputime1) / (totalcputime2 - totalcputime1);
  }
  return pcpu;
}

bool ProcessProfiler::GetProcStatus(ProcStatus* status) {
  if (!status) {
    return false;
  }
  char file_name[64] = {0};
  char line_buff[512] = {0};
  snprintf(file_name, sizeof(file_name), "/proc/%d/status", pid_);
  FILE* fd = fopen(file_name, "r");
  if (nullptr == fd) {
    return 0;
  }

  while (fgets(line_buff, sizeof(line_buff), fd)) {
    char line_name[64] = {0};
    if (strstr(line_buff, "Name:")) {
      sscanf(line_buff, "%s %s", line_name, status->name);
    } else if (strstr(line_buff, "VmSize:")) {
      sscanf(line_buff, "%s %d", line_name, &status->vm_size);
    } else if (strstr(line_buff, "VmRSS:")) {
      sscanf(line_buff, "%s %d", line_name, &status->mem_size);
    } else if (strstr(line_buff, "Threads:")) {
      sscanf(line_buff, "%s %d", line_name, &status->threads_num);
    }
  }
  fclose(fd);
  return true;
}

void ProcessProfiler::Run() {
  ProcStatus status;
  // for the process name
  GetProcStatus(&status);
  // current time
  ::time_t current_time;
  ::time(&current_time);  // Get local time in time_t
  char formated_time[80] = {0};
  strftime(formated_time, 80, "%F.%T", localtime(&current_time));
  // log file dir (in home dir of this user)
  std::string user_home_dir = std::string(getpwuid(getuid())->pw_dir);
  std::string dir = user_home_dir + "/";
#ifdef _PROF_LOG_DIR_
  dir += std::string(_PROF_LOG_DIR_);
  {
    if (mkdir(dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) < 0) {
      if (errno != EEXIST) {
        perror("Log dir open error -- ");
        dir = user_home_dir + "/";
      }
    }
  }
#endif
  std::string log_filename = dir + "/" + std::string(status.name) + ".log." +
                             std::string(formated_time);
  std::ofstream log_fstream(log_filename);
  while (run_) {
    status.cpu_occpy = GetProcCpu(pid_);
    memset(status.name, 0, sizeof(status.name));
    if (!GetProcStatus(&status)) {
      continue;
    }

    if (log_fstream.is_open()) {
      log_fstream << status.cpu_occpy << "%, " << status.mem_size << ", "
                  << status.vm_size << std::endl;
    }
  }
  if (log_fstream.is_open()) {
    log_fstream.close();
  }
}

}  // namespace performance
}  // namespace common
}  // namespace static_map
