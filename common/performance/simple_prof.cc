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

#include "common/performance/simple_prof.h"

#include <map>

namespace static_map {
namespace common {
namespace performance {

std::shared_ptr<TimerManager> TimerManager::instance_ = nullptr;

struct StatisticDetails {
  size_t size;
  int64_t average;
  int64_t max;
  int64_t min;
  int64_t sum;
};

inline StatisticDetails GetStatistics(std::vector<int64_t>& data) {
  std::sort(data.begin(), data.end());
  StatisticDetails statistics;
  statistics.size = data.size();
  statistics.max = data.back();
  statistics.min = data.front();

  statistics.sum = 0;
  for (const auto& d : data) {
    statistics.sum += d;
  }
  statistics.average = statistics.sum / statistics.size;
  return statistics;
}

std::vector<int64_t>* TimerManager::RegisterTimer(
    const std::string& block_name) {
  std::lock_guard<std::mutex> locker(mutex_);
  return &durations_[std::this_thread::get_id()][block_name];
}

TimerManager::~TimerManager() {
  std::map<OutputUnit, std::pair<std::string, double> > output_factor;
  output_factor[kS] = std::make_pair("(s)", 1.e-6);
  output_factor[kMs] = std::make_pair("(ms)", 1.e-3);
  output_factor[kUs] = std::make_pair("(us)", 1.);

  std::cout << std::endl;
  std::cout << "\e[1;31m" << std::setw(25) << "block name"
            << " | " << std::setw(8) << "times"
            << " | " << std::setw(9) << "avg." + output_factor[unit_].first
            << " | " << std::setw(10) << "sum." + output_factor[unit_].first
            << " | " << std::setw(9) << "min." + output_factor[unit_].first
            << " | " << std::setw(9) << "max." + output_factor[unit_].first
            << "\e[0m" << std::endl;
  for (int i = 0; i < 85; ++i) {
    std::cout << "-";
  }
  std::cout << std::endl;

  const auto time_to_string = [&](const int64_t time) -> std::string {
    int factored_time = time * output_factor[unit_].second;
    return (factored_time >= 1 ? std::to_string(factored_time) : "<1");
  };

  for (auto& id_block : durations_) {
    for (auto& name_vector : id_block.second) {
      const auto statistics = GetStatistics(name_vector.second);

      std::cout << std::setw(25) << name_vector.first << " | " << std::setw(8)
                << statistics.size << " | " << std::setw(9)
                << time_to_string(statistics.average) << " | " << std::setw(10)
                << time_to_string(statistics.sum) << " | " << std::setw(9)
                << time_to_string(statistics.min) << " | " << std::setw(9)
                << time_to_string(statistics.max) << std::endl;
    }
  }
  std::cout << std::endl;
}

}  // namespace performance
}  // namespace common
}  // namespace static_map

// INIT_PROF;
