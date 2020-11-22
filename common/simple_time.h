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

/*!
 * @file simple_time.h
 * @brief header for class SimpleTime
 *
 * @author edward.liu
 * @date 2018-10-08
 *
 * @todo add the classes to namespace "common"
 */

#ifndef COMMON_SIMPLE_TIME_H_
#define COMMON_SIMPLE_TIME_H_

#include <stdio.h>
#include <sys/time.h>
#include <sys/unistd.h>
#include <chrono>
#include <cstring>
#include <limits>
#include <ostream>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "common/macro_defines.h"
#include "glog/logging.h"

namespace static_map {
class SimpleTime {
 public:
  SimpleTime();
  SimpleTime(const uint32_t s, const uint32_t n);

  SimpleTime &operator=(const SimpleTime &b);

  ~SimpleTime() = default;

  /// @brief Get current system time into a SimpleTime instance
  static SimpleTime GetCurrentTime();
  /// @brief Get a SimpleTime instance from time in second
  static SimpleTime FromSec(const double sec);
  /// @brief Get a SimpleTime instance from time in ns.
  static SimpleTime FromNanoSec(const uint64_t t);
  /// @brief Get a SimpleTime instance with the max value.
  static SimpleTime TimeMax();
  /// @brief Transfrom the SimpleTime to a double value (seconds).
  double ToSec() const;
  /// @brief Transfrom the SimpleTime to a int value (ns).
  int64_t ToNanoSec() const;
  /// @brief Set the time to zero
  inline void Zero() { secs = nsecs = 0; }
  /// @brief Determining if the time euqals zero
  inline bool IsZero() const { return (nsecs == 0 && secs == 0); }
  /// @brief Use SimpleTime as a duration and sleep for the duration.
  void Sleep();

  /// Operators
  bool operator!=(const SimpleTime &b) const;
  /// @todo the threshold (u_judge_range) should not be a const value
  bool operator==(const SimpleTime &b) const;
  /// @warning there is risk of overflow
  SimpleTime operator+(const SimpleTime &b) const;
  /// @warning there is risk of overflow
  SimpleTime operator-(const SimpleTime &b) const;
  /// @warning there is risk of overflow
  SimpleTime &operator+=(const SimpleTime &b);

  SimpleTime &operator/=(const int &a);
  /// @todo use the "judge range" in these judgement
  bool operator<(const SimpleTime &b) const;
  bool operator<=(const SimpleTime &b) const;
  bool operator>(const SimpleTime &b) const;
  bool operator>=(const SimpleTime &b) const;

  friend std::ostream &operator<<(std::ostream &os, SimpleTime t1);

  std::string DebugString() const;

 private:
  uint32_t secs;
  uint32_t nsecs;
  /*!
   * @brief it is a range when determining if two time are euqal
   * for now, it is set to 1000 (ns)
   *
   * @todo can be set by user
   */
  uint32_t u_judge_range;
};

inline double SimpleTime::ToSec() const {
  return static_cast<double>(secs) + static_cast<double>(nsecs * 1.e-9);
}

inline int64_t SimpleTime::ToNanoSec() const {
  return (int64_t)((uint64_t)secs * 1000000000ull + (uint64_t)nsecs);
}

inline void SimpleTime::Sleep() {
  auto sleep_time = std::chrono::nanoseconds(ToNanoSec());
  std::this_thread::sleep_for(sleep_time);
}

using PclTimeStamp = uint64_t;
static inline SimpleTime ToLocalTime(const PclTimeStamp &time) {
  return SimpleTime::FromNanoSec(time * 1000ull);
}

static inline PclTimeStamp ToPclTime(const SimpleTime &time) {
  return time.ToNanoSec() / 1000ull;
}

template <typename DataTypeWithTime>
std::pair<int, int> TimeStampBinarySearch(
    const std::vector<DataTypeWithTime> &data_vector, const SimpleTime &time) {
  int mid;
  int start = 0;
  int end = data_vector.size() - 1;
  while (end - start > 1) {
    mid = start + (end - start) / 2;
    if (time < data_vector.at(mid).time) {
      end = mid;
    } else {
      start = mid;
    }
  }
  CHECK_EQ(end - start, 1);
  return std::make_pair(start, end);
}

}  // namespace static_map

#endif  // COMMON_SIMPLE_TIME_H_
