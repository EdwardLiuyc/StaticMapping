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

#pragma once

#include <stdio.h>
#include <sys/time.h>
#include <sys/unistd.h>
#include <chrono>
#include <cstring>
#include <limits>
#include <thread>

#include <ostream>
#include <sstream>
#include <string>

#include "common/macro_defines.h"

namespace static_map {
/*!
 * @class SimpleTime
 * @brief class SimpleTime
 * @author edward.liu
 */
class SimpleTime {
 public:
  // @todo(edward) seperate this into .h and .cc
  SimpleTime() : secs(0), nsecs(0), u_judge_range(1) {}
  SimpleTime(uint32_t s, uint32_t n) : secs(s), nsecs(n), u_judge_range(1) {}

  ~SimpleTime() {}

  /*!
   * \brief get current system time into a SimpleTime instance
   */
  static SimpleTime get_current_time() {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);

    return SimpleTime(ts.tv_sec, ts.tv_nsec);
  }

  /*!
   * \example bgs_time.h
   * this is a example for static function in SimpleTime: for_sec(double sec)
   * * 2.03 -> SimpleTime: secs = 2; nsecs = 0.03 * 1000000000;
   * * -2.03 -> SimpleTime: secs = 0; nsecs = 0;
   *
   * this is a example for static function in SimpleTime: fromNSec(double sec)
   * * 2300000000 -> SimpleTime: secs = 2; nsecs = 300000000;
   */
  /*!
   * \brief get a SimpleTime instance from time in second
   */
  static SimpleTime from_sec(double sec) {
    SimpleTime time;

    if (sec < 0.) return time;

    time.secs = (uint32_t)sec;
    time.nsecs = (uint32_t)((sec - time.secs) * 1000000000);
    return time;
  }

  /*!
   * \brief get a SimpleTime instance from time in ns
   */
  static SimpleTime fromNSec(uint64_t t) {
    return SimpleTime((uint32_t)(t / 1000000000), (uint32_t)(t % 1000000000));
  }

  /*!
   * \brief get a SimpleTime instance with the max value
   */
  static SimpleTime TIME_MAX() {
    return SimpleTime((uint32_t)(0xFFFFFFFF), (uint32_t)999999999);
  }

  SimpleTime &operator=(const SimpleTime &b) {
    this->nsecs = b.nsecs;
    this->secs = b.secs;
    return *this;
  }

  bool operator!=(const SimpleTime &b) const { return !((*this) == b); }

  /// @todo the threshold (u_judge_range) should not be a const value
  bool operator==(const SimpleTime &b) const {
    int64_t diff = this->toNSec() - b.toNSec();
    int64_t range = u_judge_range;

    if (diff < 0) diff = -diff;
    return (diff <= range);
  }

  /*!
   * \warning there is risk of overflow
   */
  SimpleTime operator+(const SimpleTime &b) const {
    int64_t tmp_nsecs = this->toNSec() + b.toNSec();
    return fromNSec(tmp_nsecs);
  }

  /*!
   * \warning there is risk of overflow
   */
  SimpleTime operator-(const SimpleTime &b) const {
    int64_t tmp_nsecs = this->toNSec() - b.toNSec();
    if (tmp_nsecs < 0) {
      PRINT_INFO("Result is set to 0!");
      tmp_nsecs = 0;
    }
    return fromNSec(tmp_nsecs);
  }

  SimpleTime &operator/=(const int &a) {
    if (a == 0) {
      PRINT_ERROR("Div 0! return itself!");
      return *this;
    }

    int div = a;
    if (div < 0) {
      PRINT_WARNING("/a -> /(-a)!");
      div = -div;
    }
    int64_t tmp_nsecs = this->toNSec();
    tmp_nsecs /= div;

    secs = (uint32_t)(tmp_nsecs / 1000000000);
    nsecs = (uint32_t)(tmp_nsecs % 1000000000);

    return *this;
  }

  /*!
   * \warning there is risk of overflow
   */
  SimpleTime &operator+=(const SimpleTime &b) {
    *this = *this + b;
    return *this;
  }

  /*!
   * @todo use the "judge range" in these judgement
   */
  bool operator<(const SimpleTime &b) const { return (toNSec() < b.toNSec()); }
  bool operator<=(const SimpleTime &b) const {
    return (toNSec() <= b.toNSec());
  }
  bool operator>(const SimpleTime &b) const { return (toNSec() > b.toNSec()); }
  bool operator>=(const SimpleTime &b) const {
    return (toNSec() >= b.toNSec());
  }

  friend std::ostream &operator<<(std::ostream &os, SimpleTime t1) {
    os << t1.secs << "s " << t1.nsecs << "ns ";
    return os;
  }

  std::string DebugString() const {
    std::ostringstream out;
    out << secs << "s " << nsecs << "ns ";
    return out.str();
  }

  /*!
   * \brief transfrom the SimpleTime to a double value (seconds)
   */
  inline double toSec() const {
    return static_cast<double>(this->secs) +
           static_cast<double>(this->nsecs * 1.e-9);
  }

  /*!
   * \brief determining if the time euqals zero
   */
  inline bool isZero() const { return (nsecs == 0 && secs == 0); }

  /*!
   * \brief transfrom the SimpleTime to a int value (ns)
   */
  inline int64_t toNSec() const {
    return (int64_t)((uint64_t)secs * 1000000000ull + (uint64_t)nsecs);
  }

  /*!
   * \brief using usleep to delay
   */
  inline void sleep() {
    auto sleep_time = std::chrono::nanoseconds(toNSec());
    std::this_thread::sleep_for(sleep_time);
  }

  /*!
   * \brief set the time to zero
   */
  inline void zero() { secs = nsecs = 0; }

  uint32_t secs;
  uint32_t nsecs;

 private:
  /*!
   * @brief it is a range when determining if two time are euqal
   * for now, it is set to 1000 (ns)
   *
   * @todo can be set by user
   */
  uint32_t u_judge_range;
};

using PclTimeStamp = uint64_t;
static inline SimpleTime ToLocalTime(const PclTimeStamp &time) {
  return SimpleTime::fromNSec(time * 1000ull);
}

static inline PclTimeStamp ToPclTime(const SimpleTime &time) {
  return time.toNSec() / 1000ull;
}

}  // namespace static_map
