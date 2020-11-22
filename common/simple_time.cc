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

#include "common/simple_time.h"

namespace static_map {

SimpleTime::SimpleTime() : secs(0), nsecs(0), u_judge_range(1) {}

SimpleTime::SimpleTime(const uint32_t s, const uint32_t n)
    : secs(s), nsecs(n), u_judge_range(1) {}

SimpleTime &SimpleTime::operator=(const SimpleTime &b) {
  this->nsecs = b.nsecs;
  this->secs = b.secs;
  return *this;
}

SimpleTime SimpleTime::GetCurrentTime() {
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  return SimpleTime(ts.tv_sec, ts.tv_nsec);
}

SimpleTime SimpleTime::FromSec(const double sec) {
  SimpleTime time;
  if (sec < 0.) {
    return time;
  }
  time.secs = (uint32_t)sec;
  time.nsecs = (uint32_t)((sec - time.secs) * 1000000000);
  return time;
}

SimpleTime SimpleTime::FromNanoSec(const uint64_t t) {
  return SimpleTime((uint32_t)(t / 1000000000), (uint32_t)(t % 1000000000));
}

SimpleTime SimpleTime::TimeMax() {
  return SimpleTime((uint32_t)(0xFFFFFFFF), (uint32_t)999999999);
}

bool SimpleTime::operator!=(const SimpleTime &b) const {
  return !((*this) == b);
}

/// @todo the threshold (u_judge_range) should not be a const value
bool SimpleTime::operator==(const SimpleTime &b) const {
  int64_t diff = this->ToNanoSec() - b.ToNanoSec();
  int64_t range = u_judge_range;

  if (diff < 0) diff = -diff;
  return (diff <= range);
}

SimpleTime SimpleTime::operator+(const SimpleTime &b) const {
  int64_t tmp_nsecs = this->ToNanoSec() + b.ToNanoSec();
  return FromNanoSec(tmp_nsecs);
}

SimpleTime SimpleTime::operator-(const SimpleTime &b) const {
  int64_t tmp_nsecs = this->ToNanoSec() - b.ToNanoSec();
  if (tmp_nsecs < 0) {
    PRINT_INFO("Result is set to 0!");
    tmp_nsecs = 0;
  }
  return FromNanoSec(tmp_nsecs);
}

SimpleTime &SimpleTime::operator/=(const int &a) {
  if (a == 0) {
    PRINT_ERROR("Div 0! return itself!");
    return *this;
  }

  int div = a;
  if (div < 0) {
    PRINT_WARNING("/a -> /(-a)!");
    div = -div;
  }
  int64_t tmp_nsecs = this->ToNanoSec();
  tmp_nsecs /= div;

  secs = (uint32_t)(tmp_nsecs / 1000000000);
  nsecs = (uint32_t)(tmp_nsecs % 1000000000);

  return *this;
}

SimpleTime &SimpleTime::operator+=(const SimpleTime &b) {
  *this = *this + b;
  return *this;
}

bool SimpleTime::operator<(const SimpleTime &b) const {
  return (ToNanoSec() < b.ToNanoSec());
}
bool SimpleTime::operator<=(const SimpleTime &b) const {
  return (ToNanoSec() <= b.ToNanoSec());
}
bool SimpleTime::operator>(const SimpleTime &b) const {
  return (ToNanoSec() > b.ToNanoSec());
}
bool SimpleTime::operator>=(const SimpleTime &b) const {
  return (ToNanoSec() >= b.ToNanoSec());
}
std::ostream &operator<<(std::ostream &os, SimpleTime t1) {
  os << t1.DebugString();
  return os;
}

std::string SimpleTime::DebugString() const {
  std::ostringstream out;
  out << secs << "s " << nsecs << "ns ";
  return out.str();
}

}  // namespace static_map
