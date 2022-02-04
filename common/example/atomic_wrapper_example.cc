// MIT License

// Copyright (c) 2022 Edward Liu

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

#include <iostream>
#include <thread>

#include "common/atomic_wrapper.h"

using static_map::common::AtomicWrapper;

int main() {
  AtomicWrapper<int32_t> atomic_int(0);
  std::atomic<int32_t> std_atomic_int(0);

  std::thread t1([&]() -> void {
    for (int i = 0; i < 1000000; ++i) {
      ++atomic_int;
      atomic_int++;
      ++std_atomic_int;
      std_atomic_int++;
    }
  });

  std::thread t2([&]() -> void {
    for (int i = 0; i < 1000000; ++i) {
      ++atomic_int;
      atomic_int++;
      ++std_atomic_int;
      std_atomic_int++;
    }
  });

  t1.join();
  t2.join();

  std::cout << int32_t(atomic_int) << std::endl;
  std::cout << int32_t(std_atomic_int) << std::endl;

  return 0;
}
