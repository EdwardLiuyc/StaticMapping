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

#ifndef COMMON_ATOMIC_WRAPPER_H_
#define COMMON_ATOMIC_WRAPPER_H_

#include <atomic>

namespace static_map {
namespace common {

/// @class AtomicWrapper
/// @brief A wrapper class for std::atomic.
/// @details Why do we need this class:
/// When one wants to use std::atomic as a value of std::pair or just an element
/// of vector, it would be a problem that std::atomic is not
/// EmplaceConstructable. so we need a wrapper class to make it work.
template <typename T>
class AtomicWrapper {
 private:
  std::atomic<T> atom_;

 public:
  AtomicWrapper();
  explicit AtomicWrapper(const T& t);
  explicit AtomicWrapper(const std::atomic<T>& a);
  AtomicWrapper(const AtomicWrapper& other);

  AtomicWrapper& operator=(const T& other_value);
  AtomicWrapper& operator=(const AtomicWrapper& other);

  operator T();

  /// @brief Pre-increment
  template <std::enable_if_t<std::is_integral<T>::value, bool> = true>
  T operator++() {
    return ++atom_;
  }
  /// @brief Post-increment
  template <std::enable_if_t<std::is_integral<T>::value, bool> = true>
  T operator++(int) {
    return atom_++;
  }
};

template <typename T>
AtomicWrapper<T>::AtomicWrapper() : atom_() {}

template <typename T>
AtomicWrapper<T>::AtomicWrapper(const T& value) : atom_(value) {}

template <typename T>
AtomicWrapper<T>::AtomicWrapper(const std::atomic<T>& a) : atom_(a.load()) {}

template <typename T>
AtomicWrapper<T>::AtomicWrapper(const AtomicWrapper<T>& other)
    : atom_(other.atom_.load()) {}

template <typename T>
AtomicWrapper<T>& AtomicWrapper<T>::operator=(const T& other_value) {
  atom_.store(other_value);
  return *this;
}

template <typename T>
AtomicWrapper<T>& AtomicWrapper<T>::operator=(const AtomicWrapper<T>& other) {
  atom_.store(other.atom_.load());
  return *this;
}

template <typename T>
AtomicWrapper<T>::operator T() {
  return atom_.load();
}

}  // namespace common
}  // namespace static_map

#endif  // COMMON_ATOMIC_WRAPPER_H_
