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

#pragma once

#include <condition_variable>
#include <mutex>

#include <boost/thread/pthread/shared_mutex.hpp>

namespace static_map {
namespace common {

// Enable thread safety attributes only with clang.
// The attributes can be safely erased when compiling with other compilers.
#if defined(__SUPPORT_TS_ANNOTATION__) || defined(__clang__)
#define THREAD_ANNOTATION_ATTRIBUTE__(x) __attribute__((x))
#else
#define THREAD_ANNOTATION_ATTRIBUTE__(x)  // no-op
#endif

#define CAPABILITY(x) THREAD_ANNOTATION_ATTRIBUTE__(capability(x))

#define SCOPED_CAPABILITY THREAD_ANNOTATION_ATTRIBUTE__(scoped_lockable)

#define GUARDED_BY(x) THREAD_ANNOTATION_ATTRIBUTE__(guarded_by(x))

#define PT_GUARDED_BY(x) THREAD_ANNOTATION_ATTRIBUTE__(pt_guarded_by(x))

#define REQUIRES(...) \
  THREAD_ANNOTATION_ATTRIBUTE__(requires_capability(__VA_ARGS__))

#define ACQUIRE(...) \
  THREAD_ANNOTATION_ATTRIBUTE__(acquire_capability(__VA_ARGS__))

#define RELEASE(...) \
  THREAD_ANNOTATION_ATTRIBUTE__(release_capability(__VA_ARGS__))

#define EXCLUDES(...) THREAD_ANNOTATION_ATTRIBUTE__(locks_excluded(__VA_ARGS__))

#define NO_THREAD_SAFETY_ANALYSIS \
  THREAD_ANNOTATION_ATTRIBUTE__(no_thread_safety_analysis)

// Defines an annotated mutex that can only be locked through its scoped locker
// implementation.
class CAPABILITY("mutex") Mutex {
 public:
  // A RAII class that acquires a mutex in its constructor, and
  // releases it in its destructor. It also implements waiting functionality on
  // conditions that get checked whenever the mutex is released.
  class SCOPED_CAPABILITY Locker {
   public:
    explicit Locker(Mutex* mutex) ACQUIRE(mutex)
        : mutex_(mutex), lock_(mutex->mutex_) {}

    ~Locker() RELEASE() {
      lock_.unlock();
      mutex_->condition_.notify_all();
    }

    template <typename Predicate>
    void Await(Predicate predicate) REQUIRES(this) {
      mutex_->condition_.wait(lock_, predicate);
    }

   private:
    Mutex* mutex_;
    std::unique_lock<std::mutex> lock_;
  };

 private:
  std::condition_variable condition_;
  std::mutex mutex_;
};

using MutexLocker = Mutex::Locker;

/// read write mutex
/// when there is no writing in progress, several thread can access (read) the
/// memory at the same time
using ReadWriteMutex = boost::shared_mutex;
using ReadMutexLocker = boost::shared_lock<ReadWriteMutex>;
using WriteMutexLocker = boost::upgrade_to_unique_lock<ReadWriteMutex>;

}  // namespace common
}  // namespace static_map
