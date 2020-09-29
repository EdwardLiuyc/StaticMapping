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
#define BOOST_TEST_MODULE SimpleTimeTest
#include <boost/test/unit_test.hpp>

namespace static_map {

struct TestDataStruct {
  SimpleTime time;
  int i = 0;
};

BOOST_AUTO_TEST_CASE(BinarySearch) {
  std::vector<TestDataStruct> test_vec;
  for (int i = 1; i <= 20; ++i) {
    test_vec.push_back({SimpleTime::from_sec(0.1 * i), i});
  }
  {
    const auto pair =
        TimeStampBinarySearch(test_vec, SimpleTime::from_sec(0.25));
    BOOST_CHECK_EQUAL(pair.first, 1);
    BOOST_CHECK_EQUAL(pair.second, 2);
  }
  {
    const auto pair =
        TimeStampBinarySearch(test_vec, SimpleTime::from_sec(0.35));
    BOOST_CHECK_EQUAL(pair.first, 2);
    BOOST_CHECK_EQUAL(pair.second, 3);
  }
  {
    const auto pair =
        TimeStampBinarySearch(test_vec, SimpleTime::from_sec(0.4));
    BOOST_CHECK_EQUAL(pair.first, 3);
    BOOST_CHECK_EQUAL(pair.second, 4);
  }
  {
    const auto pair =
        TimeStampBinarySearch(test_vec, SimpleTime::from_sec(0.5));
    BOOST_CHECK_EQUAL(pair.first, 4);
    BOOST_CHECK_EQUAL(pair.second, 5);
  }
}

}  // namespace static_map
