
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

#include "common/file_utils.h"

#include <sys/stat.h>
#include <boost/filesystem.hpp>

namespace static_map {
namespace common {

bool FileExist(const std::string& name) {
  return boost::filesystem::exists(name);
}

std::string FilePath(const std::string& file) {
  size_t found = file.find_last_of("/");
  std::string file_path = "";
  if (found != std::string::npos) {
    file_path = file.substr(0, found);
    file_path += "/";
  }
  return file_path;
}

bool CreateDir(const std::string& path) {
  std::string path_without_slash = path;
  while (!path_without_slash.empty() && path_without_slash.back() == '/') {
    path_without_slash.resize(path_without_slash.size() - 1);
  }
  if (path_without_slash.empty() ||
      boost::filesystem::exists(path_without_slash)) {
    return true;
  }
  return boost::filesystem::create_directories(path_without_slash);
}

}  // namespace common
}  // namespace static_map
