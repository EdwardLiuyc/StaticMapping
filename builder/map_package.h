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

#ifndef BUILDER_MAP_PACKAGE_H_
#define BUILDER_MAP_PACKAGE_H_

#include <memory>
#include <string>
#include <vector>

#include "builder/multi_resolution_voxel_map.h"

namespace static_map {

class Trajectory;

struct MapPackageOptions {
  bool enable = true;
  double border_offset = 100.;
  double piece_width = 500.;
  std::string cloud_file_prefix = "part_";
  std::string descript_filename = "map_package.xml";
};

bool SaveTrajectoriesAsMapPackage(
    const std::vector<std::shared_ptr<Trajectory>> trajectores,
    const MapPackageOptions& map_package_options,
    const MrvmSettings& mrvm_options, const std::string& export_path);

}  // namespace static_map

#endif  // BUILDER_MAP_PACKAGE_H_
