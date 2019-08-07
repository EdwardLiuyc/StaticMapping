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

#include <pcl/console/parse.h>
#include "builder/multi_trajectory_map_builder.h"

int main(int argc, char** argv) {
  std::string base_map_file = "";
  pcl::console::parse_argument(argc, argv, "-b", base_map_file);
  if (base_map_file.empty()) {
    PRINT_ERROR("you should use \"-b filename\" to specify the basemap xml. ");
    return -1;
  }

  std::string incremental_map_file = "";
  pcl::console::parse_argument(argc, argv, "-i", incremental_map_file);
  if (incremental_map_file.empty()) {
    PRINT_ERROR(
        "you should use \"-i filename\" to specify the incremental map xml. ");
    return -1;
  }

  std::string new_map_file = "";
  pcl::console::parse_argument(argc, argv, "-n", new_map_file);
  if (new_map_file.empty()) {
    PRINT_ERROR("you should use \"-n filename\" to specify the new map xml. ");
    return -1;
  }

  static_map::MultiTrajectoryMapBuilderOptions options;
  options.loop_dettect_settings.use_gps = false;
  options.loop_dettect_settings.max_close_loop_distance = 10.;
  options.loop_dettect_settings.nearest_history_pos_num = 3;

  static_map::MultiTrajectoryMapBuilder builder(options);

  builder.LoadBaseMap(base_map_file);
  builder.LoadIncrementalMap(incremental_map_file);
  builder.SaveWholeMap(new_map_file);
  // builder.GenerateWholeMapPcd("2map.pcd");
  builder.GenerateStaticMap("2map_static.pcd");
  return 0;
}
