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

#include "boost/thread/thread.hpp"
#include "builder/data/cloud_types.h"
#include "common/file_utils.h"
#include "pcl/console/parse.h"
#include "pcl/features/integral_image_normal.h"
#include "pcl/io/pcd_io.h"
#include "pcl/visualization/pcl_visualizer.h"

int main(int argc, char** argv) {
  std::string bin_filename = "";
  pcl::console::parse_argument(argc, argv, "-f", bin_filename);
  if (bin_filename.empty() || !static_map::common::FileExist(bin_filename)) {
    PRINT_ERROR("bin_filename not exist!");
    return -1;
  }

  // Load .bin file.
  static_map::data::InnerCloudType::Ptr inner_cloud(
      new static_map::data::InnerCloudType);
  std::fstream fs(bin_filename, std::ios::binary | std::ios::in);
  if (!fs.is_open()) {
    PRINT_ERROR_FMT("Can not open %s", bin_filename.c_str());
    return -1;
  }
  if (inner_cloud->Deserialize(&fs) < 0) {
    PRINT_ERROR("Load file failed.");
    return -1;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  static_map::data::ToPclPointCloud(*inner_cloud, pcl_cloud.get());

  // View the bin file as xyzi points.
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer("Bin viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> color(
      pcl_cloud, "intensity");
  viewer->addPointCloud<pcl::PointXYZI>(pcl_cloud, color, bin_filename);
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, bin_filename);

  viewer->addCoordinateSystem(1.);
  viewer->setCameraPosition(0, 0, 100, 0, 0, 0);
  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(1000));
  }

  return 0;
}
