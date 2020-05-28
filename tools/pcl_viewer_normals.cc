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

#include <pcl/features/integral_image_normal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

int main(int argc, char** argv) {
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZINormal>);
  // read pcd file
  if (pcl::io::loadPCDFile<pcl::PointXYZINormal>(argv[1], *cloud) != 0) {
    return -1;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr points(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  for (int i = 0; i < cloud->size(); ++i) {
    points->push_back(pcl::PointXYZ(cloud->points[i].x, cloud->points[i].y,
                                    cloud->points[i].z));
    normals->push_back(pcl::Normal(cloud->points[i].normal_x,
                                   cloud->points[i].normal_y,
                                   cloud->points[i].normal_z));
  }

  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ>
      fildColor(points, "z");
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer("Points With Normals"));
  viewer->addPointCloud<pcl::PointXYZ>(points, fildColor, "cloud");
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
  viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(points, normals, 1,
                                                           0.3, "normals");
  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }

  return 0;
}
