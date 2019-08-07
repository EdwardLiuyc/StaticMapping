#include <pcl/console/parse.h>
#include <pcl/filters/random_sample.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>

int main(int argc, char** argv) {
  std::string pcd_filename = "";
  pcl::console::parse_argument(argc, argv, "-pc", pcd_filename);
  if (pcd_filename.empty()) {
    std::cout << "point cloud topic is empty!" << std::endl;
    return -1;
  }

  float ratio = 0.05;
  pcl::console::parse_argument(argc, argv, "-r", ratio);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);

  if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_filename, *cloud) == -1) {
    PCL_ERROR("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }

  pcl::RandomSample<pcl::PointXYZI> sample(true);
  sample.setInputCloud(cloud);
  sample.setSample((int)(cloud->size() * ratio));
  // Cloud
  pcl::PointCloud<pcl::PointXYZI> cloud_out;
  sample.filter(cloud_out);
  pcl::io::savePCDFileBinaryCompressed("sampled.pcd", cloud_out);

  return (0);
}