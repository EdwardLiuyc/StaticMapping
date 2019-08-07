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

  pcl::PointCloud<pcl::PointXYZI>::Ptr path_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_filename, *path_cloud) == -1) {
    PCL_ERROR("Couldn't read file\n");
    return (-1);
  }

  std::ofstream path_file("path.csv");
  if (path_file.is_open()) {
    std::ostringstream out;
    for (auto& point : path_cloud->points) {
      out << std::to_string(point.x) << "," << std::to_string(point.y) << ","
          << std::to_string(point.z) << "," << std::to_string(point.intensity)
          << "\n";
    }

    path_file << out.str();
    path_file.close();
  } else {
    PCL_ERROR("Failed to open file.\n");
  }

  return (0);
}