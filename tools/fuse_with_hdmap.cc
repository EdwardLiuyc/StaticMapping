#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

#include <pcl/console/parse.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/impl/search.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "fuse_hdmap_node");
  ros::NodeHandle n;

  // map cloud
  std::string map_filename = "";
  pcl::console::parse_argument(argc, argv, "-map", map_filename);
  if (map_filename.empty()) {
    std::cout << "map file is empty!" << std::endl;
    return -1;
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(map_filename, *map_cloud) == -1) {
    std::cout << "can not load pcd file \"" << map_filename << "\" into cloud."
              << std::endl;
    return (-1);
  }

  // path cloud
  std::string path_filename = "";
  pcl::console::parse_argument(argc, argv, "-path", path_filename);
  if (path_filename.empty()) {
    std::cout << "path file is empty." << std::endl;
    return -1;
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr path_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(path_filename, *path_cloud) == -1) {
    std::cout << "can not load pcd file \"" << path_filename << "\" into cloud."
              << std::endl;
    return (-1);
  }

  std::string hdmap_file = "";
  pcl::console::parse_argument(argc, argv, "-hdmap", hdmap_file);
  if (hdmap_file.empty()) {
    std::cout << "hdmap file is empty." << std::endl;
    return -1;
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr hdmap_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(hdmap_file, *hdmap_cloud) == -1) {
    std::cout << "can not load pcd file \"" << hdmap_file << "\" into cloud."
              << std::endl;
    return (-1);
  }
  std::cout << "Got the HDMap." << std::endl;

  // utm offset
  double x = 0.;
  double y = 0.;
  pcl::console::parse_argument(argc, argv, "-x", x);
  pcl::console::parse_argument(argc, argv, "-y", y);

  x -= 367016.9124389186;
  y -= 3451183.2851449004;

  std::unique_ptr<pcl::KdTreeFLANN<pcl::PointXYZI>> path_kdtree(
      new pcl::KdTreeFLANN<pcl::PointXYZI>);
  path_kdtree->setInputCloud(path_cloud);
  const float z_offset = 0.5;
  for (auto& hd_map_point : hdmap_cloud->points) {
    hd_map_point.x -= x;
    hd_map_point.y -= y;
    // hd_map_point.z = 10.;
    std::vector<int> index;
    std::vector<float> distance;
    path_kdtree->nearestKSearch(hd_map_point, 1, index, distance);
    hd_map_point.z = path_cloud->points[index[0]].z + z_offset;
    hd_map_point.intensity = 240.;
  }

  for (auto& path_point : path_cloud->points) {
    path_point.z += z_offset;
    path_point.intensity = 20.;
  }
  if (map_cloud) {
    *map_cloud += *hdmap_cloud;
    *map_cloud += *path_cloud;
  }
  pcl::io::savePCDFileBinaryCompressed("fused_hdmap_map.pcd", *map_cloud);
  return 0;
}