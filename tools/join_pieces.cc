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

#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/filters/random_sample.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>

#include "pugixml/pugixml.hpp"

int main(int argc, char** argv) {
  std::string pkg_filename = "";
  pcl::console::parse_argument(argc, argv, "-pkg", pkg_filename);
  if (pkg_filename.empty()) {
    std::cout << "Should use it this way: \n\n    join_pieces -pkg [filename]\n"
              << std::endl;
    return -1;
  }

  size_t found = pkg_filename.find_last_of("/");
  std::string pkg_file_path = "";
  if (found != std::string::npos) {
    pkg_file_path = pkg_filename.substr(0, found);
    pkg_file_path += "/";
  }

  pugi::xml_document doc;
  if (!doc.load_file(pkg_filename.c_str())) {
    std::cout << "Can not load " << pkg_filename << " as a xml." << std::endl;
    return -1;
  }
  auto package_node = doc.child("MapPackage");
  if (package_node.empty()) {
    std::cout << "Format error." << std::endl;
    return -1;
  }

  pcl::PointCloud<pcl::PointXYZI> whole_cloud;
  for (auto piece_node = package_node.child("Piece"); piece_node;
       piece_node = piece_node.next_sibling("Piece")) {
    std::string pcd_file =
        pkg_file_path + piece_node.attribute("file").as_string();
    std::cout << "Read in piece from pcd file: " << pcd_file << std::endl;

    pcl::PointCloud<pcl::PointXYZI> piece_cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_file, piece_cloud) == -1) {
      std::cout << "Error loading pointcloud from this file." << std::endl;
    } else {
      Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
      transform(0, 3) = piece_node.attribute("x").as_double();
      transform(1, 3) = piece_node.attribute("y").as_double();
      pcl::PointCloud<pcl::PointXYZI> transformed_cloud;
      pcl::transformPointCloud(piece_cloud, transformed_cloud, transform);
      whole_cloud += transformed_cloud;
    }
  }

  if (!whole_cloud.empty()) {
    pcl::io::savePCDFileBinaryCompressed("whole_static_map.pcd", whole_cloud);
  }

  return 0;
}
