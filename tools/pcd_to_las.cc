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

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/filters/random_sample.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <liblas/liblas.hpp>

int main(int argc, char** argv) {
  std::string input_pcdfile = "";
  pcl::console::parse_argument(argc, argv, "-i", input_pcdfile);
  if (input_pcdfile.empty()) {
    std::cout << "Should use it this way: \n\n    pcl_to_las -i pcd_filename\n"
              << std::endl;
    return -1;
  }

  std::string output_lasfile = "";
  pcl::console::parse_argument(argc, argv, "-o", output_lasfile);
  if (output_lasfile.empty()) {
    output_lasfile = input_pcdfile + ".las";
  }

  pcl::PCLPointCloud2 cloud;
  pcl::io::loadPCDFile(input_pcdfile, cloud);

  liblas::Header header;
  header.SetScale(0.01, 0.01, 0.01);
  header.SetDataFormatId(liblas::ePointFormat1);  // Time only

  // Set coordinate system using GDAL support
  liblas::SpatialReference srs;
  srs.SetFromUserInput("EPSG:4326");
  header.SetSRS(srs);

  std::ofstream las_ofs;
  las_ofs.open(output_lasfile, std::ios::out | std::ios::binary);
  if (las_ofs) {
    std::cout << input_pcdfile << " -> " << output_lasfile << std::endl;
  } else {
    std::cerr << "Can not open " << output_lasfile << std::endl;
    return -1;
  }

  liblas::Writer writer(las_ofs, header);
  const auto get_field_value_f = [](const pcl::PCLPointField& field,
                                    uint8_t* const point_data_start) -> double {
    double value = 0.;
    switch (field.datatype) {
      case pcl::PCLPointField::FLOAT32: {
        float float_value;
        memcpy(&float_value, point_data_start + field.offset, 4);
        value = float_value;
      } break;
      case pcl::PCLPointField::FLOAT64: {
        memcpy(&value, point_data_start + field.offset, 8);
      } break;

      default:
        break;
    }
    return value;
  };

  const int32_t point_size = cloud.height * cloud.width;
  auto data_ptr = cloud.data.data();
  for (int i = 0; i < point_size; ++i) {
    liblas::Point point(&header);
    for (const auto& field : cloud.fields) {
      if (field.name == "x") {
        point.SetX(get_field_value_f(field, data_ptr));
      }
      if (field.name == "y") {
        point.SetY(get_field_value_f(field, data_ptr));
      }
      if (field.name == "z") {
        point.SetZ(get_field_value_f(field, data_ptr));
      }
    }
    writer.WritePoint(point);
    data_ptr += cloud.point_step;
  }

  return 0;
}
