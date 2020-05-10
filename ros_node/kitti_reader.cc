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

// kitti documentation for point cloud raw data
/*

Velodyne 3D laser scan data
===========================

The velodyne point clouds are stored in the folder 'velodyne_points'. To
save space, all scans have been stored as Nx4 float matrix into a binary
file using the following code:

  stream = fopen (dst_file.c_str(),"wb");
  fwrite(data,sizeof(float),4*num,stream);
  fclose(stream);

Here, data contains 4*num values, where the first 3 values correspond to
x,y and z, and the last value is the reflectance information. All scans
are stored row-aligned, meaning that the first 4 values correspond to the
first measurement. Since each scan might potentially have a different
number of points, this must be determined from the file size when reading
the file, where 1e6 is a good enough upper bound on the number of values:

  // allocate 4 MB buffer (only ~130*4*4 KB are needed)
  int32_t num = 1000000;
  float *data = (float*)malloc(num*sizeof(float));

  // pointers
  float *px = data+0;
  float *py = data+1;
  float *pz = data+2;
  float *pr = data+3;

  // load point cloud
  FILE *stream;
  stream = fopen (currFilenameBinary.c_str(),"rb");
  num = fread(data,sizeof(float),num,stream)/4;
  for (int32_t i=0; i<num; i++) {
    point_cloud.points.push_back(tPoint(*px,*py,*pz,*pr));
    px+=4; py+=4; pz+=4; pr+=4;
  }
  fclose(stream);

x,y and y are stored in metric (m) Velodyne coordinates.

IMPORTANT NOTE: Note that the velodyne scanner takes depth measurements
continuously while rotating around its vertical axis (in contrast to the
cameras, which are triggered at a certain point in time). This means that when
computing point clouds you have to 'untwist' the points linearly with respect to
the velodyne scanner location at the beginning and the end of the 360° sweep.
The timestamps for the beginning and the end of the sweeps can be found in the
timestamps file. The velodyne rotates in counter-clockwise direction.

Of course this 'untwisting' only works for non-dynamic environments.

*/

#include <glog/logging.h>
#include <unistd.h>

#include "ros_node/kitti_reader.h"

namespace static_map {

void KittiReader::SetPointCloudDataPath(const std::string &path) {
  point_cloud_data_path_ = path;
  CHECK_NE(access(point_cloud_data_path_.c_str(), F_OK), -1);
}

MapBuilder::PointCloudPtr KittiReader::ReadFromBin(const int index) {
  char filename[256] = {0};
  snprintf(filename, sizeof(filename), "%s/%06d.bin",
           point_cloud_data_path_.c_str(), index);
  if (access(filename, F_OK) == -1) {
    return nullptr;
  }

  // allocate 4 MB buffer (only ~130*4*4 KB are needed)
  int32_t num = 1000000;
  float *data = reinterpret_cast<float *>(malloc(num * sizeof(float)));

  // pointers
  float *px = data + 0;
  float *py = data + 1;
  float *pz = data + 2;
  float *pr = data + 3;

  // load point cloud
  MapBuilder::PointCloudPtr point_cloud(new MapBuilder::PointCloudType);
  FILE *stream;
  stream = fopen(filename, "rb");
  num = fread(data, sizeof(float), num, stream) / 4;
  point_cloud->reserve(num);
  for (int32_t i = 0; i < num; i++) {
    pcl::PointXYZI point;
    point.x = *px;
    point.y = *py;
    point.z = *pz;
    point.intensity = *pr;
    point_cloud->points.push_back(point);
    px += 4;
    py += 4;
    pz += 4;
    pr += 4;
  }
  fclose(stream);
  return point_cloud;
}

}  // namespace static_map
