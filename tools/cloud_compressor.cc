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
#include <pcl/filters/random_sample.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <chrono>
#include <iostream>

// @todo use zlib instead of blosc
#if 0

#include <blosc.h>

int main(int argc, char** argv) {
  std::string pcd_filename = "";
  pcl::console::parse_argument(argc, argv, "-pc", pcd_filename);
  if (pcd_filename.empty()) {
    std::cout << "point cloud file is empty!" << std::endl;
    return -1;
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_filename, *cloud) == -1) {
    std::cout << "can not load pcd file \"" << pcd_filename << "\" into cloud."
              << std::endl;
    return -1;
  }

  blosc_init();
  blosc_set_compressor("snappy");
  blosc_set_nthreads(4);

  const int cloud_size =
      cloud->points.size() > 100000 ? 100000 : cloud->points.size();
  const int dim = 4;
  const int mem_size = cloud_size * dim * sizeof(int32_t);
  int32_t* original_mem = reinterpret_cast<int32_t*>(malloc(mem_size));
  int32_t* out_mem = reinterpret_cast<int32_t*>(malloc(mem_size));
  int32_t* decompressed_mem = reinterpret_cast<int32_t*>(malloc(mem_size));
  for (int i = 0; i < cloud_size; ++i) {
    auto& point = cloud->points[i];
    original_mem[i * dim] = (int32_t)(point.x * 1000);
    original_mem[i * dim + 1] = (int32_t)(point.y * 1000);
    original_mem[i * dim + 2] = (int32_t)(point.z * 1000);
    original_mem[i * dim + 3] = (int32_t)(point.intensity * 1000);
  }

  auto start = std::chrono::high_resolution_clock::now();
  int csize = blosc_compress(5, 1, sizeof(int32_t), mem_size, original_mem,
                             out_mem, mem_size);
  std::chrono::duration<double, std::ratio<1, 1>> duration_c(
      std::chrono::high_resolution_clock::now() - start);
  std::cout << "compress: " << duration_c.count() << "s" << std::endl;
  if (csize == 0) {
    printf("Buffer is uncompressible.  Giving up.\n");
    return 1;
  } else if (csize < 0) {
    printf("Compression error.  Error code: %d\n", csize);
    return csize;
  }
  printf("Compression: %d -> %d (%.1fx)\n", mem_size, csize,
         (1. * mem_size) / csize);

  start = std::chrono::high_resolution_clock::now();
  int dsize = blosc_decompress(out_mem, decompressed_mem, mem_size);
  if (dsize < 0) {
    printf("Decompression error.  Error code: %d\n", dsize);
  }
  std::chrono::duration<double, std::ratio<1, 1>> duration_d(
      std::chrono::high_resolution_clock::now() - start);
  std::cout << "decompress: " << duration_d.count() << "s" << std::endl;
  printf("Decompression size: %d\n", dsize);
  printf("Decompression succesful!\n");

  bool same = true;
  for (int i = 0; i < cloud_size; i++) {
    if (decompressed_mem[i] != original_mem[i]) {
      std::cout << i << ":" << decompressed_mem[i] << " " << original_mem[i]
                << std::endl;
      same = false;
      break;
    }
  }
  if (!same) {
    printf("Decompressed data differs from original!\n");
  }

  blosc_destroy();
  free(original_mem);
  free(out_mem);
  free(decompressed_mem);

  printf("Succesful roundtrip!\n");
  return 0;
}

#endif
