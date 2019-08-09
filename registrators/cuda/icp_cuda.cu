#include "icp_cuda.h"

#include <cugar/basic/timer.h>
#include <cugar/kd/cuda/kd_builder.h>
#include <cugar/kd/cuda/kd_context.h>
#include <cugar/kd/cuda/knn.h>
#include <cugar/sampling/random.h>
#include <thrust/gather.h>

#include "common/simple_thread_pool.h"

#define DEBUG_CUDA 0

namespace static_map {
namespace registrator {
namespace cuda {

void init_cuda_device() { cudaSetDevice(0); }

using cugar::device_tag;
using cugar::host_tag;
bool knn_cugar(const float *ref, int ref_points_num, const float *query,
               int query_points_num, float *knn_dist2, int *knn_index) {
#if DEBUG_CUDA
  cudaEvent_t start, stop;
  float dtime = 0;
  cudaEventCreate(&start);
  cudaEventCreate(&stop);
  cudaEventRecord(start, 0);
#endif

  cugar::Vector3f bbox_min(1.e6);
  cugar::Vector3f bbox_max(-1.e6);
  cugar::vector<host_tag, cugar::Vector4f> h_ref_points(ref_points_num);
  for (int i = 0; i < ref_points_num; ++i) {
    h_ref_points[i] = cugar::Vector4f(
        ref[i * kPointDim], ref[i * kPointDim + 1], ref[i * kPointDim + 2], 0.);
    if (h_ref_points[i][0] < bbox_min[0]) {
      bbox_min[0] = h_ref_points[i][0];
    }
    if (h_ref_points[i][1] < bbox_min[1]) {
      bbox_min[1] = h_ref_points[i][1];
    }
    if (h_ref_points[i][2] < bbox_min[2]) {
      bbox_min[2] = h_ref_points[i][2];
    }

    if (h_ref_points[i][0] > bbox_max[0]) {
      bbox_max[0] = h_ref_points[i][0];
    }
    if (h_ref_points[i][1] > bbox_max[1]) {
      bbox_max[1] = h_ref_points[i][1];
    }
    if (h_ref_points[i][2] > bbox_max[2]) {
      bbox_max[2] = h_ref_points[i][2];
    }
  }
  cugar::vector<host_tag, cugar::Vector4f> h_query_points(query_points_num);
  for (int i = 0; i < query_points_num; ++i) {
    h_query_points[i] =
        cugar::Vector4f(query[i * kPointDim], query[i * kPointDim + 1],
                        query[i * kPointDim + 2], 1.f);
  }

#if DEBUG_CUDA
  cudaEventRecord(stop, 0);
  cudaEventSynchronize(stop);
  cudaEventElapsedTime(&dtime, start, stop);
  fprintf(stderr, "bbox time: %f ms\n", dtime);
#endif

  cugar::vector<device_tag, cugar::Vector4f> d_ref_points(h_ref_points);
  cugar::vector<device_tag, cugar::Vector4f> d_query_points(h_query_points);
  cugar::vector<device_tag, cugar::Kd_node> kd_nodes;
  cugar::vector<device_tag, uint2> kd_leaves;
  cugar::vector<device_tag, uint2> kd_ranges;
  cugar::vector<device_tag, cugar::uint32> kd_index;

  cugar::cuda::Kd_context context(&kd_nodes, &kd_leaves, &kd_ranges);
  cugar::cuda::Kd_builder<cugar::uint32> builder;

// build kdtree
#if DEBUG_CUDA
  cudaEventRecord(start, 0);
#endif
  builder.build(context, kd_index, cugar::Bbox3f(bbox_min, bbox_max),
                d_ref_points.begin(), d_ref_points.end(), 8u);
#if DEBUG_CUDA
  cudaEventRecord(stop, 0);
  cudaEventSynchronize(stop);
  cudaEventElapsedTime(&dtime, start, stop);
  fprintf(stderr, "build time: %f ms\n", dtime);
#endif

  cugar::vector<device_tag, cugar::Vector4f> d_sorted_points(ref_points_num);
  thrust::gather(kd_index.begin(), kd_index.begin() + ref_points_num,
                 d_ref_points.begin(), d_sorted_points.begin());
  d_ref_points = d_sorted_points;

#if DEBUG_CUDA
  cudaEventRecord(start, 0);
#endif
  // knn search
  cugar::cuda::Kd_knn<kPointDim> knn;
  cugar::vector<device_tag, cugar::cuda::Kd_knn_result> d_results(
      query_points_num * kNNForNormal);
  const cugar::Vector4f *kd_points_ptr = raw_pointer(d_ref_points);
  const cugar::Vector4f *query_points_ptr = raw_pointer(d_query_points);
  knn.run<kNNForNormal>(
      query_points_ptr, query_points_ptr + query_points_num,
      raw_pointer(kd_nodes), raw_pointer(kd_ranges), raw_pointer(kd_leaves),
      cugar::cuda::make_load_pointer<cugar::cuda::LOAD_LDG>(kd_points_ptr),
      raw_pointer(d_results));

#if DEBUG_CUDA
  cudaEventRecord(stop, 0);
  cudaEventSynchronize(stop);
  cudaEventElapsedTime(&dtime, start, stop);
  fprintf(stderr, "search time: %f ms\n", dtime);
#endif

  // cugar::cuda::sync_and_check_error("knn search error.");
  if (knn_dist2 && knn_index) {
    cugar::vector<host_tag, cugar::cuda::Kd_knn_result> h_results(d_results);
#pragma omp parallel for num_threads(LOCAL_OMP_THREADS_NUM)
    for (int i = 0; i < kNNForNormal; ++i) {
      for (int j = 0; j < query_points_num; ++j) {
        const int index = j * kNNForNormal + i;
        knn_dist2[index] = h_results[index].dist2;
        knn_index[index] = h_results[index].index;
      }
    }
  }

#if DEBUG_CUDA
  cudaEventDestroy(start);
  cudaEventDestroy(stop);
#endif

  return true;
}

}  // namespace cuda
}  // namespace registrator
}  // namespace static_map