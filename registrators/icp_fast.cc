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

#include <cstdlib>
#include <ctime>
#include <iostream>

#include "common/macro_defines.h"
#include "common/math.h"
#include "nabo/nabo.h"
#include "registrators/icp_fast.h"

namespace static_map {
namespace registrator {

template <typename PointT>
void IcpFast<PointT>::setInputSource(const PointCloudSourcePtr& cloud) {
  source_inner_cloud_.reset(new InnerPointCloud);
  size_t size = cloud->size();
  source_inner_cloud_->points.reserve(size);
  for (size_t i = 0; i < size; ++i) {
    source_inner_cloud_->points.emplace_back();
    source_inner_cloud_->points[i].FromPoint(cloud->points[i]);
  }
}

template <typename PointT>
void IcpFast<PointT>::setInputTarget(const PointCloudTargetPtr& cloud) {
  target_inner_cloud_.reset(new InnerPointCloud);
  int points_num = cloud->size();
  const float sample_ratio = 0.1;

#ifndef _ICP_USE_CUDA_

  const int knn = kNNForNormal;
  // step1. get all points's nerghbors (with sampling)
  Eigen::MatrixXf M(kPointDim, points_num);
  Eigen::MatrixXf query(kPointDim, points_num);
  int query_num = 0;
  for (int i = 0; i < points_num; ++i) {
    M.col(i) << cloud->points[i].x, cloud->points[i].y, cloud->points[i].z;
    if (static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX) <
        sample_ratio) {
      query.col(query_num) = M.col(i);
      query_num++;
    }
  }
  query.resize(kPointDim, query_num);
  Eigen::MatrixXi indices(knn, query_num);
  Eigen::MatrixXf dists2(knn, query_num);

  start_clock();
  // using cpu
  // use kdtree in libnabo to search every point's neighbors
  Nabo::NNSearchF* knn_searcher = Nabo::NNSearchF::createKDTreeLinearHeap(M);
  // we do not want to sort the distance

  knn_searcher->knn(query, indices, dists2, knn, 0.,
                    Nabo::NNSearchF::ALLOW_SELF_MATCH);
  end_clock(__FILE__, __FUNCTION__, __LINE__);

  delete knn_searcher;

#else  // using GPU (cuda)
  // step1. get all points's nerghbors (with sampling)
  // average 0.5ms (1ms at most) for all malloc and copy ****
  float* ref =
      reinterpret_cast<float*>(malloc(points_num * kPointDim * sizeof(float)));
  float* query =
      reinterpret_cast<float*>(malloc(points_num * kPointDim * sizeof(float)));
  int query_num = 0;
  const size_t point_mem_size = kPointDim * sizeof(float);
  for (int i = 0; i < points_num; ++i) {
    ref[i * 3] = cloud->points[i].x;
    ref[i * 3 + 1] = cloud->points[i].y;
    ref[i * 3 + 2] = cloud->points[i].z;
    if (static_cast<float>(query_num) / (i + 1) < sample_ratio) {
      memcpy(&query[query_num * kPointDim], &ref[i * kPointDim],
             point_mem_size);
      query_num++;
    }
  }
  float* result_dists = reinterpret_cast<float*>(
      malloc(query_num * kNNForNormal * sizeof(float)));
  int* result_indices =
      reinterpret_cast<int*>(malloc(query_num * kNNForNormal * sizeof(int)));
  // ****
  start_clock();
  cuda::knn_cugar(ref, points_num, query, query_num, result_dists,
                  result_indices);
  end_clock(__FILE__, __FUNCTION__, __LINE__);

  // step2. calculate normals
  start_clock();
  target_inner_cloud_->points.resize(query_num);
#if defined _OPENMP
#pragma omp parallel for num_threads(10)
#endif
  for (int i = 0; i < query_num; ++i) {
    auto& inner_point = target_inner_cloud_->points[i];
    inner_point.FromFloatArray(&query[kPointDim * i]);

    if (result_dists[i * kNNForNormal] < 1.) {
      std::vector<Eigen::Vector3f> point_for_calculating_normal(kNNForNormal);
      for (int j = 0; j < kNNForNormal; ++j) {
        const int index = result_indices[i * kNNForNormal + j];
        point_for_calculating_normal[j] << ref[kPointDim * index],
            ref[kPointDim * index + 1], ref[kPointDim * index + 2];
      }
      auto norm = common::PlaneFitting(point_for_calculating_normal);
      inner_point.normal = norm.second;
      inner_point.has_normal = true;
    }
  }
  end_clock(__FILE__, __FUNCTION__, __LINE__);

  free(ref);
  free(query);
  free(result_dists);
  free(result_indices);
#endif

  target_cloud_with_mormal_.resize(query_num, 6);
  int row_index = 0;
  for (auto& p : target_inner_cloud_->points) {
    if (!p.has_normal) {
      continue;
    }
    target_cloud_with_mormal_.row(row_index) << p.point, p.normal;
    row_index++;
  }
  target_cloud_with_mormal_.resize(row_index, 6);
}

template <typename PointT>
bool IcpFast<PointT>::align(const Eigen::Matrix4f& guess,
                            Eigen::Matrix4f& result) {
  // @todo add filter for source cloud

  return true;
}

template class IcpFast<pcl::PointXYZI>;
}  // namespace registrator
}  // namespace static_map
