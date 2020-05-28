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

#include "builder/simple_frame.h"
#include "builder/msg_conversion.h"
#include "nabo/nabo.h"

namespace static_map {

// template <typename PointT>
// InlierPointPairs GetPointPairs(
//     const std::shared_ptr<SimpleFrame<PointT>>& first_frame,
//     const std::shared_ptr<SimpleFrame<PointT>>& last_frame, double
//     max_distance, int sample) {
//   CHECK(!first_frame->Cloud()->points.empty());
//   CHECK(!last_frame->Cloud()->points.empty());

//   Eigen::Matrix4f delta =
//       first_frame->GlobalPose().inverse() * last_frame->GlobalPose();

//   using PointCloudType = pcl::PointCloud<PointT>;
//   using PointCloudPtr = typename PointCloudType::Ptr;
//   PointCloudPtr transformed_cloud(new PointCloudType);
//   pcl::transformPointCloud(*last_frame->Cloud(), *transformed_cloud, delta);

//   // typedef PointMatcher<float> PM;
//   // typedef PM::DataPoints DP;
//   typedef PointMatcher<float> PM;
//   PM::DataPoints ref_cloud, read_cloud;

//   if (sample > 1) {
//     PointCloudPtr sampled_ref_cloud(new PointCloudType);
//     PointCloudPtr sampled_read_cloud(new PointCloudType);

//     for (int i = 0; i < first_frame->Cloud()->size(); i += sample) {
//       sampled_ref_cloud->push_back(first_frame->Cloud()->points[i]);
//     }
//     for (int i = 0; i < transformed_cloud->size(); i += sample) {
//       sampled_read_cloud->push_back(transformed_cloud->points[i]);
//     }

//     read_cloud = sensors::pclPointCloudToLibPointMatcherPoints<PointT>(
//         sampled_read_cloud);
//     ref_cloud = sensors::pclPointCloudToLibPointMatcherPoints<PointT>(
//         sampled_ref_cloud);

//     sampled_ref_cloud.reset();
//     sampled_read_cloud.reset();
//   } else {
//     read_cloud = sensors::pclPointCloudToLibPointMatcherPoints<PointT>(
//         transformed_cloud);
//     ref_cloud = sensors::pclPointCloudToLibPointMatcherPoints<PointT>(
//         first_frame->Cloud());
//   }
//   transformed_cloud.reset();

//   typedef typename Nabo::NearestNeighbourSearch<float> NNS;
//   NNS* kdtree;

//   const int dim = 3;
//   kdtree = NNS::create(ref_cloud.features, dim, NNS::KDTREE_LINEAR_HEAP);
//   CHECK(kdtree != NULL);

//   const int knn = 1;
//   PM::Matches::Dists dists(knn, read_cloud.getNbPoints());
//   PM::Matches::Ids ids(knn, read_cloud.getNbPoints());
//   kdtree->knn(read_cloud.features.topRows(3), ids, dists, knn);

//   // Create point pairs
//   InlierPointPairs pairs;
//   int pair_num = 0;
//   const double squared_max_distance = max_distance * max_distance;
//   for (int i = 0; i < read_cloud.getNbPoints(); ++i) {
//     if (dists(0, i) <= squared_max_distance) {
//       pair_num++;
//     }
//   }
//   pairs.pairs_num = pair_num;
//   pairs.read_points.resize(pair_num, 3);
//   pairs.ref_points.resize(pair_num, 3);
//   int pair_index = 0;

//   for (int i = 0; i < read_cloud.getNbPoints(); ++i) {
//     if (dists(0, i) <= squared_max_distance) {
//       pairs.read_points.row(pair_index) =
//           read_cloud.features.col(i).topRows(3).transpose();
//       pairs.ref_points.row(pair_index) =
//           ref_cloud.features.col(ids(0, i)).topRows(3).transpose();
//       pair_index++;
//     }
//   }

//   delete kdtree;
//   CHECK_EQ(pairs.pairs_num, pairs.read_points.rows());
//   return pairs;
// }

// template InlierPointPairs GetPointPairs<pcl::PointXYZI>(
//     const std::shared_ptr<SimpleFrame<pcl::PointXYZI>>& first_frame,
//     const std::shared_ptr<SimpleFrame<pcl::PointXYZI>>& last_frame,
//     double max_distance, int sample);

}  // namespace static_map
