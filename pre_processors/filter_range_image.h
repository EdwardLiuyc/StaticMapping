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

#ifndef PRE_PROCESSORS_FILTER_RANGE_IMAGE_H_
#define PRE_PROCESSORS_FILTER_RANGE_IMAGE_H_

#include <libpng/png.h>
#include <algorithm>
#include <map>
#include <memory>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

#include "common/eigen_hash.h"
#include "pre_processors/filter_interface.h"

namespace static_map {
namespace pre_processers {
namespace filter {

class RangeImage : public Interface {
 public:
  struct Pixel {
    float range = 0.;
    int32_t index = -1;  // index in raw point cloud
    uint32_t label = 0;  // label from depth cluster

    Pixel() {}
    Pixel(const float &r, const int32_t &i, uint32_t l = 0)
        : range(r), index(i), label(l) {}
  };
  using Index = Eigen::Vector2i;
  using MapRangeImage = std::unordered_map<Index, Pixel, std::hash<Index>>;
  using MatrixRangeImage = Eigen::MatrixXf;
  using LabelT = uint32_t;

  using LabeledPointType = pcl::PointXYZL;
  using LabeledPointCloudPtr = pcl::PointCloud<LabeledPointType>::Ptr;

  RangeImage();
  ~RangeImage() {}

  PROHIBIT_COPY_AND_ASSIGN(RangeImage);

  std::shared_ptr<Interface> CreateNewInstance() override {
    return std::make_shared<RangeImage>();
  }

  void SetInputCloud(const data::InnerCloudType::Ptr &cloud) override;

  void Filter(const data::InnerCloudType::Ptr &cloud) override;

  void DepthCluster(const LabeledPointCloudPtr &cloud = nullptr);

  void DisplayAllParams() override;

  void ToPng(const float &max_range, const char *filename,
             bool colorful = false);

 protected:
  void LabelOneComponent(LabelT label, Index index);

 private:
  // parameters
  float top_angle_;
  float btm_angle_;
  // fused cloud has the coordinate placed on base_link
  // it is too low for generating range image
  float offset_x_;
  float offset_y_;
  float offset_z_;
  int32_t vertical_line_num_;
  int32_t horizontal_line_num_;

  // inner storages
  MapRangeImage map_image_;
  MatrixRangeImage matrix_image_;

  std::vector<Index> neighbors_;
  const float segmentation_rad_threshold_ = 10 / 180. * M_PI;
  LabelT max_label_;

  std::map<LabelT, std::vector<int> /*index*/> clusters_;
};

}  // namespace filter
}  // namespace pre_processers
}  // namespace static_map

#endif  // PRE_PROCESSORS_FILTER_RANGE_IMAGE_H_
