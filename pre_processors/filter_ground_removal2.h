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

#ifndef PRE_PROCESSORS_FILTER_GROUND_REMOVAL2_H_
#define PRE_PROCESSORS_FILTER_GROUND_REMOVAL2_H_

#include <limits>
#include <memory>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "common/simple_thread_pool.h"
#include "pre_processors/filter_interface.h"

// implementation of paper
// "Fast Segmentation of 3D Pointcloud for Ground Vehicles", 2010
namespace static_map {
namespace pre_processers {
namespace filter {

class GroundRemoval2 : public Interface {
 public:
  using Point = Eigen::Vector2f;  // d, z
  struct Grid {
    Point min_z_point;
    std::vector<std::pair<int, Point>> points;
  };

  struct Line {
    Point start, end;
  };
  struct LocalLine {
    float m = 0.;
    float b = 0.;
  };

  using Segment = std::vector<Line>;

 private:
  // parameters to insert the cloud into bins
  float r_max_;
  float r_min_;
  int32_t bin_num_;
  int32_t segment_num_;

  // line fitting parameters
  float start_ground_height_;
  float long_line_threshold_;
  float max_long_line_height_;
  float max_start_height_;
  // max error for line fitting
  float max_error_;
  // y = mx + b ( max m and max b)
  float max_slope_;
  float max_b_;

  // cluster parameters
  // maximum vertical distance of point to line to be considered ground
  float max_dist_to_line_;
  // search other segments to find matched line
  float search_angle_;  // degree

  int32_t thread_num_;

  // point to a 2d array
  std::vector<Grid> grids_;

 public:
  GroundRemoval2();
  ~GroundRemoval2() {}

  PROHIBIT_COPY_AND_ASSIGN(GroundRemoval2);

  std::shared_ptr<Interface> CreateNewInstance() override {
    return std::make_shared<GroundRemoval2>();
  }

  void SetInputCloud(const data::InnerCloudType::Ptr& cloud) override;

  void Filter(const data::InnerCloudType::Ptr& cloud) override;

  void DisplayAllParams() override;

 protected:
  Segment FitLines(const int32_t& seg_index);

  void FitSegments(std::vector<Segment>* const segments);

  void ClusterGround(const std::vector<Segment>& segments,
                     std::vector<uint8_t>* const is_outlier);

  float VerticalDistanceToSegment(const Point& point, const Segment& seg);

  inline int32_t GridIndex(int32_t seg_index, int32_t bin_index) {
    return seg_index * bin_num_ + bin_index;
  }
};

}  // namespace filter
}  // namespace pre_processers
}  // namespace static_map

#endif  // PRE_PROCESSORS_FILTER_GROUND_REMOVAL2_H_
