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

template <typename PointT>
class GroundRemoval2 : public Interface<PointT> {
 public:
  USE_POINTCLOUD;

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
  GroundRemoval2()
      : Interface<PointT>(),
        r_max_(100.),
        r_min_(1.),
        bin_num_(200),
        segment_num_(180),
        start_ground_height_(-0.25),
        long_line_threshold_(1.0),
        max_long_line_height_(0.1),
        max_start_height_(0.2),
        max_error_(0.05),
        max_slope_(std::tan(M_PI / 12.)),
        max_b_(0.1),
        max_dist_to_line_(0.05),
        search_angle_(10.) /* degree */
        ,
        thread_num_(4) {
    INIT_INNER_PARAM(Interface<PointT>::kFloatParam, 0, "r_max", r_max_);
    INIT_INNER_PARAM(Interface<PointT>::kFloatParam, 1, "r_min", r_min_);
    INIT_INNER_PARAM(Interface<PointT>::kFloatParam, 2, "start_ground_height",
                     start_ground_height_);
    INIT_INNER_PARAM(Interface<PointT>::kFloatParam, 3, "long_line_threshold",
                     long_line_threshold_);
    INIT_INNER_PARAM(Interface<PointT>::kFloatParam, 4, "max_long_line_height",
                     max_long_line_height_);
    INIT_INNER_PARAM(Interface<PointT>::kFloatParam, 5, "max_start_height",
                     max_start_height_);
    INIT_INNER_PARAM(Interface<PointT>::kFloatParam, 6, "max_error",
                     max_error_);
    INIT_INNER_PARAM(Interface<PointT>::kFloatParam, 7, "max_slope",
                     max_slope_);
    INIT_INNER_PARAM(Interface<PointT>::kFloatParam, 8, "max_b", max_b_);
    INIT_INNER_PARAM(Interface<PointT>::kFloatParam, 9, "max_dist_to_line",
                     max_dist_to_line_);
    INIT_INNER_PARAM(Interface<PointT>::kFloatParam, 10, "search_angle",
                     search_angle_);

    // int32_t params
    INIT_INNER_PARAM(Interface<PointT>::kInt32Param, 0, "bin_num", bin_num_);
    INIT_INNER_PARAM(Interface<PointT>::kInt32Param, 1, "segment_num",
                     segment_num_);
    INIT_INNER_PARAM(Interface<PointT>::kInt32Param, 2, "thread_num",
                     thread_num_);
  }
  ~GroundRemoval2() {}

  GroundRemoval2(const GroundRemoval2&) = delete;
  GroundRemoval2& operator=(const GroundRemoval2&) = delete;

  std::shared_ptr<Interface<PointT>> CreateNewInstance() override {
    return std::make_shared<GroundRemoval2<PointT>>();
  }

  void SetInputCloud(const PointCloudPtr& cloud) override {
    this->inliers_.clear();
    this->outliers_.clear();
    if (cloud == nullptr || cloud->empty()) {
      LOG(WARNING) << "cloud empty, do nothing!" << std::endl;
      this->inner_cloud_ = nullptr;
      return;
    }
    // step1 initialise
    this->inner_cloud_ = cloud;
    // init the grids
    grids_.clear();
    Grid default_grid_value;
    default_grid_value.min_z_point[0] = 1.e6;
    default_grid_value.min_z_point[1] = 1.e6;
    grids_.resize(bin_num_ * segment_num_, default_grid_value);

    const float double_pi = M_PI * 2;
    const float delta_alpha = double_pi / segment_num_;
    const float delta_bin = (r_max_ - r_min_) / bin_num_;

    const int size = this->inner_cloud_->size();
    struct InnerPoint {
      int s_index;
      int b_index;
      Point point;  // d, z
      int cloud_index;
    };
    std::vector<InnerPoint> inner_points;
    inner_points.resize(size);

// step2 insert the cloud into grids
#if defined _OPENMP
#pragma omp parallel for num_threads(LOCAL_OMP_THREADS_NUM)
#endif
    for (int i = 0; i < size; ++i) {
      auto& point = this->inner_cloud_->points[i];
      float range = std::sqrt(point.x * point.x + point.y * point.y);
      if (range < r_min_ && range > r_max_) {
        inner_points[i].s_index = -1;
        inner_points[i].b_index = -1;
      } else {
        // index
        float rad = std::atan2(point.y, point.x);
        if (rad < 0.) {
          rad += double_pi;
        }
        int32_t s_index = rad / delta_alpha;
        int32_t b_index = (range - r_min_) / delta_bin;
        // clamp the indices
        if (b_index >= bin_num_) {
          b_index = bin_num_ - 1;
        } else if (b_index < 0) {
          b_index = 0;
        }
        if (s_index >= segment_num_) {
          s_index = segment_num_ - 1;
        } else if (s_index < 0) {
          s_index = 0;
        }
        inner_points[i].s_index = s_index;
        inner_points[i].b_index = b_index;
        inner_points[i].cloud_index = i;
        inner_points[i].point[0] = range;
        inner_points[i].point[1] = point.z;
      }
    }

    for (auto& inner_point : inner_points) {
      if (inner_point.s_index < 0) {
        continue;
      }
      auto& grid =
          grids_.at(GridIndex(inner_point.s_index, inner_point.b_index));
      if (grid.points.empty() || inner_point.point[1] < grid.min_z_point[1]) {
        grid.min_z_point[0] = inner_point.point[0];
        grid.min_z_point[1] = inner_point.point[1];
      }
      if (inner_point.point[1] <= grid.min_z_point[1] + 0.5) {
        grid.points.push_back(
            std::make_pair(inner_point.cloud_index,
                           Point(inner_point.point[0], inner_point.point[1])));
      }
    }
  }

  void Filter(const PointCloudPtr& cloud) override {
    if (!cloud || !Interface<PointT>::inner_cloud_) {
      LOG(WARNING) << "nullptr cloud, do nothing!" << std::endl;
      return;
    }
    // step1 prepare
    this->FilterPrepare(cloud);
    std::vector<Segment> segments;
    segments.resize(segment_num_);
    FitSegments(&segments);

    // step2 cluster ( ground )
    auto cloud_size = this->inner_cloud_->size();
    std::vector<uint8_t> is_outlier(cloud_size, 0);
    ClusterGround(segments, &is_outlier);

    // step3 manage inliers and outliers
    for (int i = 0; i < cloud_size; ++i) {
      if (is_outlier[i]) {
        this->outliers_.push_back(i);
      } else {
        this->inliers_.push_back(i);
        cloud->push_back(this->inner_cloud_->points[i]);
      }
    }
    // no need to sort inliers and outliers( already in-order )
  }

  void DisplayAllParams() override {
    PARAM_INFO(r_max_);
    PARAM_INFO(r_min_);
    PARAM_INFO(start_ground_height_);
    PARAM_INFO(long_line_threshold_);
    PARAM_INFO(max_long_line_height_);
    PARAM_INFO(max_start_height_);
    PARAM_INFO(max_error_);
    PARAM_INFO(max_slope_);
    PARAM_INFO(max_b_);
    PARAM_INFO(max_dist_to_line_);
    PARAM_INFO(search_angle_);

    // int32_t params
    PARAM_INFO(bin_num_);
    PARAM_INFO(segment_num_);
    PARAM_INFO(thread_num_);
  }

 protected:
  Segment FitLines(const int32_t& seg_index) {
    CHECK(seg_index >= 0 && seg_index < segment_num_);

    Segment segment;
    int start_index = 0;
    for (start_index = 0; start_index < bin_num_; ++start_index) {
      if (!grids_[GridIndex(seg_index, start_index)].points.empty()) {
        break;
      }
    }
    if (start_index >= bin_num_ - 1) {
      return std::move(segment);
    }

    std::vector<Point> current_line_points;
    current_line_points.push_back(
        grids_[GridIndex(seg_index, start_index)].min_z_point);

    LocalLine current_line;
    bool is_long_line = false;
    float ground_height = start_ground_height_;
    for (int i = start_index + 1; i < bin_num_; ++i) {
      auto& grid = grids_.at(GridIndex(seg_index, i));
      if (grid.points.empty()) {
        continue;
      }

      auto& current_point = grid.min_z_point;
      if (current_point[0] - current_line_points.back()[0] >=
          long_line_threshold_) {
        is_long_line = true;
      }

      float expected_z = std::numeric_limits<float>::max();
      if (is_long_line && current_line_points.size() > 2) {
        expected_z = current_line.m * current_point[0] + current_line.b;
      }
      if (current_line_points.size() >= 2) {
        current_line_points.push_back(current_point);
        current_line = FitLocalLine(current_line_points);
        auto error = GetMaxError(current_line_points, current_line);
        if (error > max_error_ || std::fabs(current_line.m) > max_slope_ ||
            // std::fabs(current_line.b - ground_height ) > max_b_ ||
            (is_long_line && std::fabs(expected_z - current_point[1]) >
                                 max_long_line_height_)) {
          current_line_points.pop_back();
          if (current_line_points.size() >= 3) {
            auto new_line = FitLocalLine(current_line_points);
            segment.push_back(LocalLineToLine(new_line, current_line_points));

            // update ground height
            ground_height =
                new_line.m * current_line_points.back()[0] + new_line.b;
          }
          // start a new line
          is_long_line = false;
          current_line_points.erase(current_line_points.begin(),
                                    --current_line_points.end());
          --i;
        }
      } else {
        if (!is_long_line && std::fabs(current_line_points.back()[1] -
                                       ground_height) < max_start_height_) {
          current_line_points.push_back(current_point);
        } else {
          // start a new line
          current_line_points.clear();
          current_line_points.push_back(current_point);
        }
      }
    }
    if (current_line_points.size() > 2) {
      auto new_line = FitLocalLine(current_line_points);
      segment.push_back(LocalLineToLine(new_line, current_line_points));
    }

    return std::move(segment);
  }

  void FitSegments(std::vector<Segment>* const segments) {
    auto calculate_in_one_thread = [&](const int& index) {
      (*segments)[index] = FitLines(index);
    };
#if defined _OPENMP
// openmp version
#pragma omp parallel for num_threads(LOCAL_OMP_THREADS_NUM)
    for (int i = 0; i < segment_num_; ++i) {
      calculate_in_one_thread(i);
    }
#else
    // thread pool version
    common::ThreadPool pool(thread_num_);
    for (int i = 0; i < segment_num_; ++i) {
      pool.enqueue(calculate_in_one_thread, i);
    }
#endif
  }

  void ClusterGround(const std::vector<Segment>& segments,
                     std::vector<uint8_t>* const is_outlier) {
    CHECK(is_outlier);
    const float delta_alpha = M_PI * 2 / segment_num_;
    int search_max_step = search_angle_ / 180. * M_PI / delta_alpha;
    std::vector<int> segment_index_candidate;
    for (int i = search_max_step; i > 0; --i) {
      segment_index_candidate.push_back(i);
      segment_index_candidate.push_back(-i);
    }

    // using thread pool to accelerate
    auto pool = std::make_shared<common::ThreadPool>(thread_num_);
    auto calculate_in_one_thread = [&](const int& s /* seg_index */) {
      for (int b = 0; b < bin_num_; ++b) {
        auto grid_index = GridIndex(s, b);
        auto& grid = grids_[grid_index];
        if (grid.points.empty()) {
          continue;
        }

        for (auto& index_point : grid.points) {
          auto& point = index_point.second;
          auto distance = VerticalDistanceToSegment(point, segments.at(s));
          if (distance < 0.) {
            // getting a distance < 0 means that you did not
            // find a line match the point
            // you can try find the line in close segment
            for (auto i : segment_index_candidate) {
              int can_seg_index = s + i;
              if (can_seg_index < 0) {
                can_seg_index += segment_num_;
              } else if (can_seg_index >= segment_num_) {
                can_seg_index -= segment_num_;
              }

              distance =
                  VerticalDistanceToSegment(point, segments.at(can_seg_index));
              if (distance > 0.) {
                break;
              }
            }
          }
          if (distance > 0. && distance <= max_dist_to_line_) {
            // this is a ground removal filter
            // so, if you found a point on ground
            // you should add it into outliers
            (*is_outlier)[index_point.first] = 1;
          }
        }  // end loop in on grid
      }    // loop for bins
    };

    for (int i = 0; i < segment_num_; ++i) {
      pool->enqueue(calculate_in_one_thread, i);
    }
    // reset the shared pointer to destroy the thread pool
    // the destrcutor will wait for all threads then return
    pool.reset();
  }

  float VerticalDistanceToSegment(const Point& point, const Segment& seg) {
    const float margin = 0.1;
    float distance = -1.;
    for (auto& line : seg) {
      CHECK(line.start[0] < line.end[0]);

      if (line.start[0] - margin < point[0] &&
          line.end[0] + margin > point[0]) {
        float delta_z = line.end[1] - line.start[1];
        float delta_d = line.end[0] - line.start[0];
        float expected_z =
            (point[0] - line.start[0]) / delta_d * delta_z + line.start[1];
        distance = std::fabs(point[1] - expected_z);
      }
    }
    return distance;
  }

  inline LocalLine FitLocalLine(const std::vector<Point>& points) {
    LocalLine line_result;
    auto point_num = points.size();
    Eigen::MatrixXd X(point_num, 2);
    Eigen::VectorXd Y(point_num);
    for (int i = 0; i < point_num; ++i) {
      X(i, 0) = points[i][0];
      X(i, 1) = 1;
      Y(i) = points[i][1];
    }
    Eigen::VectorXd result = X.colPivHouseholderQr().solve(Y);
    line_result.m = result(0);
    line_result.b = result(1);
    return line_result;
  }

  inline float GetMaxError(const std::vector<Point>& points,
                           const LocalLine& line) {
    float max_error = 0.;
    for (auto& point : points) {
      float error = std::fabs(line.m * point[0] + line.b - point[1]);
      if (error > max_error) {
        max_error = error;
      }
    }
    return max_error;
  }

  inline Line LocalLineToLine(const LocalLine& local_line,
                              const std::vector<Point>& line_points) {
    Line line;
    auto start_d = line_points.front()[0];
    auto end_d = line_points.back()[0];
    line.start[0] = start_d;
    line.start[1] = local_line.m * start_d + local_line.b;
    line.end[0] = end_d;
    line.end[1] = local_line.m * end_d + local_line.b;

    return line;
  }

  inline int32_t GridIndex(int32_t seg_index, int32_t bin_index) {
    return seg_index * bin_num_ + bin_index;
  }
};

}  // namespace filter
}  // namespace pre_processers
}  // namespace static_map

#endif  // PRE_PROCESSORS_FILTER_GROUND_REMOVAL2_H_
