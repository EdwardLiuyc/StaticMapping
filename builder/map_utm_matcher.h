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

#ifndef BUILDER_MAP_UTM_MATCHER_H_
#define BUILDER_MAP_UTM_MATCHER_H_

#include <string>
#include <vector>
#include "Eigen/Eigen"
// local
#include "common/math.h"

namespace static_map {
/*
 * @class MapUtmMatcher
 * @brief matcher for matcher path and utm path (for bost 2d and 3d)
 *
 * @notice this class is in no use anymore since the map&utm matching
 * has been done in isam_optimizer
 */
template <int DIM>
class MapUtmMatcher {
 public:
  MapUtmMatcher() : output_file_path_("") {}
  ~MapUtmMatcher() {}

  enum { kDimValue = DIM };

  MapUtmMatcher(const MapUtmMatcher&) = delete;
  MapUtmMatcher& operator=(const MapUtmMatcher&) = delete;

  /// @brief insert data for the matching
  void InsertPositionData(const Eigen::VectorNd<DIM>& utm_position,
                          const Eigen::VectorNd<DIM>& map_position,
                          const Eigen::VectorNd<DIM>& map_direction);
  /*
   * @brief run the match with inserted data
   * @return even if we match 2d pathes, we still return a 4*4 matrix
   * if DIM == 2
   * | cos_theta  -sin_theta  0    tx |
   * | sin_theta  cos_theta   0    ty |
   * |   0          0         1    0  |
   * |   0          0         0    1  |
   * else if DIM = 3
   * |  R   T  |
   * |  0   1  |
   * */
  Eigen::Matrix4d RunMatch(bool output_files = true);
  /// @brief if we need to output files when matching, we will output them to
  /// this path
  void SetOutputPath(const std::string& path) { output_file_path_ = path; }
  /// @brief output the error for all inserted data to a text file
  void OutputError(const Eigen::Matrix4d& result, const std::string& filename);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // inserted data
  std::vector<Eigen::VectorNd<DIM>> utm_positions_;
  std::vector<Eigen::VectorNd<DIM>> map_positions_;
  std::vector<Eigen::VectorNd<DIM>> map_directions_;

  std::string output_file_path_;
};

}  // namespace static_map

#endif  // BUILDER_MAP_UTM_MATCHER_H_
