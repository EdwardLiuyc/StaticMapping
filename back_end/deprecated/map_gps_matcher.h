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

#ifndef BACK_END_DEPRECATED_MAP_GPS_MATCHER_H_
#define BACK_END_DEPRECATED_MAP_GPS_MATCHER_H_

#include <string>
#include <vector>
#include "Eigen/Eigen"
// local
#include "common/math.h"

namespace static_map {
/*
 * @class MapGpsMatcher
 * @brief matcher for matcher path and gps(enu) path (for bost 2d and 3d)
 *
 * @notice this class is in no use anymore since the map&gps(enu) matching
 * has been done in isam_optimizer
 */
template <int DIM>
class MapGpsMatcher {
 public:
  MapGpsMatcher() : output_file_path_("") {}
  ~MapGpsMatcher() {}

  enum { kDimValue = DIM };

  MapGpsMatcher(const MapGpsMatcher&) = delete;
  MapGpsMatcher& operator=(const MapGpsMatcher&) = delete;

  /// @brief insert data for the matching
  void InsertPositionData(const Eigen::VectorNd<DIM>& enu_position,
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
  std::vector<Eigen::VectorNd<DIM>> enu_positions_;
  std::vector<Eigen::VectorNd<DIM>> map_positions_;
  std::vector<Eigen::VectorNd<DIM>> map_directions_;

  std::string output_file_path_;
};

}  // namespace static_map

#endif  // BACK_END_DEPRECATED_MAP_GPS_MATCHER_H_
