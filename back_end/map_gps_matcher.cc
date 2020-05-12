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

// stl
#include <fstream>
#include <ostream>
// third party
#include "ceres/problem.h"
#include "glog/logging.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
// local
#include "back_end/map_gps_matcher.h"
#include "common/macro_defines.h"
#include "cost_functions/gps_map_match.h"

namespace static_map {

template <int DIM>
void MapGpsMatcher<DIM>::InsertPositionData(
    const Eigen::VectorNd<DIM>& enu_position,
    const Eigen::VectorNd<DIM>& map_position,
    const Eigen::VectorNd<DIM>& map_direction) {
  const Eigen::VectorNd<DIM> new_enu = enu_position;
  enu_positions_.push_back(new_enu);
  map_positions_.push_back(map_position);
  map_directions_.push_back(map_direction);
}

template <int DIM>
Eigen::Matrix4d MapGpsMatcher<DIM>::RunMatch(bool output_files) {
  if (enu_positions_.empty()) {
    PRINT_WARNING("No positions to run the match.");
    return Eigen::Matrix4d::Identity();
  }

  // step1 get estimate in 2d even if it is a 3d matcher
  // step1.1 get average yaw
  const int corre_size = map_positions_.size();
  const int half = corre_size / 2;
  const double min_distance = 5.;
  int num = 0;
  double average_yaw = 0.;
  for (int i = 0; i < half; ++i) {
    // vector in both coord system
    const Eigen::VectorNd<DIM> vec_map =
        map_positions_[i + half] - map_positions_[i];
    const Eigen::VectorNd<DIM> vec_enu =
        enu_positions_[i + half] - enu_positions_[i];
    // shoule not be to close from start to end
    if (vec_map.template topRows(2).norm() < min_distance) {
      continue;
    }

    num++;
    // yaw
    double alpha_map = std::atan2(vec_map[1], vec_map[0]);
    double alpha_enu = std::atan2(vec_enu[1], vec_enu[0]);
    double diff = alpha_map - alpha_enu;
    if (diff < 0.) {
      diff += (M_PI * 2);
    }
    CHECK(diff >= 0. && diff <= M_PI * 2);
    average_yaw += diff;
  }
  if (num > 0) {
    average_yaw /= num;
  } else {
    PRINT_ERROR("No suitable data for the calculation.");
    return Eigen::Matrix4d::Identity();
  }

  // we esitmate only yaw as the rotation
  // so in 2d or 3d, we init the rotation in the same way
  Eigen::Matrix<double, DIM, DIM> rotation =
      Eigen::Matrix<double, DIM, DIM>::Identity();
  rotation(0, 0) = std::cos(average_yaw);
  rotation(0, 1) = -std::sin(average_yaw);
  rotation(1, 0) = std::sin(average_yaw);
  rotation(1, 1) = std::cos(average_yaw);

  // step1.2, get the average translation
  Eigen::VectorNd<DIM> translation = Eigen::VectorNd<DIM>::Zero();
  for (int i = 0; i < half; ++i) {
    const Eigen::VectorNd<DIM> mid_point_submap =
        (map_positions_[i + half] + map_positions_[i]) / 2;
    // remove the rotation
    const Eigen::VectorNd<DIM> mid_point_enu =
        rotation * (enu_positions_[i + half] + enu_positions_[i]) / 2;
    translation += (mid_point_submap - mid_point_enu);
  }
  translation /= static_cast<double>(half);

  auto vec_to_point = [&](const Eigen::VectorNd<DIM>& v) {
    pcl::PointXYZI point;
    point.x = v[0];
    point.y = v[1];
    if (kDimValue == 2) {
      point.z = 0.;
    } else {
      point.z = v[2];
    }
    return point;
  };

  auto output_compare_file = [&](const std::string& filename,
                                 const Eigen::VectorNd<DIM>& t,
                                 const Eigen::Matrix<double, DIM, DIM>& r) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr path_compare_cloud(
        new pcl::PointCloud<pcl::PointXYZI>);
    for (int i = 0; i < corre_size; ++i) {
      Eigen::VectorNd<DIM> enu_in_map = r * enu_positions_[i] + t;
      pcl::PointXYZI point_pcl_map = vec_to_point(map_positions_[i]);
      pcl::PointXYZI point_pcl_enu = vec_to_point(enu_in_map);
      point_pcl_map.intensity = 1.;
      point_pcl_enu.intensity = 2.;
      path_compare_cloud->push_back(point_pcl_map);
      path_compare_cloud->push_back(point_pcl_enu);
    }
    if (!path_compare_cloud->empty()) {
      pcl::io::savePCDFileBinaryCompressed(filename, *path_compare_cloud);
    }
  };

  if (output_files) {
    // output the path in pcd using the initial estimate
    output_compare_file(output_file_path_ + "enu_before.pcd", translation,
                        rotation);
  }

  // step2 optimizing
  const int max_iter_num = 100;
  if (kDimValue == 2) {
    // step2, use svd to solve the problem
    // just like point-line icp
    // iteration loop
    for (int iter = 0; iter < max_iter_num; ++iter) {
      Eigen::MatrixXd A(corre_size, 3);
      Eigen::MatrixXd b(corre_size, 1);
      for (int i = 0; i < corre_size; ++i) {
        Eigen::VectorNd<DIM> p_u = enu_positions_[i];
        Eigen::VectorNd<DIM> p_m = map_positions_[i];
        // Get the normal vector to normal
        Eigen::VectorNd<DIM> n_m = Eigen::VectorNd<DIM>::Zero();
        n_m << -map_directions_[i](1, 0), map_directions_[i](0, 0);
        n_m.normalize();
        Eigen::VectorNd<DIM> opt_p_u = rotation * p_u + translation;

        A.row(i) << (opt_p_u(0, 0) * n_m(1, 0) - opt_p_u(1, 0) * n_m(0, 0)),
            n_m(0, 0), n_m(1, 0);
        b(i, 0) = n_m(0, 0) * (p_m(0, 0) - opt_p_u(0, 0)) +
                  n_m(1, 0) * (p_m(1, 0) - opt_p_u(1, 0));
      }
      Eigen::MatrixXd At = A.transpose();
      Eigen::MatrixXd matAtA = At * A;
      Eigen::MatrixXd matAtb = At * b;
      Eigen::VectorXd result = A.colPivHouseholderQr().solve(b);

      Eigen::Matrix<double, DIM, DIM> delta_r;
      delta_r << std::cos(result[0]), -std::sin(result[0]), std::sin(result[0]),
          std::cos(result[0]);
      rotation = delta_r * rotation;

      Eigen::VectorNd<DIM> delta_t;
      delta_t << result[1], result[2];
      translation = delta_r * translation + delta_t;

      const double delta_trans = delta_t.norm();
      const double delta_angle =
          (delta_r - Eigen::Matrix<double, DIM, DIM>::Identity()).norm();
      if (delta_trans < 1.e-10 && delta_angle < 1.e-10) {
        PRINT_DEBUG_FMT("finish in %d iters", iter);
        break;
      }
    }
  } else if (kDimValue == 3) {
    Eigen::Matrix3d temp_r;
    temp_r << rotation;
    const Eigen::Vector3d eulers = common::RotationMatrixToEulerAngles(temp_r);
    // init estimate
    double t[3] = {translation[0], translation[1], translation[2]};
    double r[3] = {eulers[0], eulers[1], eulers[2]};
    ceres::Problem problem;
    for (int i = 0; i < corre_size; ++i) {
      Eigen::Vector3d map_position, enu_position, map_direction;
      map_position << map_positions_[i][0], map_positions_[i][1],
          map_positions_[i][2];
      enu_position << enu_positions_[i][0], enu_positions_[i][1],
          enu_positions_[i][2];
      map_direction << map_directions_[i][0], map_directions_[i][1],
          map_directions_[i][2];
      auto cost_function = cost_functions::GpsEnuToMap3D::Create(
          map_position, enu_position, map_direction);
      problem.AddResidualBlock(cost_function, new ceres::HuberLoss(1.0), t, r);
    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = 100;
    options.num_threads = 8;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // get result
    translation << t[0], t[1], t[2];
    rotation =
        common::EulerAnglesToRotationMatrix(Eigen::Vector3d(r[0], r[1], r[2]))
            .block<DIM, DIM>(0, 0);
  } else {
    LOG(FATAL) << "Wrong Dim = " << kDimValue << std::endl;
  }

  if (output_files) {
    output_compare_file(output_file_path_ + "enu.pcd", translation, rotation);
  }

  Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
  result.block<DIM, DIM>(0, 0) = rotation.inverse();
  result.block<DIM, 1>(0, 3) = -rotation.inverse() * translation;
  if (output_files) {
    std::ofstream enu_file(output_file_path_ + "enu.txt");
    if (enu_file.is_open()) {
      enu_file << "-x " << std::to_string(result(0, 3)) << " -y "
               << std::to_string(result(1, 3)) << "\n";
      enu_file.close();
    } else {
      PRINT_ERROR("Failed to open file.");
    }
  }
  return result;
}

template <int DIM>
void MapGpsMatcher<DIM>::OutputError(const Eigen::Matrix4d& result,
                                     const std::string& filename) {
  CHECK(!filename.empty());
  std::ofstream error_file(filename);
  if (!error_file.is_open()) {
    PRINT_WARNING("File cannot open!");
    return;
  }
  std::string error_content;
  const Eigen::Matrix<double, DIM, DIM> rotation = result.block<DIM, DIM>(0, 0);
  const Eigen::VectorNd<DIM> translation = result.block<DIM, 1>(0, 3);
  for (int i = 0; i < map_positions_.size(); ++i) {
    Eigen::VectorNd<DIM> map_position = map_positions_[i];
    Eigen::VectorNd<DIM> enu_in_map =
        rotation.inverse() * (enu_positions_[i] - translation);
    Eigen::VectorNd<DIM> map_direction = map_directions_[i];
    // ingore the distance in Z axis
    if (kDimValue == 3) {
      map_position[2] = 0.;
      map_direction[2] = 0.;
      enu_in_map[2] = 0.;
    }
    error_content += std::to_string(
        common::DistanceToLine<DIM>(enu_in_map, map_position, map_direction));
    error_content += "\n";
  }
  if (error_file.is_open()) {
    error_file << error_content;
    error_file.close();
  }
}

template class MapGpsMatcher<2>;
template class MapGpsMatcher<3>;

}  // namespace static_map
