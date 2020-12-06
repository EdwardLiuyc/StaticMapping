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

#include "pre_processors/filter_range_image.h"

namespace static_map {
namespace pre_processers {
namespace filter {

RangeImage::RangeImage()
    : Interface(),
      top_angle_(30.),
      btm_angle_(-15.),
      offset_x_(0.f),
      offset_y_(0.f),
      offset_z_(0.f),
      vertical_line_num_(40),
      horizontal_line_num_(1800) {
  neighbors_.push_back(Index(0, 1));
  neighbors_.push_back(Index(0, -1));
  neighbors_.push_back(Index(0, 2));
  neighbors_.push_back(Index(0, -2));
  neighbors_.push_back(Index(1, 0));
  neighbors_.push_back(Index(-1, 0));

  // float params
  INIT_FLOAT_PARAM("top_angle", top_angle_);
  INIT_FLOAT_PARAM("btm_angle", btm_angle_);
  INIT_FLOAT_PARAM("offset_x", offset_x_);
  INIT_FLOAT_PARAM("offset_y", offset_y_);
  INIT_FLOAT_PARAM("offset_z", offset_z_);
  // int32_t params
  INIT_INT32_PARAM("vertical_line_num", vertical_line_num_);
  INIT_INT32_PARAM("horizontal_line_num", horizontal_line_num_);
}

void RangeImage::DisplayAllParams() {
  PARAM_INFO(top_angle_);
  PARAM_INFO(btm_angle_);
  PARAM_INFO(offset_x_);
  PARAM_INFO(offset_y_);
  PARAM_INFO(offset_z_);
  PARAM_INFO(vertical_line_num_);
  PARAM_INFO(horizontal_line_num_);
}

void RangeImage::SetInputCloud(const data::InnerCloudType::Ptr &cloud) {
  if (cloud == nullptr || cloud->points.empty()) {
    LOG(WARNING) << "cloud empty, do nothing!" << std::endl;
    this->inner_cloud_ = nullptr;
    return;
  }
  if (vertical_line_num_ <= 0 || horizontal_line_num_ <= 0) {
    return;
  }

  this->inner_cloud_ = cloud;
  map_image_.clear();
  matrix_image_.resize(vertical_line_num_, horizontal_line_num_);
  matrix_image_.setZero();
}

void RangeImage::Filter(const data::InnerCloudType::Ptr &cloud) {
  if (!cloud || !Interface::inner_cloud_) {
    LOG(WARNING) << "nullptr cloud, do nothing!" << std::endl;
    return;
  }

  this->FilterPrepare(cloud);
  const float image_horizontal_res =
      M_PI * 2 / static_cast<float>(horizontal_line_num_);
  const float image_vertical_res = (top_angle_ - btm_angle_) /
                                   static_cast<float>(vertical_line_num_) /
                                   180.f * M_PI;

  int row_index = 0;
  int col_index = 0;
  float vertical_rad = 0.;
  float horizontal_rad = 0.;
  float range = 0.;
  const auto &input_cloud = this->inner_cloud_;
  for (int i = 0; i < input_cloud->points.size(); ++i) {
    auto point = input_cloud->points[i];
    point.x += offset_x_;
    point.y += offset_y_;
    point.z += offset_z_;
    float distance_in_xy = std::sqrt(point.x * point.x + point.y * point.y);
    if (distance_in_xy < 0.01f) {
      this->outliers_.push_back(i);
      continue;
    }
    vertical_rad = std::atan2(point.z, distance_in_xy);
    row_index = (vertical_rad - btm_angle_ / 180.f * M_PI) / image_vertical_res;
    if (row_index < 0 || row_index >= vertical_line_num_) {
      this->outliers_.push_back(i);
      continue;
    }
    horizontal_rad = std::atan2(point.y, point.x);
    if (horizontal_rad < 0.f) {
      horizontal_rad += M_PI * 2;
    }
    col_index = std::lround(horizontal_rad / image_horizontal_res);
    if (col_index >= horizontal_line_num_) col_index -= horizontal_line_num_;
    if (col_index < 0 || col_index >= horizontal_line_num_) {
      this->outliers_.push_back(i);
      continue;
    }

    if (matrix_image_(row_index, col_index) < 1.e-6) {
      range =
          std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
      matrix_image_(row_index, col_index) = range;
      map_image_[Index(row_index, col_index)] = Pixel(range, i);
      this->inliers_.push_back(i);
    } else {
      this->outliers_.push_back(i);
    }
  }
  for (auto &i : this->inliers_) {
    cloud->points.push_back(input_cloud->points[i]);
  }
}

void RangeImage::DepthCluster(const LabeledPointCloudPtr &cloud) {
  CHECK(this->inner_cloud_ != nullptr);
  LabelT label = 1;
  for (int row = 0; row < vertical_line_num_; ++row) {
    for (int col = 0; col < horizontal_line_num_; ++col) {
      if (matrix_image_(row, col) < 1.e-6) {
        continue;
      }
      if (map_image_.at(Index(row, col)).label > 0) {
        continue;
      }
      LabelOneComponent(label, Index(row, col));
      label++;
      max_label_ = label;
    }
  }
  if (cloud) {
    for (auto &cluster : clusters_) {
      auto &indices = cluster.second;
      for (auto i : indices) {
        LabeledPointType point;
        point.x = this->inner_cloud_->points[i].x;
        point.y = this->inner_cloud_->points[i].y;
        point.z = this->inner_cloud_->points[i].z;
        point.label = cluster.first;

        cloud->push_back(point);
      }
    }
  }
}

void RangeImage::ToPng(const float &max_range, const char *filename,
                       bool colorful) {
  CHECK(matrix_image_.rows() > 0 && matrix_image_.cols() > 0);
  FILE *fp = fopen(filename, "wb");
  if (!fp) {
    LOG(ERROR) << "Can not open file " << filename << std::endl;
    return;
  }

  png_uint_32 width = matrix_image_.cols();
  png_uint_32 height = matrix_image_.rows();
  png_structp png_ptr =
      png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
  if (!png_ptr) {
    LOG(ERROR) << "Could not allocate write struct" << std::endl;
    return;
  }
  png_infop info_ptr = png_create_info_struct(png_ptr);
  if (!info_ptr) {
    LOG(ERROR) << "Could not allocate info struct" << std::endl;
    png_destroy_write_struct(&png_ptr, (png_infopp)NULL);
    return;
  }

  LOG(INFO) << "Write range image file: " << filename << std::endl;
  png_init_io(png_ptr, fp);
  png_set_IHDR(png_ptr, info_ptr, width, height, 8, PNG_COLOR_TYPE_RGB,
               PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_BASE,
               PNG_FILTER_TYPE_BASE);
  png_write_info(png_ptr, info_ptr);

  png_bytep row = (png_bytep)malloc(3 * width * sizeof(png_byte));
  for (png_uint_32 y = 0; y < height; ++y) {
    for (png_uint_32 x = 0; x < width; ++x) {
      if (matrix_image_(y, x) > max_range) {
        continue;
      }
      if (colorful) {
        png_byte value = matrix_image_(y, x) / max_range * 255;
        if (matrix_image_(y, x) < 1.e-6) {
          row[x * 3] = 0;
          row[x * 3 + 1] = 255;
          row[x * 3 + 2] = 0;
        } else {
          row[x * 3] = value;
          row[x * 3 + 1] = value;
          row[x * 3 + 2] = value;
        }
      } else {
        png_byte value = matrix_image_(y, x) / max_range * 255;
        row[x * 3] = value;
        row[x * 3 + 1] = value;
        row[x * 3 + 2] = value;
      }
    }
    png_write_row(png_ptr, row);
  }
  png_write_end(png_ptr, NULL);

  if (fp != NULL) {
    fclose(fp);
  }
  if (info_ptr != NULL) {
    png_free_data(png_ptr, info_ptr, PNG_FREE_ALL, -1);
  }
  if (png_ptr != NULL) {
    png_destroy_write_struct(&png_ptr, (png_infopp)NULL);
  }
}

void RangeImage::LabelOneComponent(LabelT label, Index index) {
  std::queue<Index> labeling_queue;
  std::vector<int> indices;
  labeling_queue.push(index);

  int row_num = matrix_image_.rows();
  int col_num = matrix_image_.cols();
  CHECK_EQ(row_num, vertical_line_num_);
  CHECK_EQ(col_num, horizontal_line_num_);

  const float image_horizontal_res =
      M_PI * 2 / static_cast<float>(horizontal_line_num_);
  const float image_vertical_res = (top_angle_ - btm_angle_) /
                                   static_cast<float>(vertical_line_num_) /
                                   180. * M_PI;

  while (!labeling_queue.empty()) {
    const Index current = labeling_queue.front();
    labeling_queue.pop();
    if (matrix_image_(current[0], current[1]) < 1.e-6 ||
        map_image_.at(current).label > 0) {
      continue;
    }

    map_image_.at(current).label = label;
    CHECK_GE(map_image_.at(current).index, 0);
    indices.push_back(map_image_.at(current).index);
    for (const auto &step : neighbors_) {
      Index neighbor = current + step;
      if (neighbor[0] < 0 || neighbor[0] >= row_num) {
        continue;
      }
      if (neighbor[1] < 0) {
        neighbor[1] += col_num;
      } else if (neighbor[1] >= col_num) {
        neighbor[1] -= col_num;
      }

      if (matrix_image_(neighbor[0], neighbor[1]) < 1.e-6 ||
          map_image_.at(neighbor).label > 0) {
        continue;
      }

      float d1 = std::max(matrix_image_(current[0], current[1]),
                          matrix_image_(neighbor[0], neighbor[1]));
      float d2 = std::min(matrix_image_(current[0], current[1]),
                          matrix_image_(neighbor[0], neighbor[1]));

      float alpha = 0.;
      if (step[0] == 0) {
        alpha = image_horizontal_res;
      } else {
        alpha = image_vertical_res;
      }
      float beta =
          std::atan2(d2 * std::sin(alpha), (d1 - d2 * std::cos(alpha)));
      if (beta > segmentation_rad_threshold_) {
        labeling_queue.push(neighbor);
      }
    }
  }
  if (indices.size() >= 20) {
    clusters_[label] = std::move(indices);
  }
}

}  // namespace filter
}  // namespace pre_processers
}  // namespace static_map
