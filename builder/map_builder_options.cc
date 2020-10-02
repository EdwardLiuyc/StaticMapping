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

#include "builder/map_builder.h"
#include "pugixml/pugixml.hpp"

namespace static_map {

namespace {
void ReadMatcherOptions(
    const pugi::xml_node& father_node, const std::string& node_name,
    static_map::registrator::MatcherOptions* const options) {
  GET_SINGLE_OPTION(father_node, node_name.c_str(), "type", options->type, int,
                    registrator::Type);
  GET_SINGLE_OPTION(father_node, node_name.c_str(), "accepted_min_score",
                    options->accepted_min_score, float, float);

  if (!father_node.child(node_name.c_str()).empty()) {
    auto matcher_options_node = father_node.child(node_name.c_str());
    if (!matcher_options_node.child("inner_filters").empty()) {
      options->inner_filters_node = matcher_options_node.child("inner_filters");
    }
    for (auto registrator_options_node =
             matcher_options_node.child("registrator_options");
         registrator_options_node;
         registrator_options_node =
             registrator_options_node.next_sibling("registrator_options")) {
      if (registrator_options_node.attribute("type").as_int() ==
          options->type) {
        options->registrator_options_node =
            pugi::xml_node(registrator_options_node.internal_object());
        break;
      }
    }
  }
}
}  // namespace

void CheckOptions(const MapBuilderOptions& options) {
  CHECK_GE(options.back_end_options.submap_options.frame_count, 2)
      << "A submap must constain at least 2 frames" << std::endl;
  CHECK(options.back_end_options.loop_detector_setting.use_gps ||
        options.back_end_options.loop_detector_setting.use_descriptor)
      << "You should selete at least one way to do loop detect: use gps or "
         "descriptor or both.";
  CHECK_GE(options.front_end_options.accumulate_cloud_num, 1);
  CHECK_GT(options.output_mrvm_settings.hit_prob, 0.5);
  CHECK_LT(options.output_mrvm_settings.miss_prob, 0.5);
  CHECK_GE(options.output_mrvm_settings.max_point_num_in_cell, 1);
}

MapBuilderOptions& MapBuilder::Initialise(const char* config_file_name) {
  bool use_default_config = false;
  if (!config_file_name || config_file_name[0] == '\0') {
    PRINT_WARNING("Invalid config file name. using default config.");
    use_default_config = true;
  }
  std::ifstream file(config_file_name);
  if (!use_default_config && !file.good()) {
    PRINT_WARNING("File not exist. using default config.");
    use_default_config = true;
  }

  if (!use_default_config && !options_xml_doc_.load_file(config_file_name)) {
    PRINT_WARNING("Failed to load target file. using default config.");
    use_default_config = true;
  }
  pugi::xml_node doc_node = options_xml_doc_.child("edward_liu");
  if (!use_default_config && doc_node.empty()) {
    PRINT_WARNING("Wrong node. using default config.");
    use_default_config = true;
  }
  pugi::xml_node static_map_node = doc_node.child("static_mapping");
  if (!use_default_config && static_map_node.empty()) {
    PRINT_WARNING("Wrong node. using default config.");
    use_default_config = true;
  }

  if (use_default_config) {
    InitialiseInside();
    return options_;
  }

  PRINT_INFO_FMT("Load configurations from file: %s", config_file_name);

  std::cout
      << BOLD
      << "\n*********************** configs from file ***********************\n"
      << NONE_FORMAT << std::endl;

  // whole options
  auto& whole_options = options_.whole_options;
  using std::string;
  GET_SINGLE_OPTION(static_map_node, "whole_options", "export_file_path",
                    whole_options.export_file_path, string, string);
  GET_SINGLE_OPTION(static_map_node, "whole_options", "map_package_path",
                    whole_options.map_package_path, string, string);
  GET_SINGLE_OPTION(static_map_node, "whole_options", "odom_calib_mode",
                    whole_options.odom_calib_mode, int, OdomCalibrationMode);
  std::cout << std::endl;

  auto& output_mrvm_settings = options_.output_mrvm_settings;
  GET_SINGLE_OPTION(static_map_node, "output_mrvm_settings", "output_average",
                    output_mrvm_settings.output_average, bool, bool);
  GET_SINGLE_OPTION(static_map_node, "output_mrvm_settings", "output_rgb",
                    output_mrvm_settings.output_rgb, bool, bool);
  GET_SINGLE_OPTION(static_map_node, "output_mrvm_settings",
                    "use_max_intensity", output_mrvm_settings.use_max_intensity,
                    bool, bool);
  GET_SINGLE_OPTION(static_map_node, "output_mrvm_settings", "prob_threshold",
                    output_mrvm_settings.prob_threshold, float, float);
  GET_SINGLE_OPTION(static_map_node, "output_mrvm_settings", "low_resolution",
                    output_mrvm_settings.low_resolution, float, float);
  GET_SINGLE_OPTION(static_map_node, "output_mrvm_settings", "high_resolution",
                    output_mrvm_settings.high_resolution, float, float);
  GET_SINGLE_OPTION(static_map_node, "output_mrvm_settings", "hit_prob",
                    output_mrvm_settings.hit_prob, float, float);
  GET_SINGLE_OPTION(static_map_node, "output_mrvm_settings", "miss_prob",
                    output_mrvm_settings.miss_prob, float, float);
  GET_SINGLE_OPTION(static_map_node, "output_mrvm_settings", "z_offset",
                    output_mrvm_settings.z_offset, float, float);
  GET_SINGLE_OPTION(static_map_node, "output_mrvm_settings",
                    "max_point_num_in_cell",
                    output_mrvm_settings.max_point_num_in_cell, int, int);

  std::cout << std::endl;

  // filters
  auto filters_node = static_map_node.child("filters");
  if (!filters_node.empty()) {
    filter_factory_.InitFromXmlNode(filters_node);
  } else {
    std::cout << BOLD << "  No filters" << std::endl;
  }
  std::cout << std::endl;

  // front end options
  auto front_end_node = static_map_node.child("front_end_options");
  if (!front_end_node.empty()) {
    ReadMatcherOptions(front_end_node, "scan_matcher_options",
                       &options_.front_end_options.scan_matcher_options);

    auto& front_end_options = options_.front_end_options;
    GET_SINGLE_OPTION(static_map_node, "front_end_options",
                      "accumulate_cloud_num",
                      front_end_options.accumulate_cloud_num, int, int);

    auto& motion_filter_options = options_.front_end_options.motion_filter;
    GET_SINGLE_OPTION(front_end_node, "motion_filter", "translation_range",
                      motion_filter_options.translation_range, float, float);
    GET_SINGLE_OPTION(front_end_node, "motion_filter", "angle_range",
                      motion_filter_options.angle_range, float, float);
    GET_SINGLE_OPTION(front_end_node, "motion_filter", "time_range",
                      motion_filter_options.time_range, float, float);

    auto& motion_compensation_options =
        options_.front_end_options.motion_compensation_options;
    GET_SINGLE_OPTION(front_end_node, "motion_compensation_options", "enable",
                      motion_compensation_options.enable, bool, bool);
    GET_SINGLE_OPTION(front_end_node, "motion_compensation_options",
                      "use_average", motion_compensation_options.use_average,
                      bool, bool);

    auto& imu_options = options_.front_end_options.imu_options;
    GET_SINGLE_OPTION(front_end_node, "imu_options", "use_imu",
                      imu_options.enabled, bool, bool);
    GET_SINGLE_OPTION(front_end_node, "imu_options", "type", imu_options.type,
                      int, data::ImuType);
    GET_SINGLE_OPTION(front_end_node, "imu_options", "imu_frequency",
                      imu_options.frequency, float, float);
    GET_SINGLE_OPTION(front_end_node, "imu_options", "gravity_constant",
                      imu_options.gravity_constant, float, float);
  } else {
    PRINT_WARNING("No config for front end");
  }

  // back end options
  auto back_end_node = static_map_node.child("back_end_options");
  if (!back_end_node.empty()) {
    ReadMatcherOptions(back_end_node, "submap_matcher_options",
                       &options_.back_end_options.submap_matcher_options);

    auto& submap_options = options_.back_end_options.submap_options;
    GET_SINGLE_OPTION(back_end_node, "submap_options", "frame_count",
                      submap_options.frame_count, int, int32_t);
    GET_SINGLE_OPTION(back_end_node, "submap_options", "enable_inner_mrvm",
                      submap_options.enable_inner_mrvm, bool, bool);
    GET_SINGLE_OPTION(back_end_node, "submap_options", "enable_voxel_filter",
                      submap_options.enable_voxel_filter, bool, bool);
    GET_SINGLE_OPTION(back_end_node, "submap_options",
                      "enable_random_sampleing",
                      submap_options.enable_random_sampleing, bool, bool);
    GET_SINGLE_OPTION(back_end_node, "submap_options", "random_sampling_rate",
                      submap_options.random_sampling_rate, float, float);
    GET_SINGLE_OPTION(back_end_node, "submap_options", "voxel_size",
                      submap_options.voxel_size, float, float);
    GET_SINGLE_OPTION(back_end_node, "submap_options", "enable_disk_saving",
                      submap_options.enable_disk_saving, bool, bool);
    GET_SINGLE_OPTION(back_end_node, "submap_options", "enable_check",
                      submap_options.enable_check, bool, bool);
    GET_SINGLE_OPTION(back_end_node, "submap_options", "disk_saving_delay",
                      submap_options.disk_saving_delay, int, int32_t);
    GET_SINGLE_OPTION(back_end_node, "submap_options", "saving_name_prefix",
                      submap_options.saving_name_prefix, string, string);

    auto& isam_optimizer_options =
        options_.back_end_options.isam_optimizer_options;
    GET_SINGLE_OPTION(back_end_node, "isam_optimizer_options", "use_odom",
                      isam_optimizer_options.use_odom, bool, bool);
    GET_SINGLE_OPTION(back_end_node, "isam_optimizer_options", "use_gps",
                      isam_optimizer_options.use_gps, bool, bool);
    GET_SINGLE_OPTION(back_end_node, "isam_optimizer_options",
                      "output_graph_pic",
                      isam_optimizer_options.output_graph_pic, bool, bool);

    auto& loop_detector_setting =
        options_.back_end_options.loop_detector_setting;
    GET_SINGLE_OPTION(back_end_node, "loop_detector_setting", "use_gps",
                      loop_detector_setting.use_gps, bool, bool);
    GET_SINGLE_OPTION(back_end_node, "loop_detector_setting", "use_descriptor",
                      loop_detector_setting.use_descriptor, bool, bool);
    GET_SINGLE_OPTION(back_end_node, "loop_detector_setting",
                      "output_matched_cloud",
                      loop_detector_setting.output_matched_cloud, bool, bool);
    GET_SINGLE_OPTION(back_end_node, "loop_detector_setting",
                      "trying_detect_loop_count",
                      loop_detector_setting.trying_detect_loop_count, int, int);
    GET_SINGLE_OPTION(back_end_node, "loop_detector_setting",
                      "loop_ignore_threshold",
                      loop_detector_setting.loop_ignore_threshold, int, int);
    GET_SINGLE_OPTION(back_end_node, "loop_detector_setting",
                      "nearest_history_pos_num",
                      loop_detector_setting.nearest_history_pos_num, int, int);
    GET_SINGLE_OPTION(
        back_end_node, "loop_detector_setting", "max_close_loop_distance",
        loop_detector_setting.max_close_loop_distance, float, float);
    GET_SINGLE_OPTION(
        back_end_node, "loop_detector_setting", "max_close_loop_z_distance",
        loop_detector_setting.max_close_loop_z_distance, float, float);
    GET_SINGLE_OPTION(back_end_node, "loop_detector_setting",
                      "m2dp_match_score",
                      loop_detector_setting.m2dp_match_score, float, float);
    GET_SINGLE_OPTION(
        back_end_node, "loop_detector_setting", "accept_scan_match_score",
        loop_detector_setting.accept_scan_match_score, float, float);
  }

  auto& map_package_options = options_.map_package_options;
  GET_SINGLE_OPTION(static_map_node, "map_package_options", "enable",
                    map_package_options.enable, bool, bool);
  GET_SINGLE_OPTION(static_map_node, "map_package_options", "border_offset",
                    map_package_options.border_offset, double, double);
  GET_SINGLE_OPTION(static_map_node, "map_package_options", "piece_width",
                    map_package_options.piece_width, double, double);
  GET_SINGLE_OPTION(static_map_node, "map_package_options", "cloud_file_prefix",
                    map_package_options.cloud_file_prefix, string, string);
  GET_SINGLE_OPTION(static_map_node, "map_package_options", "descript_filename",
                    map_package_options.descript_filename, string, string);

  std::cout
      << BOLD
      << "\n*****************************************************************\n"
      << NONE_FORMAT << std::endl;

  CheckOptions(options_);
  InitialiseInside();
  return options_;
}

}  // namespace static_map

#undef GET_SINGLE_OPTION
