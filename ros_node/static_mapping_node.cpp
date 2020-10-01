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

// third party
#include <nav_msgs/Path.h>
#include <pcl/console/parse.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
// stl
#include <unistd.h>
#include <sstream>
// local
#include "builder/map_builder.h"
#include "common/file_utils.h"
#include "common/performance/simple_prof.h"
#include "common/simple_time.h"
#include "ros_node/kitti_reader.h"
#include "ros_node/msg_conversion.h"
#include "ros_node/playable_bag.h"
#include "ros_node/tf_bridge.h"

using static_map::MapBuilder;
static MapBuilder::Ptr map_builder;

using static_map::sensors::ImuMsg;
using static_map::sensors::NavSatFixMsg;
using static_map::sensors::OdomMsg;

void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*msg, pcl_pc2);
  MapBuilder::PointCloudPtr incoming_cloud(new MapBuilder::PointCloudType);
  pcl::fromPCLPointCloud2(pcl_pc2, *incoming_cloud);

  std::vector<int> inliers;  // no use, just for the function
  pcl::removeNaNFromPointCloud(*incoming_cloud, *incoming_cloud, inliers);
  map_builder->InsertPointcloudMsg(incoming_cloud);
}

void imu_callback(const sensor_msgs::Imu& imu_msg) {
  ImuMsg::Ptr incomming_imu(new ImuMsg);
  *incomming_imu = static_map_ros::ToLocalImu(imu_msg);
  map_builder->InsertImuMsg(incomming_imu);
}

void odom_callback(const nav_msgs::Odometry& odom_msg) {
  OdomMsg::Ptr local_odom(new OdomMsg);
  *local_odom = static_map_ros::ToLocalOdom(odom_msg);
  map_builder->InsertOdomMsg(local_odom);
}

void gps_callback(const sensor_msgs::NavSatFix& gps_msg) {
  NavSatFixMsg::Ptr local_gps(new NavSatFixMsg);
  *local_gps = static_map_ros::ToLocalNavSatMsg(gps_msg);
  map_builder->InsertGpsMsg(local_gps);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "static_mapping_node");
  ros::NodeHandle n;

  SIMPLE_PROF_USE_MS;
  SIMPLE_PROF_MERGE_OUTPUT;

  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  google::InstallFailureSignalHandler();

  // parse auguements
  // point cloud
  std::string point_cloud_topic = "";
  pcl::console::parse_argument(argc, argv, "-pc", point_cloud_topic);
  if (point_cloud_topic.empty()) {
    PRINT_ERROR("point cloud topic is empty!");
    return -1;
  } else {
    PRINT_INFO_FMT("Get point cloud from ROS topic: %s",
                   point_cloud_topic.c_str());
  }
  ros::Subscriber sub_pointcloud =
      n.subscribe(point_cloud_topic, 10, pointcloud_callback);
  std::string cloud_frame_id = "base_link";
  pcl::console::parse_argument(argc, argv, "-pc_frame_id", cloud_frame_id);

  // imu
  std::string imu_topic = "";
  bool use_imu = false;
  ros::Subscriber sub_imu;
  std::string imu_frame_id = "/novatel_imu";
  pcl::console::parse_argument(argc, argv, "-imu", imu_topic);
  if (imu_topic.empty()) {
    PRINT_WARNING("No imu topic, expect no imu messages.");
  } else {
    PRINT_INFO_FMT("Get imu data from ROS topic: %s", imu_topic.c_str());
    sub_imu = n.subscribe(imu_topic, 100, imu_callback);
    pcl::console::parse_argument(argc, argv, "-imu_frame_id", imu_frame_id);
    use_imu = true;
  }

  // odom
  bool use_odom = false;
  std::string odom_topic = "";
  std::string odom_frame_id = "";
  pcl::console::parse_argument(argc, argv, "-odom", odom_topic);
  pcl::console::parse_argument(argc, argv, "-odom_frame_id", odom_frame_id);
  ros::Subscriber sub_odom;
  if (!odom_topic.empty() && !odom_frame_id.empty()) {
    sub_odom = n.subscribe(odom_topic, 100, odom_callback);
    use_odom = true;
    PRINT_INFO_FMT("Get odom data from ROS topic: %s", odom_topic.c_str());
  } else {
    PRINT_WARNING("No odom data accessed!");
  }

  // gps
  bool use_gps = false;
  std::string gps_topic = "";
  std::string gps_frame_id = "";
  pcl::console::parse_argument(argc, argv, "-gps", gps_topic);
  pcl::console::parse_argument(argc, argv, "-gps_frame_id", gps_frame_id);
  ros::Subscriber gps_subscriber;
  if (!gps_topic.empty() && !gps_frame_id.empty()) {
    gps_subscriber = n.subscribe(gps_topic, 100, gps_callback);
    use_gps = true;
    PRINT_INFO_FMT("Get gps data from ROS topic: %s", gps_topic.c_str());
  } else {
    PRINT_WARNING("No GPS data accessed!");
  }

  // config file
  std::string config_file = "";
  pcl::console::parse_argument(argc, argv, "-cfg", config_file);

  // urdf file
  std::string urdf_file = "";
  pcl::console::parse_argument(argc, argv, "-urdf", urdf_file);

  // tracking frame
  // default: base_link
  std::string tracking_frame = "base_link";
  pcl::console::parse_argument(argc, argv, "-track", tracking_frame);

  // ros bag
  std::string bag_file = "";
  pcl::console::parse_argument(argc, argv, "-bag", bag_file);
  if (!bag_file.empty() && !static_map::common::FileExist(bag_file)) {
    PRINT_WARNING_FMT("%s does not exist.", bag_file.c_str());
    bag_file = "";
  }

  // kitti data set
  std::string kitti_path = "";
  pcl::console::parse_argument(argc, argv, "-kitti", kitti_path);
  if (!kitti_path.empty() && !static_map::common::FileExist(kitti_path)) {
    PRINT_WARNING_FMT("%s does not exist.", kitti_path.c_str());
    kitti_path = "";
  }

  if (!kitti_path.empty() && !bag_file.empty()) {
    PRINT_ERROR("Should not read bag and kitti data set at the same time.");
    return -1;
  }

  std::string pub_topics = "";
  pcl::console::parse_argument(argc, argv, "-pubs", pub_topics);

  ros::Publisher map_publisher =
      n.advertise<sensor_msgs::PointCloud2>("/optimized_map", 1);
  ros::Publisher submap_publisher =
      n.advertise<sensor_msgs::PointCloud2>("/submap", 1);
  ros::Publisher pose_marker_pub =
      n.advertise<visualization_msgs::Marker>("/submap_pose", 1);
  ros::Publisher path_pub =
      n.advertise<nav_msgs::Path>("/static_mapping_path", 1);

  auto show_function =
      [&](const static_map::MapBuilder::PointCloudPtr& cloud) -> void {
    sensor_msgs::PointCloud2 map_pointcloud;
    pcl::toROSMsg(*cloud, map_pointcloud);
    map_pointcloud.header.frame_id = "/map";
    map_pointcloud.header.stamp = ros::Time::now();
    map_publisher.publish(map_pointcloud);
  };

  auto show_submap_function =
      [&](const static_map::MapBuilder::PointCloudPtr& cloud) -> void {
    sensor_msgs::PointCloud2 submap_pointcloud;
    pcl::toROSMsg(*cloud, submap_pointcloud);
    submap_pointcloud.header.frame_id = "/map";
    submap_pointcloud.header.stamp = ros::Time::now();
    submap_publisher.publish(submap_pointcloud);
  };

  const auto publish_path = [&](const std::vector<Eigen::Matrix4d>& poses) {
    nav_msgs::Path path;
    path.header.frame_id = "/map";

    path.poses.reserve(poses.size());
    for (const Eigen::Matrix4d& pose : poses) {
      geometry_msgs::PoseStamped pose_ros;
      pose_ros.header.frame_id = "/map";
      Eigen::Quaterniond q(Eigen::Matrix3d(pose.block(0, 0, 3, 3)));
      pose_ros.pose.orientation.w = q.w();
      pose_ros.pose.orientation.x = q.x();
      pose_ros.pose.orientation.y = q.y();
      pose_ros.pose.orientation.z = q.z();
      pose_ros.pose.position.x = pose(0, 3);
      pose_ros.pose.position.y = pose(1, 3);
      pose_ros.pose.position.z = pose(2, 3);

      path.poses.push_back(pose_ros);
    }
    path_pub.publish(path);
  };

  map_builder = std::make_shared<MapBuilder>();

  // @todo(edward) merge these 2 situation into 1
  if (urdf_file.empty()) {
    tf::TransformListener listener;
    map_builder->SetTrackingToLidar(static_map_ros::LoopUpTransfrom(
        tracking_frame, cloud_frame_id, listener));
    if (use_imu) {
      map_builder->SetTrackingToImu(static_map_ros::LoopUpTransfrom(
          tracking_frame, imu_frame_id, listener));
    }
    if (use_odom) {
      map_builder->SetTransformOdomToLidar(static_map_ros::LoopUpTransfrom(
          odom_frame_id, cloud_frame_id, listener));
      map_builder->SetTrackingToOdom(static_map_ros::LoopUpTransfrom(
          tracking_frame, odom_frame_id, listener));
    }
    if (use_gps) {
      map_builder->SetTrackingToGps(static_map_ros::LoopUpTransfrom(
          tracking_frame, gps_frame_id, listener));
    }
  } else {
    tf2_ros::Buffer tf_buffer;
    static_map_ros::ReadStaticTransformsFromUrdf(urdf_file, &tf_buffer);
    map_builder->SetTrackingToLidar(static_map_ros::LoopUpTransfrom(
        tracking_frame, cloud_frame_id, tf_buffer));
    if (use_imu) {
      map_builder->SetTrackingToImu(static_map_ros::LoopUpTransfrom(
          tracking_frame, imu_frame_id, tf_buffer));
    }
    if (use_odom) {
      map_builder->SetTransformOdomToLidar(static_map_ros::LoopUpTransfrom(
          odom_frame_id, cloud_frame_id, tf_buffer));
      map_builder->SetTrackingToOdom(static_map_ros::LoopUpTransfrom(
          tracking_frame, odom_frame_id, tf_buffer));
    }
    if (use_gps) {
      map_builder->SetTrackingToGps(static_map_ros::LoopUpTransfrom(
          tracking_frame, gps_frame_id, tf_buffer));
    }
  }

  if (!config_file.empty()) {
    const auto options = map_builder->Initialise(config_file.c_str());
    if (options.front_end_options.imu_options.enabled && !use_imu) {
      PRINT_ERROR("You should set a imu topic if you enable using imu.");
      map_builder->FinishAllComputations();
      return 0;
    }
  } else {
    map_builder->Initialise(NULL);
  }
  map_builder->EnableUsingOdom(use_odom);
  map_builder->EnableUsingGps(use_gps);
  if (!pub_topics.empty()) {
    if (std::strstr(pub_topics.c_str(), "[map]")) {
      map_builder->SetShowMapFunction(show_function);
    }
    if (std::strstr(pub_topics.c_str(), "[submap]")) {
      map_builder->SetShowSubmapFunction(show_submap_function);
    }
    if (std::strstr(pub_topics.c_str(), "[path]")) {
      map_builder->SetShowPathFunction(publish_path);
    }
  }

  REGISTER_FUNC;

  if (!bag_file.empty()) {
    static_map_ros::PlayableBag bag(bag_file, ros::TIME_MIN, ros::TIME_MAX,
                                    ::ros::Duration(1.0));

    const double duration_sec = bag.DurationInSeconds();
    const auto begin_time = std::get<0>(bag.GetBeginEndTime());

    int message_index = 0;
    ros::Time last_message_time = begin_time;
    while (bag.IsMessageAvailable() && ros::ok()) {
      auto message = bag.GetNextMessage();
      if (message.isType<sensor_msgs::PointCloud2>() &&
          message.getTopic() == point_cloud_topic) {
        pointcloud_callback(message.instantiate<sensor_msgs::PointCloud2>());
      } else if (message.isType<sensor_msgs::Imu>() &&
                 message.getTopic() == imu_topic) {
        if (use_imu) {
          imu_callback(*message.instantiate<sensor_msgs::Imu>());
        }
      } else if (message.isType<nav_msgs::Odometry>() &&
                 message.getTopic() == odom_topic) {
        if (use_odom) {
          odom_callback(*message.instantiate<nav_msgs::Odometry>());
        }
      } else if (message.isType<sensor_msgs::NavSatFix>() &&
                 message.getTopic() == gps_topic) {
        if (use_gps) {
          gps_callback(*message.instantiate<sensor_msgs::NavSatFix>());
        }
      }

      // read bag in twice time of the bag speed
      // like : rosbag play ***.bag -r 2
      if (message.getTime() > last_message_time) {
        usleep((message.getTime() - last_message_time).toSec() * 0.1 * 1000000);
      }
      last_message_time = message.getTime();

      ++message_index;
      if (message_index % 10000 == 0) {
        PRINT_COLOR_FMT(BOLD, "processed %lf of %lf seconds of the bag... ",
                        (message.getTime() - begin_time).toSec(), duration_sec);
      }
    }
  } else if (!kitti_path.empty()) {
    PRINT_INFO("No bag, read kitti data.");
    static_map::KittiReader kitti_reader;
    kitti_reader.SetPointCloudDataPath(kitti_path);
    int index = 0;
    while (true && ros::ok()) {
      MapBuilder::PointCloudPtr point_cloud = kitti_reader.ReadFromBin(index);
      if (point_cloud && !point_cloud->empty()) {
        // LOG(ERROR) << point_cloud->size();
        point_cloud->header.frame_id = cloud_frame_id;
        point_cloud->header.stamp =
            static_map::SimpleTime::get_current_time().toNSec() / 1000ull;
        map_builder->InsertPointcloudMsg(point_cloud);

        usleep(100000);
      } else {
        break;
      }
      index++;
    }
  } else {
    ::ros::spin();
  }

  map_builder->FinishAllComputations();
  return 0;
}
