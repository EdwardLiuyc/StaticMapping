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
#include <sstream>
// local
#include "builder/map_builder.h"
#include "builder/msg_conversion.h"
#include "common/simple_time.h"
#include "ros_node/tf_bridge.h"

using static_map::MapBuilder;
static MapBuilder::Ptr map_builder;

using static_map::sensors::ImuMsg;
using static_map::sensors::NavSatFixMsg;
using static_map::sensors::OdomMsg;

void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*msg, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZI>::Ptr incoming_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromPCLPointCloud2(pcl_pc2, *incoming_cloud);

  std::vector<int> inliers;  // no use, just for the function
  pcl::removeNaNFromPointCloud(*incoming_cloud, *incoming_cloud, inliers);
  map_builder->InsertPointcloudMsg(incoming_cloud);
}

void imu_callback(const sensor_msgs::Imu& imu_msg) {
  ImuMsg::Ptr incomming_imu(new ImuMsg);
  *incomming_imu = static_map::sensors::ToLocalImu(imu_msg);
  map_builder->InsertImuMsg(incomming_imu);
}

void odom_callback(const nav_msgs::Odometry& odom_msg) {
  OdomMsg::Ptr local_odom(new OdomMsg);
  *local_odom = static_map::sensors::ToLocalOdom(odom_msg);
  map_builder->InsertOdomMsg(local_odom);
}

void gps_callback(const sensor_msgs::NavSatFix& gps_msg) {
  NavSatFixMsg::Ptr local_gps(new NavSatFixMsg);
  *local_gps = static_map::sensors::ToLocalNavSatMsg(gps_msg);
  map_builder->InsertGpsMsg(local_gps);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "static_mapping_node");
  ros::NodeHandle n;

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

  ros::Publisher map_publisher =
      n.advertise<sensor_msgs::PointCloud2>("/optimized_map", 1);
  ros::Publisher submap_publisher =
      n.advertise<sensor_msgs::PointCloud2>("/submap", 1);
  ros::Publisher pose_marker_pub =
      n.advertise<visualization_msgs::Marker>("/submap_pose", 1);

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

  auto show_pose_function = [&](const Eigen::Matrix4f& pose) -> void {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information
    // on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS
    // Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the
    // frame/time specified in the header
    marker.pose.position.x = pose(0, 3);
    marker.pose.position.y = pose(1, 3);
    marker.pose.position.z = pose(2, 3);
    Eigen::Quaternionf q(Eigen::Matrix3f(pose.block(0, 0, 3, 3)));
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 2.0;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    pose_marker_pub.publish(marker);
  };

  map_builder = std::make_shared<MapBuilder>();

  // @todo(edward) add a paramater fot tracking frame
  const std::string tracking_frame = "base_link";

  if (urdf_file.empty()) {
    tf::TransformListener listener;
    map_builder->SetTrackingToLidar(
        static_map_ros::LoopUpTransfrom(tracking_frame, cloud_frame_id,
                                        listener)
            .cast<float>());
    if (use_imu) {
      map_builder->SetTrackingToImu(static_map_ros::LoopUpTransfrom(
                                        tracking_frame, imu_frame_id, listener)
                                        .cast<float>());
    }
    if (use_odom) {
      map_builder->SetTransformOdomToLidar(
          static_map_ros::LoopUpTransfrom(odom_frame_id, cloud_frame_id,
                                          listener)
              .cast<float>());
    }
  } else {
    tf2_ros::Buffer tf_buffer;
    static_map_ros::ReadStaticTransformsFromUrdf(urdf_file, &tf_buffer);
    map_builder->SetTrackingToLidar(
        static_map_ros::LoopUpTransfrom(tracking_frame, cloud_frame_id,
                                        tf_buffer)
            .cast<float>());
    if (use_imu) {
      map_builder->SetTrackingToImu(static_map_ros::LoopUpTransfrom(
                                        tracking_frame, imu_frame_id, tf_buffer)
                                        .cast<float>());
    }
    if (use_odom) {
      map_builder->SetTransformOdomToLidar(
          static_map_ros::LoopUpTransfrom(odom_frame_id, cloud_frame_id,
                                          tf_buffer)
              .cast<float>());
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
  map_builder->SetShowMapFunction(show_function);
  map_builder->SetShowPoseFunction(show_pose_function);
  map_builder->SetShowSubmapFunction(show_submap_function);
  ros::Rate loop_rate(1000);
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  map_builder->FinishAllComputations();
  while (ros::ok()) {
    static_map::SimpleTime::from_sec(1.).sleep();
  }
  return 0;
}
