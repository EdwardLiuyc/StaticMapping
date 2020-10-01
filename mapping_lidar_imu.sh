## usally, you can just leave this config file just like this, it will work fine
CONFIG_PATH=./config/lidar_imu_default.xml
## if you have a urdf file that contains robot model,
## set it here and the mapping process will not listen to tf topics
URDF_FILE=./urdf/test2.urdf
## the follow 2 items must be set!!!
## the topic name of your pointcloud msg (ros)
POINT_CLOUD_TOPIC=velodyne_points
## the frame id of your pointcloud msg (ros)
POINT_CLOUD_FRAME_ID=frame_velodyne_points

## the following items are optional
## if you do not have an imu or gps or odom
## just remove the line started with
## imu: -imu -imu_frame_id
## odom: -odom -odom_frame_id
## gps: -gps -gps_frame_id
## and If you got one of these topics
## you MUST provide the tf connection between the one to pointcloud frame
IMU_TOPIC=imu/raw_data
IMU_FRAME_ID=imu_link

BAG_FILE=/mnt/Data/visual/map_test/20200507124024_1_NoPlace.bag

./build/ros_node/static_mapping_node \
  -cfg ${CONFIG_PATH} \
  -urdf ${URDF_FILE} \
  -pc ${POINT_CLOUD_TOPIC} \
  -pc_frame_id ${POINT_CLOUD_FRAME_ID} \
  -imu ${IMU_TOPIC} \
  -imu_frame_id ${IMU_FRAME_ID} \
  -bag ${BAG_FILE} \
  -pubs [map][path][edge]

exit 0 

