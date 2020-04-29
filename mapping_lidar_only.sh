## usally, you can just leave this config file just like this, it will work fine
CONFIG_PATH=./config/lidar_only.xml
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

GPS_TOPIC=navsatfix
GPS_FRAME_ID=gps

./build/static_mapping_node \
  -cfg ${CONFIG_PATH} \
  -pc ${POINT_CLOUD_TOPIC} \
  -pc_frame_id ${POINT_CLOUD_FRAME_ID} \
  -urdf ${URDF_FILE} 

exit 0 

