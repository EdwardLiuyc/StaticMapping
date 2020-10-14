## usally, you can just leave this config file just like this, it will work fine
CONFIG_PATH=./config/lidar_only_kitti.xml
URDF_FILE=./urdf/test2.urdf
## the follow 2 items must be set!!!
## the topic name of your pointcloud msg (ros)
POINT_CLOUD_TOPIC=velodyne_points
## the frame id of your pointcloud msg (ros)
POINT_CLOUD_FRAME_ID=frame_velodyne_points

KITTI_PATH=/mnt/Data/kitti/lidar/00/velodyne

./build/ros_node/static_mapping_node \
  -cfg ${CONFIG_PATH} \
  -pc ${POINT_CLOUD_TOPIC} \
  -pc_frame_id ${POINT_CLOUD_FRAME_ID} \
  -urdf ${URDF_FILE} \
  -kitti ${KITTI_PATH} \
  -pubs [map][path][edge][submap]

exit 0 
