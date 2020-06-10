## Introduction
<img src="../mapping.png" width="800" />

## build
### requirements  
- PCL(1.8 or higher)
- GTSAM(4.0 or higher)
- Boost
- Eigen
- libPNG
- libnabo
- libpointmatcher
- CUDA(optional, default: OFF)
  - cuda_utils( if use cuda )
- TBB(optional, default: ON)
- OpenCV(optional, default: ON)  

#### GTSAM

```bash
git clone https://bitbucket.org/gtborg/gtsam.git
cd gtsam 
mkdir build && cd build
ccmake ..

### important ### 
# set GTSAM_USE_SYSTEM_EIGEN to ON
# then generate makefile

make -j8
sudo make install 
```

#### libnabo

```bash
git clone https://github.com/ethz-asl/libnabo.git
cd libnabo
### checkout to the latest release version
git checkout tags/1.0.6
mkdir build && cd build
cmake ..
make -j8
sudo make install
```

#### libpointmatcher 

```bash
git clone https://github.com/ethz-asl/libpointmatcher.git
cd libnabo
mkdir build && cd build
cmake ..
make -j8
sudo make install
```

### About optional libs
- Cuda: We have made some attempts in fasting the kdtree in ICP by creating the kdtree on GPU, but the GPU Kdtree is not fast enough(just 1.5~2 times faster than libnabo). Notice that if you use CUDA, your g++ version should be lower than 6.0 because the nvcc does not support the 6.0 or high version g++.  
- TBB: We have used concurrency containers in TBB for many multi-thread situations, if you turn off this options, the process will use stl containers such as std::vector instead and many multi-thread algorithm will degenerate into single-thread. So, Using TBB is **strongly recommended**;  
- Opencv: All the matrices in code is in Eigen way, Opencv is only for generating the jpg file of pose gragh. It is a debug function so you can change this option as you wish.

### compiling

```bash
chmod +x make.sh
mkdir build && cd build
cmake .. & cd ..
./make.sh  # directly using cmake
```

### BUG
- DO NOT use g++ 7.x, unknow bug with Eigen

## How to use?
### step1 run the mapping process

```bash
mkdir pcd
mkdir -p pkgs/test
./mapping.sh
```
before that, you should kown what is in the script:
```bash
## usally, you can just leave this config file just like this, it will work fine
CONFIG_PATH=./config/static_mapping_default.xml
## the follow 2 items must be set!!!
## the topic name of your pointcloud msg (ros)
POINT_CLOUD_TOPIC=/fused_point_cloud
## the frame id of your pointcloud msg (ros)
POINT_CLOUD_FRAME_ID=base_link

## the following items are optional
## if you do not have a imu or gps or odom
## just remove the line started with
## imu: -imu -imu_frame_id
## odom: -odom -odom_frame_id
## gps: -gps -gps_frame_id
## and If you got one of these topics
## you MUST provide the tf connection between the one to pointcloud frame
IMU_TOPIC=/imu/data
IMU_FRAME_ID=novatel_imu

ODOM_TOPIC=/navsat/odom
ODOM_FRAME_ID=novatel_odom

GPS_TOPIC=/navsat/fix
GPS_FRAME_ID=novatel_imu

./build/static_mapping_node \
  -cfg ${CONFIG_PATH} \
  -pc ${POINT_CLOUD_TOPIC} \
  -pc_frame_id ${POINT_CLOUD_FRAME_ID} \
  -imu ${IMU_TOPIC} \
  -imu_frame_id ${IMU_FRAME_ID} \
  -odom ${ODOM_TOPIC} \
  -odom_frame_id ${ODOM_FRAME_ID} \
  -gps ${GPS_TOPIC} \
  -gps_frame_id ${GPS_FRAME_ID} 

exit 0 
```

### step2 
play bag that includes pointcloud msgs or run the lidar driver

### step3 
when finished, just press 'CTRL+C' to terminate the mapping process. NOTICE that the mapping process will not end right after you 'CTRL+C', it has many more calculations to do, so just wait.  
Finally, you will get a static map like this:
<img src="../xi1_xi2.png" width="800" />  
<img src="../detail.png" width="800" />  

