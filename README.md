<img src="https://i.v2ex.co/701l6CyX.png" />

[![Badge](https://img.shields.io/badge/link-996.icu-%23FF4D5B.svg?style=flat-square)](https://996.icu/#/en_US)
[![LICENSE](https://img.shields.io/badge/license-Anti%20996-blue.svg?style=flat-square)](https://github.com/996icu/996.ICU/blob/master/LICENSE)
[![Slack](https://img.shields.io/badge/slack-996icu-green.svg?style=flat-square)](https://join.slack.com/t/996icu/shared_invite/enQtNjI0MjEzMTUxNDI0LTkyMGViNmJiZjYwOWVlNzQ3NmQ4NTQyMDRiZTNmOWFkMzYxZWNmZGI0NDA4MWIwOGVhOThhMzc3NGQyMDBhZDc)


## Introduction
<img src="https://i.v2ex.co/JJR3KAjT.png" width="800" />

## Build
### requirements  
```bash
## basic depencencies
sudo apt install cmake \
  libboost-dev \
  ibeigen3-dev \
  libpng-dev \
  libgoogle-glog-dev \
  libatlas-base-dev

# SuiteSparse and CXSparse (optional)
# - If you want to build Ceres as a *static* library (the default)
#   you can use the SuiteSparse package in the main Ubuntu package
#   repository:
sudo apt-get install libsuitesparse-dev
# - However, if you want to build Ceres as a *shared* library, you must
#   add the following PPA:
sudo add-apt-repository ppa:bzindovic/suitesparse-bugfix-1319687
sudo apt update
sudo apt install libsuitesparse-dev

## PCL(1.8 or higher version is strongly recommended)
sudo apt install libpcl-dev

## clone some 3rd party libs into a new folder
mkdir workspace
cd workspace
## GTSAM(4.0 or higher is needed)
git clone https://bitbucket.org/gtborg/gtsam.git
cd gtsam 
mkdir build && cd build
ccmake ..
### important
# set GTSAM_USE_SYSTEM_EIGEN to ON
# then generate makefile
make -j8
sudo make install 
cd ../..

## Ceres Solver(1.12 or higher)
git clone https://github.com/ceres-solver/ceres-solver.git
cd ceres-solver
git checkout tags/1.14.0
mkdir build && cd build
cmake ..
make -j8
sudo make install
cd ../..

## libnabo 
git clone https://github.com/ethz-asl/libnabo.git
cd libnabo
### checkout to the latest release version
git checkout tags/1.0.6
mkdir build && cd build
cmake ..
make -j8
sudo make install
cd ../..

## libpointmatcher
git clone https://github.com/ethz-asl/libpointmatcher.git
cd libnabo
mkdir build && cd build
cmake ..
make -j8
sudo make install
cd ../..
```

### Optional libs
- Cuda: We have made some attempts in fasting the kdtree in ICP by creating the kdtree on GPU, but the GPU Kdtree is not fast enough(just 1.5~2 times faster than libnabo). Notice that if you use CUDA, your g++ version should be lower than 6.0 because the nvcc does not support the 6.0 or high version g++.  
- cuda_utils: 
- TBB: We have used concurrency containers in TBB for many multi-thread situations, if you turn off this options, the process will use stl containers such as std::vector instead and many multi-thread algorithm will degenerate into single-thread. So, Using TBB is **strongly recommended**;  
- Opencv: All the matrices in code is in Eigen way, Opencv is only for generating the jpg file of pose gragh. It is a debug function so you can change this option as you wish.

### compiling
```bash
mkdir build && cd build
cmake ..
make -j8
sudo make install
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
## usally, you can leave this config file just like this, it will work fine
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

<img src="https://i.v2ex.co/39r4ya20l.png" witdh="800" />

part of it:  
<img src="https://i.v2ex.co/Z2hNFv49l.png" witdh="800" />

## document  
You can use `doxygen Doxyfile` to generate your docs, they are in the `doc` folder.

## References
1. **M2DP: A Novel 3D Point Cloud Descriptor and Its Application in Loop**,Li He, Xiaolong Wang and Hong Zhang,IEEE International Conference on Intelligent Robots and Systems (2016) 2016-November 231-237
2. **ISAM2: Incremental smoothing and mapping using the Bayes tree**,Michael Kaess, Hordur Johannsson, Richard Roberts, Viorela Ila, John Leonard, and Frank Dellaert Abstract, International Journal of Robotics Research (2012) 31(2) 216-235
3. **Comparing ICP Variants on Real-World Data Sets**, Autonomous Robots 2013
4. **Fast Segmentation of 3D Pointcloud for Ground Vehicles**, M. Himmelsbach and Felix v. Hundelshausen and H.-J. Wuensche, IEEE Intelligent Vehicles Symposium, Proceedings, 2010
