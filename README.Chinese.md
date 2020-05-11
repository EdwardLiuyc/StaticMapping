<img src="doc/logo.png" />

[![Badge](https://img.shields.io/badge/link-996.icu-%23FF4D5B.svg?style=flat-square)](https://996.icu/#/en_US)
[![LICENSE](https://img.shields.io/badge/license-Anti%20996-blue.svg?style=flat-square)](https://github.com/996icu/996.ICU/blob/master/LICENSE)
[![Slack](https://img.shields.io/badge/slack-996icu-green.svg?style=flat-square)](https://join.slack.com/t/996icu/shared_invite/enQtNjI0MjEzMTUxNDI0LTkyMGViNmJiZjYwOWVlNzQ3NmQ4NTQyMDRiZTNmOWFkMzYxZWNmZGI0NDA4MWIwOGVhOThhMzc3NGQyMDBhZDc)


# 简介
<img src="doc/mapping.png" width="800" />

# 构建 & 编译
## 使用 Docker
在 ubuntu 18.04 下，用 docker 来构建是最好的方式，后面会针对加入 ubuntu 16.04 等其他系统的 docker。
### 获取 docker 镜像
#### 中国大陆区域
在中国大陆，最快的方式就是直接从阿里云获取镜像：
```docker
docker pull registry.cn-hangzhou.aliyuncs.com/edward_slam/static_mapping:master_bionic_latest
```
或者也可以直接在本机构建：
```docker
docker build --rm -t slam/static_mapping:latest . 
```

#### 其他区域
首先，移除掉 dockerfile 中针对中国大陆区域优化的部分：
```docker
COPY ./config/tsinghua_source.txt /etc/apt/sources.list
RUN sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros1-latest.list' 
```
接着在本机构建：
```docker
docker build --rm -t slam/static_mapping:latest . 
```

### 构建项目
```bash 
## get code 
git clone https://github.com/EdwardLiuyc/StaticMapping.git
cd StaticMapping

## start the docker container
docker run -it --rm -v --net=host `pwd`:'/home/docker/src/StaticMapping' \
  registry.cn-hangzhou.aliyuncs.com/edward_slam/static_mapping:master_bionic_latest /bin/bash

## in the container 
mkdir -p build && cd build
cmake ..
make -j8
```

## 本地环境构建项目
### 依赖项
#### 基础库
```bash
## basic depencencies
sudo apt -y install cmake \
  libboost-dev \
  libeigen3-dev \
  libpng-dev \
  libgoogle-glog-dev \
  libatlas-base-dev\
  libsuitesparse-dev
```

#### ROS 
按照[官方文档](http://wiki.ros.org/kinetic/Installation/Ubuntu)来进行安装，项目在 kinetic 和 melodic 版本中都经过了测试。

#### PCL

```bash
## tested in pcl-1.7 (ubuntu16.04) and pcl-1.8 (ubuntu18.04)
sudo apt -y install libpcl-dev
```

#### GTSAM
```bash
## install GeoGraphic first 
wget https://nchc.dl.sourceforge.net/project/geographiclib/distrib/GeographicLib-1.50.1.tar.gz
tar -zxvf GeographicLib-1.50.1.tar.gz && cd GeographicLib-1.50.1 && \
  mkdir build && cd build && \
  cmake .. && make -j4 && \
  sudo make install && cd ..

## Go to your wordspace lisk /home/user/3rd_parties
## GTSAM(4.0 or higher is needed)
git clone https://bitbucket.org/gtborg/gtsam.git
cd gtsam 
git checkout tags/4.0.0-alpha2
mkdir build && cd build
cmake -DGTSAM_USE_SYSTEM_EIGEN=ON ..
make -j8
sudo make install 
```

#### Ceres Solver 
可以使用 apt 进行安装
```bash 
sudo apt -y install libceres-dev
```
或者你可以按照 ceres solver 的[官方安装文档](http://ceres-solver.org/installation.html)来安装

#### libnabo
```bash 
git clone https://github.com/ethz-asl/libnabo.git
cd libnabo
### checkout to the latest release version
git checkout tags/1.0.7
mkdir build && cd build
cmake ..
make -j8
sudo make install
```

#### libpointmatcher
```bash
git clone https://github.com/ethz-asl/libpointmatcher.git
cd libpointmatcher
mkdir build && cd build
cmake ..
make -j8
sudo make install
```

### 选配项
- **CUDA**: 我尝试过使用 cuda 加速 kdtree，尝试来加速 ICP，但是测试下来发现也只达到 libnabo 的 1.5x ~ 2x 的速度，所以暂时没有继续跟进。
- **cuda_utils**: 
- **TBB**: 在项目中我使用了 tbb 库中的一些线程安全的容器等，如果把 tbb 使用选项关掉，代码中会自动切换到 stl 容器，这样很多功能会退化到单线程，所以**强烈建议**打开 tbb 的选项。

### 编译项目
```bash
mkdir build && cd build
cmake ..
make -j8
```

# 如何使用？
> 目前建议是在本机运行建图，后面会继续完善在 docker 中建图的流程
## 步骤一 开启建图进程
```bash
mkdir pcd
mkdir -p pkgs/test
./mapping.sh
```
在这之前您可以了解脚本中的具体内容:
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

## 步骤二  
播放 rosbag 或者开启所需的传感器启动

## 步骤三  
直接 'CTRL+C' 停止建图的进程，不过要注意的是进程不会马上结束，您只需要等所有运算结束，点云会生成在 pcd 文件夹中：  
<img src="doc/xi1_xi2.png" width="800" />  
其中的一部分：  
<img src="doc/detail.png" width="800" />

# 文档  
使用 `doxygen Doxyfile` 生成文档，文档会生成在 `doc` 中.

# 引用
1. **M2DP: A Novel 3D Point Cloud Descriptor and Its Application in Loop**,Li He, Xiaolong Wang and Hong Zhang,IEEE International Conference on Intelligent Robots and Systems (2016) 2016-November 231-237
2. **ISAM2: Incremental smoothing and mapping using the Bayes tree**,Michael Kaess, Hordur Johannsson, Richard Roberts, Viorela Ila, John Leonard, and Frank Dellaert Abstract, International Journal of Robotics Research (2012) 31(2) 216-235
3. **Comparing ICP Variants on Real-World Data Sets**, Autonomous Robots 2013
4. **Fast Segmentation of 3D Pointcloud for Ground Vehicles**, M. Himmelsbach and Felix v. Hundelshausen and H.-J. Wuensche, IEEE Intelligent Vehicles Symposium, Proceedings, 2010