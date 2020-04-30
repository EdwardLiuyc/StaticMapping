# docker file for static mapping 

FROM ros:melodic-ros-core-bionic

MAINTAINER edward "liuyongchuan@outlook.com"

COPY ./config/tsinghua_source.txt /etc/apt/sources.list

RUN sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros1-latest.list'

RUN apt -y update && apt -y upgrade && \
  apt -y install cmake libboost-dev libeigen3-dev libpng-dev libgoogle-glog-dev libatlas-base-dev \
  libsuitesparse-dev libpcl-dev libceres-dev wget \
  ros-melodic-tf* ros-melodic-pcl* ros-melodic-opencv* ros-melodic-urdf

# Enable tab completion by uncommenting it from /etc/bash.bashrc
# The relevant lines are those below the phrase "enable bash completion in interactive shells"
RUN export SED_RANGE="$(($(sed -n '\|enable bash completion in interactive shells|=' /etc/bash.bashrc)+1)),$(($(sed -n '\|enable bash completion in interactive shells|=' /etc/bash.bashrc)+7))" && \
    sed -i -e "${SED_RANGE}"' s/^#//' /etc/bash.bashrc && \
    unset SED_RANGE

# Create user "docker" with sudo powers
RUN useradd -m docker && \
    usermod -aG sudo docker && \
    echo '%sudo ALL=(ALL) NOPASSWD: ALL' >> /etc/sudoers && \
    cp /root/.bashrc /home/docker/ && \
    mkdir -p /home/docker/src/StaticMapping && \
    chown -R --from=root docker /home/docker

# Use C.UTF-8 locale to avoid issues with ASCII encoding
ENV LC_ALL=C.UTF-8
ENV LANG=C.UTF-8

WORKDIR /home/docker/src
ENV HOME /home/docker
ENV USER docker
USER docker
ENV PATH /home/docker/.local/bin:$PATH
# Avoid first use of sudo warning. c.f. https://askubuntu.com/a/22614/781671
RUN touch $HOME/.sudo_as_admin_successful

## Get All Code first
RUN git clone https://github.com/ethz-asl/libnabo.git && \
  git clone https://github.com/ethz-asl/libpointmatcher.git && \
  wget https://nchc.dl.sourceforge.net/project/geographiclib/distrib/GeographicLib-1.50.1.tar.gz && \
  git clone https://bitbucket.org/gtborg/gtsam.git

## build dependencies
## libnabo 
RUN cd libnabo && \
  git checkout tags/1.0.7 && \
  mkdir build && cd build && \
  cmake .. && make -j4 && \
  sudo make install && cd ..

## libpointmatcher 
RUN cd libpointmatcher && \
  git checkout tags/1.3.1 && \
  mkdir build && cd build && \
  cmake .. && make -j4 && \
  sudo make install && cd ..

## Geographic
RUN tar -zxvf GeographicLib-1.50.1.tar.gz && cd GeographicLib-1.50.1 && \
  mkdir build && cd build && \
  cmake .. && make -j4 && \
  sudo make install && cd ..

## gtsam
RUN cd gtsam && \
  git checkout tags/4.0.0-alpha2 && \
  mkdir build && cd build && \
  cmake -DGTSAM_USE_SYSTEM_EIGEN=ON .. && \
  make -j4 && \
  sudo make install 
