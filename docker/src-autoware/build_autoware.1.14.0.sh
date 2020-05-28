#!/bin/bash

sudo rm -rf /home/ros/catkin_ws

sudo apt update
sudo apt-get install -y \
    python-catkin-pkg \
    python-rosdep \
    python3-pip \
    python3-colcon-common-extensions \
    python3-setuptools \
    python3-vcstool

pip3 install -U setuptools

cd /tmp
wget http://bitbucket.org/eigen/eigen/get/3.3.7.tar.gz
mkdir eigen
tar --strip-components=1 -xzvf 3.3.7.tar.gz -C eigen
cd eigen
mkdir build
cd build
cmake ..
make
sudo make install
cd /tmp
rm -rf 3.3.7.tar.gz eigen

cd
mkdir -p autoware.ai/src
cd autoware.ai
wget -O autoware.ai.repos "https://gitlab.com/autowarefoundation/autoware.ai/autoware/raw/1.14.0/autoware.ai.repos?inline=false"
vcs import src < autoware.ai.repos
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro melodic

AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

echo 'source /home/ros/autoware.ai/install/setup.bash' >> /home/ros/.bashrc

sudo cp -v /tmp/src-autoware/entrypoint.sh /tmp/
