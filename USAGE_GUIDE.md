# Usage Guide
Check out the Requirements below!

Afterwards simply check out the repo and integrate it into your ROS2 environment.

## Requirements
This node requires the [v2x_msgs](https://github.com/virtual-vehicle/v2x_msgs) library from the [vehicleCAPTAIN](https://github.com/virtual-vehicle/vehicle_captain) toolbox.

This node is designed for the [v2x_routing_core](https://github.com/virtual-vehicle/vehicle_captain_routing_core) from the [vehicleCAPTAIN](https://github.com/virtual-vehicle/vehicle_captain) toolbox.
However, you may introduce your own V2XServer connection.

Also, check out the ROS2 dependencies (package.xml)!

## In case you want to install ZMQ locally

You may use zmq_setup.sh for setup

In top level "CMakeLists.txt" replace setting of ZMQ libs with "FindZeroMQ.cmake"

### LibZMQ v4.3.4
```shell
cd /opt
git clone https://github.com/zeromq/libzmq.git
cd /opt/libzmq
git checkout v4.3.4

apt-get update -qq
apt-get install -qq --yes --no-install-recommends \
libkrb5-dev \
libsodium23

./autogen.sh
./configure --prefix=/usr/local --with-libgssapi_krb5
make
make check
make install

ldconfig && ldconfig -p | grep libzmq
```

### CPPZMQ v4.8.0
WARNING: v4.8.0 recommended, but Ubuntu 18.04 has wrong cmake version!
WARNING: also for that reason, the -DCPPZMQ_BUILD_TESTS=OFF is required
```shell
cd /opt
git clone https://github.com/zeromq/cppzmq.git
cd /opt/cppzmq
git checkout v4.7.1

mkdir build
cd /opt/cppzmq/build

cmake -DCPPZMQ_BUILD_TESTS=OFF ..
make install
```
