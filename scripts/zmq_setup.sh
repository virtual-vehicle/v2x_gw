#Install dependencies
echo "dependencies"
apt-get update -qq \
    && apt-get install -qq --yes --no-install-recommends \
        autoconf \
        automake \
        build-essential \
        git \
        libkrb5-dev \
        libsodium-dev \
        libtool \
        pkg-config \
    && rm -rf /var/lib/apt/lists/*
echo "dependencies - done"

#Install libzmq
echo "libzmq"
echo "libzmq - checkout repo"
[ ! -d "/opt" ] && mkdir -p "/opt"
cd /opt
git clone https://github.com/zeromq/libzmq.git
cd /opt/libzmq
git checkout v4.3.4

echo "libzmq - install dependencies"
apt-get update -qq
apt-get install -qq --yes --no-install-recommends \
libkrb5-dev \
libsodium23

echo "libzmq - generate install environment"
./autogen.sh

echo "libzmq - configure install environment"
./configure --prefix=/usr/local --with-libgssapi_krb5

echo "libzmq - make, make check"
make
make check

echo "libzmq - install"
make install

echo "libzmq - check output"
ldconfig && ldconfig -p | grep libzmq

echo "libzmq - done"

echo "cppzmq"
echo "cppzmq - checkout git"
[ ! -d "/opt" ] && mkdir -p "/opt"
cd /opt
git clone https://github.com/zeromq/cppzmq.git
cd /opt/cppzmq
git checkout v4.7.1

echo "cppzmq - create build"
mkdir build
cd /opt/cppzmq/build

echo "cppzmq - cmake"
cmake -DCPPZMQ_BUILD_TESTS=OFF ..

echo "cppzmq - make install"
make install

echo "cppzmq - done"
