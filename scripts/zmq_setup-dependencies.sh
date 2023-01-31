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

apt-get update -qq
apt-get install -qq --yes --no-install-recommends \
    libkrb5-dev \
    libsodium23
echo "dependencies - done"