FROM ubuntu:18.04

RUN ln -sf /usr/share/zoneinfo/Asia/Tokyo /etc/localtime

RUN apt-get update && apt-get install -y  \
    libboost-all-dev libeigen3-dev \
    git cmake curl unzip \
    build-essential \
    libglfw3-dev \
    libepoxy-dev \
    sudo \
    libopencv-dev \
    doxygen graphviz \
    openssl \
    libssl-dev \
    libpython2.7-dev \
    # From Pangolin v0.8 recommended
    libgl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols libegl1-mesa-dev \
    libc++-dev libglew-dev libeigen3-dev cmake g++ ninja-build \
    libjpeg-dev libpng-dev \
    libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev && \
    # Reduce image size
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN git clone \
    --branch v0.8 \
    --depth 1 \
    --recursive \
    --shallow-submodules \
    https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j && \
    sudo make install

# Examples doesn't works without this
RUN ldconfig
