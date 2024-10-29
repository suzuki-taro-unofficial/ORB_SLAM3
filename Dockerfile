FROM ubuntu:18.04

RUN ln -sf /usr/share/zoneinfo/Asia/Tokyo /etc/localtime

RUN apt update && \
    apt upgrade -y

RUN apt install -y \
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
    libpython2.7-dev

RUN git clone --recursive https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && \
    git checkout v0.8 && \
    yes | ./scripts/install_prerequisites.sh recommended && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j && \
    sudo make install

RUN sudo ldconfig
