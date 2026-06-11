################### ROS2 Base

FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

RUN apt-get update && apt-get install -y \
    ca-certificates \
    gnupg \
    lsb-release \
    sudo \
    wget \
    curl \
    git \
    vim \
    build-essential \
    cmake \
    python3-pip \
    python-is-python3 \
    gdb \
    valgrind \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

################### Python 

RUN pip3 install --no-cache-dir --upgrade pip && \
    pip3 install --no-cache-dir \
    empy==3.3.4 \
    pyros-genmsg \
    kconfiglib \
    jsonschema \
    jinja2 \
    pyserial \
    pyyaml \
    packaging \
    toml \
    numpy \
    pandas \
    future

################### Gazebo

RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update && \
    apt-get install -y gz-garden && \
    rm -rf /var/lib/apt/lists/*

################### PX4 Autopilot dependencies
    
RUN apt-get update && apt-get install -y \
    astyle \
    lcov \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    ninja-build \
    protobuf-compiler \
    libeigen3-dev \
    libopencv-dev \
    rsync \
    && rm -rf /var/lib/apt/lists/*

################### Micro-XRCE-DDS Agent

RUN cd /tmp && \
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && \
    mkdir build && cd build && \
    cmake .. && \
    make && \
    make install && \
    ldconfig /usr/local/lib/ && \
    cd / && rm -rf /tmp/Micro-XRCE-DDS-Agent

################### Environment setup

# Create workspace directory with correct ownership
WORKDIR /workspaces

RUN echo "export PATH=$PATH:/scripts" >> ~/.bashrc

COPY post_create.sh /tmp