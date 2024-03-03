# Base image with cuda, cudnn and ubuntu 22.04
FROM nvidia/cuda:12.1.0-cudnn8-devel-ubuntu22.04

# Suppress interactions during build
ENV DEBIAN_FRONTEND="noninteractive" 

# Install ROS2
RUN apt update
RUN apt install software-properties-common -y
RUN add-apt-repository universe

RUN apt update && apt install curl -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update && apt install ros-dev-tools -y
RUN apt update && apt upgrade -y
RUN apt install ros-iron-desktop -y

# Install Opencv with cuda support
RUN apt install unzip
ENV OPENCV_VERSION=4.8.0
RUN cd /opt/ &&\
    # Download and unzip OpenCV and opencv_contrib and delte zip files
    wget https://github.com/opencv/opencv/archive/$OPENCV_VERSION.zip &&\
    unzip $OPENCV_VERSION.zip &&\
    rm $OPENCV_VERSION.zip &&\
    wget https://github.com/opencv/opencv_contrib/archive/$OPENCV_VERSION.zip &&\
    unzip ${OPENCV_VERSION}.zip &&\
    rm ${OPENCV_VERSION}.zip &&\
    # Create build folder and switch to it
    mkdir /opt/opencv-${OPENCV_VERSION}/build && cd /opt/opencv-${OPENCV_VERSION}/build &&\
    # Cmake configure
    cmake \
        -DOPENCV_EXTRA_MODULES_PATH=/opt/opencv_contrib-${OPENCV_VERSION}/modules \
        -DWITH_CUDA=ON \
        -DCUDA_ARCH_BIN=7.5,8.0,8.6 \
        -DCMAKE_BUILD_TYPE=RELEASE \
        # Install path will be /usr/local/lib (lib is implicit)
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        .. &&\
    # Make
    make -j"$(nproc)" && \
    # Install to /usr/local/lib
    make install && \
    ldconfig &&\
    # Remove OpenCV sources and build folder
    rm -rf /opt/opencv-${OPENCV_VERSION} && rm -rf /opt/opencv_contrib-${OPENCV_VERSION}
# Add graphics capability to allow rviz to run with GPU acceleration
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility,graphics
# https://github.com/NVIDIA/nvidia-docker/issues/856
RUN rm -f /usr/lib/x86_64-linux-gnu/libcuda.so*
RUN rm -f /usr/lib/x86_64-linux-gnu/libnvidia-*.so*

# Install apt dependencies
RUN apt update && apt-get install --no-install-recommends -y \
            python3-pip \
            build-essential \
            cmake \
            git \
            wget \
            unzip \
            yasm \
            pkg-config \
            libswscale-dev \
            libtbb2 \
            libtbb-dev \
            libjpeg-dev \
            libpng-dev \
            libtiff-dev \
            libavformat-dev \
            libpq-dev \
            libxine2-dev \
            libglfw3-dev \
            libglew-dev \
            libtiff5-dev \
            zlib1g-dev \
            libjpeg-dev \
            libavcodec-dev \
            libavformat-dev \
            libavutil-dev \
            libpostproc-dev \
            libswscale-dev \
            libeigen3-dev \
            libtbb-dev \
            libgtk2.0-dev \
            pkg-config \
            python3-dev \
            python3-numpy \
            gfortran \
            libyaml-cpp-dev \
            ffmpeg \
            tmux \
            nano \
	    vim \
            clang-format \
	    clangd \
            zsh \
            htop \
            nvtop \
            tmux \
            xterm \
            lsb-release \
            bash-completion \


# Install python dependencies
RUN pip3 install --upgrade pip  && pip3 install pybind11[global] pyyaml scipy termcolor imviz matplotlib autopep8

# install gfortran for cpp bindings of fortran spline routines
RUN apt-get -y update && apt-get install -y gfortran && apt-get clean && rm -rf /var/lib/apt/lists/*

# set entrypoint
COPY ./docker/content/docker-entrypoint-ros2.sh /usr/bin/docker-entrypoint.sh

# TODO if entrypoint is set, the welcome msg is displayed but not connected to the docker, does this has something to do with the daemon entrypoint?
ENTRYPOINT ["/usr/bin/docker-entrypoint.sh"]


