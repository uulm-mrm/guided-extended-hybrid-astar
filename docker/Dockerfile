ARG MODULE_NAME

FROM nvidia/cuda:12.1.0-cudnn8-devel-ubuntu22.04

ARG MODULE_NAME

ENV MODULE_NAME=$MODULE_NAME

RUN mkdir -p /opt/$MODULE_NAME/
WORKDIR /opt/$MODULE_NAME

# copy requirements in root folder
COPY requirements.txt /opt/$MODULE_NAME

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
            nano

RUN pip3 install --upgrade pip  && pip3 install -r requirements.txt

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

# replace entrypoint
COPY ./docker/content/ros_entrypoint.sh /
