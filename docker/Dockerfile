# =========================
# 視覺相機：ROS Humble 基底
# =========================
FROM ros:humble AS base_ros_humble
ARG DEBIAN_FRONTEND=noninteractive
ENV TERM=xterm-256color
# 這兩行請確認你有這些腳本（相對於 build context）
COPY ./scripts/build/ /tmp/
COPY ./scripts/entrypoint/ros_entrypoint.sh /ros_entrypoint.sh
RUN chown root:root /ros_entrypoint.sh && \
    chmod 755 /ros_entrypoint.sh && \
    sh /tmp/install_depend.sh

# -------------- librealsense builder --------------
FROM base_ros_humble AS librealsense-builder
ARG LIBREALSENSE_VERSION=2.55.1
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    cmake pkg-config build-essential python3 python3-dev ca-certificates \
    libssl-dev libusb-1.0-0-dev libudev-dev libgtk-3-dev \
 && apt-get clean -y && rm -rf /var/lib/apt/lists/*
WORKDIR /usr/src
RUN curl -L https://codeload.github.com/IntelRealSense/librealsense/tar.gz/refs/tags/v${LIBREALSENSE_VERSION} -o librealsense.tar.gz && \
    tar -zxf librealsense.tar.gz && rm librealsense.tar.gz && \
    ln -s /usr/src/librealsense-${LIBREALSENSE_VERSION} /usr/src/librealsense
RUN cd /usr/src/librealsense && mkdir build && cd build && \
    cmake -DPYTHON_EXECUTABLE=$(which python3) \
          -DCMAKE_INSTALL_PREFIX=/opt/librealsense \
          -DBUILD_GRAPHICAL_EXAMPLES=OFF \
          -DBUILD_PYTHON_BINDINGS:bool=true \
          -DCMAKE_BUILD_TYPE=Release ../ && \
    make -j"$(($(nproc)-1))" all && make install

# -------------- realsense runtime（ROS 2 相機容器） --------------
FROM base_ros_humble AS realsense
ARG REALSENSE_ROS_VERSION=4.55.1
ARG USER=camera
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG DEBIAN_FRONTEND=noninteractive
ENV ROS_WS_PATH=/home/$USER/ros2_ws

# 從 builder 複製二進位與規則
COPY --from=librealsense-builder /opt/librealsense /usr/local/
COPY --from=librealsense-builder /usr/lib/python3/dist-packages/pyrealsense2 /usr/lib/python3/dist-packages/pyrealsense2
COPY --from=librealsense-builder /usr/src/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/
COPY --from=librealsense-builder /usr/src/librealsense/config/99-realsense-d4xx-mipi-dfu.rules /etc/udev/rules.d/
ENV PYTHONPATH=$PYTHONPATH:/usr/local/lib

RUN apt-get update && apt-get install -y --no-install-recommends \
    libusb-1.0-0 udev ca-certificates \
    ros-humble-diagnostic-updater ros-humble-xacro \
    python3-tqdm python3-requests \
    libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at \
 && apt-get clean -y && rm -rf /var/lib/apt/lists/*

# 建使用者、初始化 rosdep
RUN sh /tmp/setup_user.sh $USER $USER_UID $USER_GID
ENTRYPOINT [ "/ros_entrypoint.sh" ]
USER $USER
RUN mkdir -p $ROS_WS_PATH/src && \
    git clone --branch $REALSENSE_ROS_VERSION https://github.com/IntelRealSense/realsense-ros.git $ROS_WS_PATH/src/realsense-ros && \
    sh /tmp/rosdep_init.sh $USER
WORKDIR $ROS_WS_PATH
# ============================================
# Base Stage
# ============================================
FROM ubuntu:22.04 AS base

LABEL maintainer="wuchengzhi20060819@gmail.com"

ENV LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

SHELL ["/bin/bash", "-c"]

RUN apt-get update && \
    apt-get install -y locales && \
    locale-gen en_US.UTF-8 && \
    update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 && \
    useradd -ms /bin/bash user

# ============================================
# Build Stage
# ============================================
FROM base AS build

RUN apt-get update && apt-get install -y \
    curl \
    git \
    lsb-release \
    software-properties-common \
    build-essential \
    cmake \
    python3-pip \
    python3-rosdep \
    python3-setuptools \
    ros-dev-tools \
    python3-numpy \
    libspdlog-dev

RUN python3 -m pip install -U colcon-common-extensions vcstool

# Add ROS 2 repo and key
RUN add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    apt-get install -y ros-core-dev

# Clone and build ROS 2 Humble
RUN mkdir -p /ros2_humble/src && cd /ros2_humble && \
    vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src && \
    rosdep init && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y --rosdistro humble --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

WORKDIR /ros2_humble
RUN colcon build

# Install ROS 1 Noetic
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1.list && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - && \
    apt-get update && \
    apt-get install -y ros-noetic-ros-core

# Clone and build ros1_bridge
RUN git clone https://github.com/ros2/ros1_bridge src/ros1_bridge && \
    source /opt/ros/noetic/setup.bash && \
    source install/setup.bash && \
    colcon build --packages-select ros1_bridge --cmake-force-configure

COPY ./start-bridge.sh /start-bridge.sh
RUN chmod +x /start-bridge.sh

# ============================================
# Release Stage
# ============================================
FROM base AS release

RUN apt-get update && apt-get install -y \
    ros-noetic-ros-core \
    ros-noetic-rostopic \
    ros-noetic-rosgraph-msgs \
    python3-rospy \
    python3-rosgraph \
    python3-catkin-pkg \
    ros-core-dev \
    libspdlog-dev \
    python3-packaging \
    python3-numpy

COPY --from=build /ros2_humble/install /ros2_humble/install
COPY ./start-bridge.sh /start-bridge.sh

USER user

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_humble/install/setup.bash" >> ~/.bashrc

WORKDIR /home/user
CMD ["/bin/bash", "./start-bridge.sh"]
