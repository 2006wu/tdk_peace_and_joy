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
