ARG ROS1_IMAGE=erl/ros-noetic:cpu-common
FROM ${ROS1_IMAGE}


ARG TZ=America/Los_Angeles
ARG LOCALE=en_US.UTF-8
ARG APT_MIRROR=""
ARG ROS_APT_SOURCE_VERSION=1.1.0

ENV TZ=${TZ}
ENV LANG=${LOCALE}
ENV LANGUAGE=${LOCALE}
ENV LC_ALL=${LOCALE}

RUN cp /etc/apt/sources.list /etc/apt/sources.list.backup &&\
    if [ -n "$APT_MIRROR" ]; then \
    sed -i "s|http://archive.ubuntu.com/ubuntu/|$APT_MIRROR|g" /etc/apt/sources.list; \
    fi
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone && \
    DEBIAN_FRONTEND=noninteractive apt update && apt install -y locales && locale-gen en_US.UTF-8

RUN DEBIAN_FRONTEND=noninteractive apt update -y && apt install -y \
    software-properties-common \
    curl

RUN add-apt-repository universe
RUN curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
RUN dpkg -i /tmp/ros2-apt-source.deb

# https://docs.ros.org/en/humble/How-To-Guides/Using-ros1_bridge-Jammy-upstream.html
# the only way to have ros1_bridge built is to have ROS2 built from source.

# install dependencies

RUN DEBIAN_FRONTEND=noninteractive apt update -y && apt upgrade -y && apt install -y \
    python3-flake8-docstrings \
    python3-pip \
    python3-pytest-cov \
    ros-dev-tools \
    build-essential \
    cmake \
    git \
    python3-flake8 \
    python3-flake8-docstrings \
    python3-pip \
    python3-pytest \
    python3-pytest-cov \
    python3-rosdep \
    python3-setuptools \
    wget
RUN python3 -m pip install -U \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-import-order \
    flake8-quotes \
    "pytest>=5.3" \
    pytest-repeat \
    pytest-rerunfailures \
    empy==3.3.4 \
    colcon-common-extensions \
    vcstool

# https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html
# add ROS2 source
WORKDIR /root/ros2_humble
RUN mkdir -p src && vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src
RUN rosdep update && rosdep install --from-paths src --ignore-src -y --skip-keys \
    "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"
RUN colcon build --merge-install --install-base /opt/ros/humble --cmake-args -DCMAKE_INSTALL_PREFIX=/opt/ros/humble

# build ros1_bridge
WORKDIR /root/ws_ros1_bridge
RUN mkdir -p src && cd src && git clone https://github.com/ros2/ros1_bridge.git
RUN . /opt/ros/noetic/setup.sh && . /opt/ros/humble/setup.sh && \
    colcon build --merge-install --install-base /opt/ros1_bridge --cmake-args -DCMAKE_INSTALL_PREFIX=/opt/ros1_bridge

# clean
WORKDIR /root
RUN apt-get clean  && rm -rf /var/lib/apt/lists/*
RUN if [ -n "$APT_MIRROR" ]; then \
    mv /etc/apt/sources.list.backup /etc/apt/sources.list; \
    fi
RUN rm -rf /root/ros2_humble && rm -rf /root/ws_ros1_bridge

# set environment variables (it is better to set them from the docker run command)
# ENV ROS_MASTER_URI=http://127.0.0.1:11311
# ENV ROS_IP=127.0.0.1
# ENV ROS_HOSTNAME=127.0.0.1

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
