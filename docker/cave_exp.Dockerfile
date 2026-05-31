FROM osrf/ros:noetic-desktop-full

ARG USER_ID=1000
ARG GROUP_ID=1000
ARG CATKIN_WS_DIR
ARG HTTP_PROXY
ARG HTTPS_PROXY
ARG http_proxy
ARG https_proxy

# RUN echo "USER_ID: $USER_ID, GROUP_ID:$GROUP_ID" 

SHELL ["/bin/bash", "-c"]
ENV CATKIN_WS=$CATKIN_WS_DIR
ENV HTTP_PROXY=${HTTP_PROXY}
ENV HTTPS_PROXY=${HTTPS_PROXY}
ENV http_proxy=${http_proxy}
ENV https_proxy=${https_proxy}
RUN echo "The value of CATKIN_WS_DIR is $CATKIN_WS_DIR" 

RUN getent group "${GROUP_ID}" >/dev/null || groupadd -g "${GROUP_ID}" rosuser || true && \
    id -u "${USER_ID}" >/dev/null 2>&1 || useradd -m -u "${USER_ID}" -g "${GROUP_ID}" -s /bin/bash rosuser || true && \
    sed -i 's/\$(groups)/$(groups 2>\/dev\/null)/g' /etc/bash.bashrc || true

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    git \
    tmux \
    cmake \
    wget \
    pcl-tools \
    build-essential \
    libdw-dev \
    libtbb-dev \
    libelf-dev \
    libglew-dev \
    libglfw3-dev \
    libblosc-dev \
    libpcl-dev \
    libeigen3-dev \
    libspdlog-dev \
    libopenexr-dev \
    libarmadillo-dev \
    liblog4cplus-dev \
    libatlas-base-dev \
    libsuitesparse-dev \
    libgoogle-glog-dev \
    libignition-common3-graphics-dev \
    libignition-common3-profiler-dev \
    ros-${ROS_DISTRO}-rosfmt \
    ros-${ROS_DISTRO}-rviz \
    ros-${ROS_DISTRO}-navigation \
    ros-${ROS_DISTRO}-lms1xx \
    ros-${ROS_DISTRO}-message-filters \
    ros-${ROS_DISTRO}-tf* \
    python3-tk \
    python3-pip \
    python3-pandas \
    python3-wstool \
    python3-matplotlib \
    python3-catkin-tools && \
    rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/stevengj/nlopt.git --branch v2.7.1 && \
    cd nlopt && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j8 && \
    make install && \
    cd ../.. && \
    rm -rf nlopt 

WORKDIR $CATKIN_WS

COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh && \
    echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> /etc/bash.bashrc && \
    echo '[ -n "$CATKIN_WS" ] && [ -f "$CATKIN_WS/devel/setup.bash" ] && source "$CATKIN_WS/devel/setup.bash"' >> /etc/bash.bashrc
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
