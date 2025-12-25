# ROS2 Humble Builder stage (builds g2o and the ROS workspace)
FROM ros:humble AS builder

ENV DEBIAN_FRONTEND=noninteractive
ENV WORKSPACE_PATH=/root/workspace
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV IGN_RENDERING_API=ogre

# Remove any stale ROS apt sources/keys shipped by base image to avoid GPG validation errors,
# then install curl (needed to fetch the official ROS apt key)
RUN rm -f /etc/apt/sources.list.d/ros2.list \
    /etc/apt/sources.list.d/ros2-latest.list \
    /etc/apt/sources.list.d/ros-latest.list && \
    rm -f /usr/share/keyrings/ros2-archive-keyring.gpg \
    /usr/share/keyrings/ros-archive-keyring.gpg \
    /usr/share/keyrings/ros2-latest-archive-keyring.gpg || true && \
    apt-get update && apt-get install -y --no-install-recommends \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Add ROS apt signing key and repository, then install remaining build packages
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common \
    vim \
    python3-pip \
    python3-tk \
    build-essential \
    cmake \
    git \
    ccache \
    pkg-config \
    make \
    libeigen3-dev \
    libspdlog-dev \
    libsuitesparse-dev \
    qtdeclarative5-dev \
    qt5-qmake \
    libqglviewer-dev-qt5 \
    ros-$ROS_DISTRO-cyclonedds \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

# Add updated mesa drivers (kept separate to preserve parity with original behavior)
RUN add-apt-repository -y ppa:kisak/kisak-mesa && apt-get update && apt-get upgrade -y && rm -rf /var/lib/apt/lists/*

# Source ROS by default for interactive shells
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

# Copy workspace and scripts into the builder
COPY workspace/ $WORKSPACE_PATH/src/
COPY scripts/setup/ /root/scripts/setup

# Install rosdep dependencies
# NOTE: ros_gz packages sometimes aren't available on the ROS apt index for all platforms.
# Skip ros-gz here so the build can continue — if you need ros_gz for simulation, install
# it on the host or add the appropriate apt repo later.
RUN rosdep update && cd $WORKSPACE_PATH && \
    rosdep install --from-paths src -y --ignore-src --rosdistro=$ROS_DISTRO --skip-keys="gazebo,ros-gz,ros_gz" || \
    (echo "Warning: some rosdeps failed to install, continuing build. Missing deps may be optional (e.g. ros_gz)." && true)

# Build g2o (shallow clone + ccache) and install into /usr/local
WORKDIR /opt
RUN git clone --depth=1 https://github.com/RainerKuemmerle/g2o.git && \
    cd g2o && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
      -DG2O_BUILD_EXAMPLES=OFF \
      -DG2O_BUILD_APPLICATIONS=OFF \
      -DCMAKE_CXX_COMPILER_LAUNCHER=ccache && \
    make -j$(nproc) && make install

# Tune ccache (keeps cache size reasonable)
RUN ccache --max-size=2G || true

# Build workspace using ccache (same final packages, only faster builds)
RUN export CCACHE_DIR=/root/.ccache && \
    mkdir -p /root/.ccache && \
    cd $WORKSPACE_PATH && \
    echo "=== Workspace contents ===" && \
    find src -name "package.xml" -exec dirname {} \; && \
    echo "=== Building workspace ===" && \
    bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --cmake-args '-DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER_LAUNCHER=ccache'" && \
    echo "=== Build completed, checking install directory ===" && \
    ls -la install/ && \
    echo "=== Checking for setup.bash ===" && \
    ls -la install/setup.bash && \
    echo "=== Checking limo_simulation install ===" && \
    find install/limo_simulation -type f && \
    echo "=== Checking for launch files ===" && \
    find install/ -name "*.launch.py" && \
    echo "source $WORKSPACE_PATH/install/setup.bash" >> /root/.bashrc

# Final runtime stage (smaller image, copy only what we need)
FROM ros:humble

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV WORKSPACE_PATH=/root/workspace
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV IGN_RENDERING_API=ogre

# Remove any stale ROS apt sources/keys (if present), ensure ROS apt key and repo are present, then install runtime packages in one layer and clean apt cache
RUN rm -f /etc/apt/sources.list.d/ros2.list \
    /etc/apt/sources.list.d/ros2-latest.list \
    /etc/apt/sources.list.d/ros-latest.list && \
    rm -f /usr/share/keyrings/ros2-archive-keyring.gpg \
    /usr/share/keyrings/ros-archive-keyring.gpg \
    /usr/share/keyrings/ros2-latest-archive-keyring.gpg || true && \
    apt-get update && apt-get install -y --no-install-recommends curl && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common \
    vim \
    python3-pip \
    python3-tk \
    mesa-utils \
    libsuitesparse-dev \
    ros-$ROS_DISTRO-teleop-twist-keyboard \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-ros-gz-sim \
    ros-$ROS_DISTRO-ros-gz-bridge \
    x11-apps \
    ros-$ROS_DISTRO-cyclonedds \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-xacro \
    && rm -rf /var/lib/apt/lists/*

# Copy built workspace and g2o install from the builder
COPY --from=builder /root/workspace/install/ $WORKSPACE_PATH/install/
COPY --from=builder /usr/local/ /usr/local/

# Verify the workspace was copied correctly
RUN echo "=== Verifying copied workspace ===" && \
    ls -la $WORKSPACE_PATH/install/ && \
    if [ -f "$WORKSPACE_PATH/install/setup.bash" ]; then \
        echo "setup.bash found"; \
    else \
        echo "ERROR: setup.bash not found!"; \
        exit 1; \
    fi

# Copy setup scripts
COPY --from=builder /root/scripts/setup /root/scripts/setup

# Source ROS and workspace by default - but only if the files exist
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc && \
    echo "export LD_LIBRARY_PATH=/usr/local/lib:\$LD_LIBRARY_PATH" >> /root/.bashrc && \
    if [ -f "$WORKSPACE_PATH/install/setup.bash" ]; then \
        echo "source $WORKSPACE_PATH/install/setup.bash" >> /root/.bashrc; \
    else \
        echo "echo 'Warning: Workspace setup.bash not found'" >> /root/.bashrc; \
    fi

# Ensure workspace path exists (keeps behavior consistent)
WORKDIR /root
