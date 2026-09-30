FROM ros:humble-ros-base-jammy

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000
ARG TF2_FIXED_VERSION=0.25.24

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      curl \
      git \
      libapr1-dev \
      libeigen3-dev \
      libpcl-dev \
      libyaml-cpp-dev \
      python3-colcon-common-extensions \
      python3-pip \
      python3-pytest \
      python3-scipy \
      python3-yaml \
      ros-humble-gtsam \
      ros-humble-nav2-amcl \
      ros-humble-nav2-behaviors \
      ros-humble-nav2-bt-navigator \
      ros-humble-nav2-collision-monitor \
      ros-humble-nav2-controller \
      ros-humble-nav2-costmap-2d \
      ros-humble-nav2-lifecycle-manager \
      ros-humble-nav2-map-server \
      ros-humble-nav2-msgs \
      ros-humble-nav2-navfn-planner \
      ros-humble-nav2-planner \
      ros-humble-nav2-regulated-pure-pursuit-controller \
      ros-humble-pcl-conversions \
      ros-humble-pcl-ros \
      ros-humble-pointcloud-to-laserscan \
      ros-humble-robot-localization \
      ros-humble-rosbag2 \
      ros-humble-rviz2 \
      ros-humble-tf-transformations \
      ros-humble-tf2-geometry-msgs \
      ros-humble-tf2-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

# tf2 < 0.25.24 can deadlock the TF listener against a MessageFilter's
# waitForTransform (ros2/geometry2#990), freezing Nav2 TF after minutes.
RUN installed="$(dpkg-query -W -f='${Version}' ros-humble-tf2)"; \
    if dpkg --compare-versions "$installed" ge "$TF2_FIXED_VERSION"; then \
      echo "System tf2 $installed already contains the deadlock fix"; \
    else \
      source /opt/ros/humble/setup.bash \
      && curl -fsSL "https://github.com/ros2/geometry2/archive/refs/tags/$TF2_FIXED_VERSION.tar.gz" \
        | tar -xz -C /tmp \
      && cmake -S "/tmp/geometry2-$TF2_FIXED_VERSION/tf2" -B /tmp/tf2-build \
        -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
      && cmake --build /tmp/tf2-build --target tf2 -j"$(nproc)" \
      && install -m 0644 /tmp/tf2-build/libtf2.so /opt/ros/humble/lib/libtf2.so \
      && rm -rf /tmp/geometry2-* /tmp/tf2-build; \
    fi

RUN pip3 install --no-cache-dir \
      'numpy==1.23.5' 'open3d==0.18.0' transforms3d pybase64 transformations \
    && pip3 install --no-cache-dir --no-deps 'ros2-numpy==0.0.5'

COPY ros2_ws/src/Livox-SDK2 /tmp/Livox-SDK2
RUN cmake -S /tmp/Livox-SDK2 -B /tmp/livox-build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /tmp/livox-build -j"$(nproc)" \
    && cmake --install /tmp/livox-build \
    && rm -rf /tmp/Livox-SDK2 /tmp/livox-build \
    && ldconfig

COPY ros2_ws/src/Sophus /tmp/Sophus
RUN cmake -S /tmp/Sophus -B /tmp/sophus-build -DCMAKE_BUILD_TYPE=Release \
      -DSOPHUS_USE_BASIC_LOGGING=ON -DBUILD_SOPHUS_TESTS=OFF \
    && cmake --build /tmp/sophus-build -j"$(nproc)" \
    && cmake --install /tmp/sophus-build \
    && rm -rf /tmp/Sophus /tmp/sophus-build

# Match the host user so files written to the mounted project stay user-owned.
RUN if getent passwd "$USER_UID" >/dev/null; then \
      userdel -r "$(getent passwd "$USER_UID" | cut -d: -f1)"; \
    fi \
    && if ! getent group "$USER_GID" >/dev/null; then \
      groupadd --gid "$USER_GID" "$USERNAME"; \
    fi \
    && useradd --uid "$USER_UID" --gid "$USER_GID" --create-home --shell /bin/bash "$USERNAME"

USER $USERNAME
WORKDIR /workspace
