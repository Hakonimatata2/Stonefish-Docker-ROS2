# ROS 2 Humble + Ubuntu 22.04 (Jammy)
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
# Litt mindre krøll med X11 i container
ENV QT_X11_NO_MITSHM=1

# Grunnverktøy og avhengigheter
RUN apt-get update && apt-get install -y \
    build-essential cmake git pkg-config \
    libglm-dev libsdl2-dev libfreetype6-dev \
    python3-pip python3-colcon-common-extensions python3-rosdep python3-vcstool \
    # Visualisering / bildeverktøy
    ros-humble-rviz2 \
    ros-humble-rqt ros-humble-rqt-image-view \
    ros-humble-image-tools ros-humble-image-transport \
    # PCL (noen av rosdep'ene roper på dette)
    libpcl-dev ros-humble-pcl-ros ros-humble-pcl-conversions \
    && rm -rf /var/lib/apt/lists/*

# Init rosdep (idempotent)
RUN rosdep init || true && rosdep update

# Ha ROS i shell som standard + auto-source workspace hvis bygget
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    printf '\n# Source workspace overlay hvis det finnes\n[ -f /root/stonefish_ros2_ws/install/setup.bash ] && source /root/stonefish_ros2_ws/install/setup.bash\n' >> /root/.bashrc

# ---- Bygg og installer Stonefish i bildet ----
ARG STONEFISH_REF=master    # sett til tag/branch ved behov, f.eks. v2.0.0
WORKDIR /root
RUN git clone https://github.com/patrykcieslak/stonefish.git && \
    cd stonefish && git checkout ${STONEFISH_REF} && \
    mkdir build && cd build && \
    cmake .. && \
    make -j"$(nproc)" && \
    make install && ldconfig

# Arbeidskatalog for ROS2-workspace
WORKDIR /root/stonefish_ros2_ws