# NVIDIA OpenGL + GLVND på Ubuntu 22.04
FROM nvidia/opengl:1.2-glvnd-runtime-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
# Hint til GLVND for å velge NVIDIA-vendor når verten kjører Intel/Optimus
ENV __GLX_VENDOR_LIBRARY_NAME=nvidia
ENV __NV_PRIME_RENDER_OFFLOAD=1
ENV __VK_LAYER_NV_optimus=NVIDIA_only
# Nyttig ved X11 i container
ENV QT_X11_NO_MITSHM=1
# Gjør at nvidia-container-toolkit monterer riktige capabilites (kan også settes ved run)
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute

# Locale (ROS-installasjon liker dette)
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US.UTF-8 && update-locale LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# ROS 2 Humble apt-kilde
RUN apt-get update && apt-get install -y curl gnupg lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/ros2.list

# ROS 2 + verktøy og libs du trenger
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions python3-rosdep python3-vcstool \
    build-essential cmake git pkg-config \
    libglm-dev libsdl2-dev libfreetype6-dev \
    libpcl-dev ros-humble-pcl-ros ros-humble-pcl-conversions \
    && rm -rf /var/lib/apt/lists/*

# Init rosdep (idempotent)
RUN rosdep init || true && rosdep update

# Shell-miljø
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    printf '\n[ -f /root/stonefish_ros2_ws/install/setup.bash ] && source /root/stonefish_ros2_ws/install/setup.bash\n' >> /root/.bashrc

# ---- Bygg og installer Stonefish (system) ----
ARG STONEFISH_REF=master
WORKDIR /root
RUN git clone https://github.com/patrykcieslak/stonefish.git && \
    cd stonefish && git checkout ${STONEFISH_REF} && \
    mkdir build && cd build && \
    cmake .. && make -j"$(nproc)" && make install && ldconfig

# ROS2-workspace
WORKDIR /root/stonefish_ros2_ws
