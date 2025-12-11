# NVIDIA OpenGL + GLVND på Ubuntu 22.04
FROM nvidia/opengl:1.2-glvnd-runtime-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV __GLX_VENDOR_LIBRARY_NAME=nvidia
ENV __NV_PRIME_RENDER_OFFLOAD=1
ENV __VK_LAYER_NV_optimus=NVIDIA_only
ENV QT_X11_NO_MITSHM=1
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute

# Locale
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US.UTF-8 && update-locale LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# ROS 2 Humble apt-kilde
RUN apt-get update && apt-get install -y curl gnupg lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/ros2.list

# ROS 2 + verktøy
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions python3-rosdep python3-vcstool \
    build-essential cmake git pkg-config \
    libglm-dev libsdl2-dev libfreetype6-dev \
    libpcl-dev ros-humble-pcl-ros ros-humble-pcl-conversions \
    libglib2.0-0 libsm6 libxext6 libxrender1 fonts-dejavu-core \
    && rm -rf /var/lib/apt/lists/*

# Python verktøy
RUN apt-get update && apt-get install -y \
    python3-pip python3-venv python3-dev && \
    rm -rf /var/lib/apt/lists/*

# --- Python-pakker for notebooks og analyse ---
# Behold distroens matplotlib -> pinn NumPy til 1.26.x for ABI-komp
# + pin "matplotlib-inline==0.1.6"
RUN pip3 install --no-cache-dir \
    jupyterlab notebook ipykernel ipywidgets \
    "matplotlib-inline==0.1.6" \
    tqdm pyyaml \
    "numpy==1.26.4" "pandas==2.2.*" \
    rosbags \
    scipy \
    opencv-python-headless \
    pillow \
    seaborn \
    scikit-learn \
    plotly \
    colorama \
    imageio

# Init rosdep
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