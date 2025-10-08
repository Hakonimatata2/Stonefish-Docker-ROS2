# Stonefish ROS 2 BlueROV2 Simulation

Dette repoet beskriver hvordan du bygger og kjører en BlueROV2-simulering i ROS 2 (Humble) med Stonefish-simulatoren.

## Installer NVIDIA container tools for docker

Følg instruksene på siden deres.

## Bygg Docker-image

docker build -t stonefish-ros2:humble .

## Kjør container

Installer VS Code-utvidelsene:

- Docker

- Dev Containers

Kjør i terminalen:

```
export XSOCK=/tmp/.X11-unix
export XAUTH=$HOME/.Xauthority
xhost +si:localuser:root
docker run -it --rm --gpus all \
  -e DISPLAY=$DISPLAY \
  -e XAUTHORITY=$XAUTH \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute \
  -e __NV_PRIME_RENDER_OFFLOAD=1 \
  -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
  -e __VK_LAYER_NV_optimus=NVIDIA_only \
  -v $XSOCK:$XSOCK:rw \
  -v $XAUTH:$XAUTH:ro \
  -v $HOME/stonefish_ros2_ws:/root/stonefish_ros2_ws \
  --ipc=host \
  --name stonefish \
  stonefish-ros2:humble \
  /bin/bash
```

Åpne Docker-fanen i VS Code og attatch container to VS Code.

### Test GPU
Kjør i container terminalen:

apt-get update && apt-get install -y mesa-utils
glxinfo -B | egrep 'OpenGL vendor|OpenGL renderer'

Renderer skal være satt til NVIDIA og ikke Intel

## Klon Stonefish ROS 2-bridge

I containeren:

mkdir -p ~/stonefish_ros2_ws/src
cd ~/stonefish_ros2_ws/src
git clone https://github.com/patrykcieslak/stonefish_ros2.git

### Endring i kildekoden (ROS2Interface.cpp)

Åpne stonefish_ros2/src/ROS2Interface.cpp og legg til følgende inkludering øverst:

```
#include <cstdint>   // for int64_t
```

Erstatt så funksjonen fra linje 680 med:

```
void ROS2Interface::PublishEventBasedCamera(rclcpp::PublisherBase::SharedPtr pub, EventBasedCamera* ebc)
{
    int32_t* data = (int32_t*)ebc->getImageDataPointer();
    stonefish_ros2::msg::EventArray msg;

    msg.header.frame_id = ebc->getName();
    msg.header.stamp = nh_->get_clock()->now();
    ebc->getResolution(msg.width, msg.height);

    msg.events.resize(ebc->getLastEventCount());
    for (size_t i = 0; i < msg.events.size(); ++i)
    {
        msg.events[i].x = (unsigned int)(data[i * 2] >> 16);
        msg.events[i].y = (unsigned int)(data[i * 2] & 0xFFFF);

        const rclcpp::Time base(msg.header.stamp);
        const rclcpp::Duration dt(0, std::abs(data[i * 2 + 1]));
        const rclcpp::Time t = dt + base;
        const int64_t nsec = t.nanoseconds();
        msg.events[i].ts.sec = static_cast<int32_t>(nsec / 1000000000LL);
        msg.events[i].ts.nanosec = static_cast<uint32_t>(nsec % 1000000000LL);

        msg.events[i].polarity = data[i * 2 + 1] > 0;
    }

    std::static_pointer_cast<
        rclcpp::Publisher<stonefish_ros2::msg::EventArray>>(pub)->publish(msg);
}
```

## Sett opp egen ROS 2-pakke for BlueROV2-simulatoren

Denne fikser det som er under.

cd src
ros2 pkg create --build-type ament_cmake "package_name" --dependencies rclcpp

Mappestruktur:

<pre>
stonefish_ros2_ws/
├─ src/
│  ├─ stonefish_bluerov2/
│  |  ├─ CMakeLists.txt
│  |  ├─ package.xml
│  |  ├─ launch/
│  |  │  └─ sim.py
│  |  ├─ data/
│  |  └─ scenarios/
│  |     ├─ robot.scn
│  |     └─ scenario1.scn
│  └─ stonefish_ros2/
├─ build/
├─ install/
└─ log/
</pre>

### CMakeLists.txt
```
cmake_minimum_required(VERSION 3.8)
project(stonefish_bluerov2)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch scenarios data
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

### package.xml

```
<?xml version="1.0"?>
<package format="3">
  <name>stonefish_bluerov2</name>
  <version>0.0.1</version>
  <description>BlueROV2 scenario and launch files for Stonefish ROS 2</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>stonefish_ros2</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```
ro
## Bygg og installer

cd ~/stonefish_ros2_ws
colcon build --symlink-install
source install/setup.bash


### Hvis du får feil, prøv:

rm -rf build install log

kjør forrige steg på nytt

## Kjør simuleringen
ros2 launch stonefish_bluerov2 sim.py

## Ved nye filer / endringer
Kjør "colcon build" for å bygge pakkene på nytt