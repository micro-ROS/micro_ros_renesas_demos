<br/>
<a>
   <p align="center">
      <img width="40%" src=".images/renesas_logo.gif">
      <img width="40%" src=".images/microros_logo.png">
   </p>
</a>
<br/>

# micro-ROS motor demo for Renesas e<sup>2</sup> studio

[![micro-ROS HIL tests](https://github.com/micro-ROS/micro_ros_renesas_testbench/actions/workflows/ci.yml/badge.svg)](https://github.com/micro-ROS/micro_ros_renesas_testbench/actions/workflows/ci.yml)
[![micro-ROS HIL agent](https://github.com/micro-ROS/micro_ros_renesas_testbench/actions/workflows/build_agent.yml/badge.svg)](https://github.com/micro-ROS/micro_ros_renesas_testbench/actions/workflows/build_agent.yml)

This branch provides an example project for motor control using [micro-ROS](https://micro.ros.org/) in [Renesas e<sup>2</sup> studio](https://www.renesas.com/us/en/software-tool/e-studio).

- [micro-ROS motor demo for Renesas e<sup>2</sup> studio](#micro-ros-motor-demo-for-renesas-esup2sup-studio)
  - [Target platform](#target-platform)
  - [Requirements](#requirements)
  - [Getting started](#getting-started)
  - [Using the micro-ROS Agent](#using-the-micro-ros-agent)
  - [Using ROS2 graph utils](#using-ros2-graph-utils)
  - [License](#license)
  - [Known Issues / Limitations](#known-issues--limitations)

---
## Target platform

| MCU | Family | Reference board | Transports |
| --- | ------ | --------------- | ---------- |
| [RA6T2](https://www.renesas.com/us/en/products/microcontrollers-microprocessors/ra-cortex-m-mcus/ra6t2-240mhz-arm-cortex-m33-trustzone-high-real-time-engine-motor-control)     | RA Series | [MCK-RA6T2](https://www.renesas.com/us/en/products/microcontrollers-microprocessors/ra-cortex-m-mcus/rtk0ema270s00020bj-mck-ra6t2-renesas-flexible-motor-control-kit-ra6t2-mcu-group) | CAN FD |

## Requirements

- [Renesas e<sup>2</sup> studio](https://www.renesas.com/us/en/software-tool/e-studio) for Linux<sup>1</sup>
- FSP board packs for Renesas e<sup>2</sup> studio: [Details](fps_install_packs.md).
- GNU Arm Embedded Toolchain v10.3.1.20210824 (Other compatible toolchain may work).
- [Install colcon](https://colcon.readthedocs.io/en/released/user/installation.html) and dependencies, for example with:

```bash
pip3 install colcon-common-extensions catkin_pkg lark-parser empy
```

*<sup>1</sup> Currently only support for Linux is available*

## Getting started

For using this demo, just clone recursively this repository branch in your computer with:

```bash
git clone -b demo_motor --recurse-submodules https://github.com/micro-ROS/micro_ros_renesas_demos
```

Open Renesas e<sup>2</sup> studio, import the project inside `micro_ros_motor_demo`, and finally build and flash

Once the communication is stablished, the following topics are available:

- `/motor/speed`: motor output speed on rpm (Float32).
- `/motor/joint`: motor output position/velocity with gear reduction applied (TF2 joint message).
- `/motor/cmd`: motor input velocity commands (Float32).
  The motor can be controled from a ROS2 interface with this topic:

  ```bash
  ros2 topic pub /motor/cmd std_msgs/Float32 "data: 500" -t 1
  ```

## Using the micro-ROS Agent

To start a dockerized **micro-ROS Agent** instance using CAN FD transport:

```bash
# CAN FD micro-ROS Agent
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO canfd --dev [YOUR CAN INTERFACE] -v6
```

micro-ROS Agent can also be build from source in a ROS 2 environment: [Details](https://micro.ros.org/docs/tutorials/core/first_application_linux/).

## Using ROS2 graph utils

A dockerized graph and RViz instance are included in this demo, to start them:
```bash
# Give permissions to Docker to access the X display server
xhost +

# Initialize docker images
docker-compose up -d
```

To stop the applications just down the Docker Compose:
```bash
docker-compose down
```

## License

This repository is open-sourced under the Apache-2.0 license. See the [LICENSE](LICENSE) file for details.
To find a list of other open-source components included in this repository,
see the file [3rd-party-licenses.txt](3rd-party-licenses.txt).

## Known Issues / Limitations

There are no known limitations.
