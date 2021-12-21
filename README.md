<br/>
<a>
   <p align="center">
      <img width="40%" src=".images/renesas_logo.gif">
      <img width="40%" src=".images/microros_logo.png">
   </p>
</a>
<br/>

# micro-ROS demos for Renesas e<sup>2</sup> studio

[![micro-ROS HIL tests](https://github.com/micro-ROS/micro_ros_renesas_testbench/actions/workflows/ci.yml/badge.svg)](https://github.com/micro-ROS/micro_ros_renesas_testbench/actions/workflows/ci.yml)
[![micro-ROS HIL agent](https://github.com/micro-ROS/micro_ros_renesas_testbench/actions/workflows/build_agent.yml/badge.svg)](https://github.com/micro-ROS/micro_ros_renesas_testbench/actions/workflows/build_agent.yml)

This package provides example projects for using [micro-ROS](https://micro.ros.org/) in a [Renesas e<sup>2</sup> studio](https://www.renesas.com/us/en/software-tool/e-studio). All demos target [Renesas RA family](https://www.renesas.com/us/en/products/microcontrollers-microprocessors/ra-cortex-m-mcus), an ARM Cortex-M based MCU series, enabling a full micro-ROS compatibility for developing robotics and IoT applications.

- [micro-ROS demos for Renesas e<sup>2</sup> studio](#micro-ros-demos-for-renesas-esup2sup-studio)
  - [Target platform](#target-platform)
  - [Requirements](#requirements)
  - [Available demos](#available-demos)
  - [Getting started](#getting-started)
  - [Using the micro-ROS Agent](#using-the-micro-ros-agent)
  - [License](#license)
  - [Known Issues / Limitations](#known-issues--limitations)

---
## Target platform

| MCU                                                                                                                                                                             | Family    | Reference board                                                                                                                              |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------- | -------------------------------------------------------------------------------------------------------------------------------------------- |
| [RA6M5](https://www.renesas.com/us/en/products/microcontrollers-microprocessors/ra-cortex-m-mcus/ra6m5-200mhz-arm-cortex-m33-trustzone-highest-integration-ethernet-and-can-fd) | RA Series | [EK-RA6M5](https://www.renesas.com/us/en/products/microcontrollers-microprocessors/ra-cortex-m-mcus/ek-ra6m5-evaluation-kit-ra6m5-mcu-group) |

## Requirements

- [Renesas e<sup>2</sup> studio](https://www.renesas.com/us/en/software-tool/e-studio) for Linux<sup>1</sup>
- FSP board packs for Renesas e<sup>2</sup> studio: [Details](fps_install_packs.md).
- GNU Arm Embedded Toolchain v10.3.1.20210824 (Other compatible toolchain may work).
- [Install colcon](https://colcon.readthedocs.io/en/released/user/installation.html) and dependencies, for example with:

```bash
pip3 install colcon-common-extensions catkin_pkg lark-parser empy
```

*<sup>1</sup> Currently only support for Linux is available*
## Available demos

| RTOS                                                        | Transport | Description                                            | Folder                                             |
| ----------------------------------------------------------- | --------- | ------------------------------------------------------ | -------------------------------------------------- |
| Bare Metal                                                  | CAN FD    | micro-ROS using a CAN FD transport                     | [`micro_ros_can`](micro_ros_can)                 |
| Bare Metal                                                  | UART      | micro-ROS using a serial UART transport                | [`micro_ros_uart`](micro_ros_uart)                 |
| Bare Metal                                                  | USB       | micro-ROS using a serial USB-CDC transport             | [`micro_ros_usb`](micro_ros_usb)                   |
| [FreeRTOS](https://www.freertos.org/)                       | UDP       | micro-ROS using a network transport and FreeRTOS + TCP | [`micro_ros_udp_freertos`](micro_ros_udp_freertos) |
| [ThreadX](https://azure.microsoft.com/en-us/services/rtos/) | UDP       | micro-ROS using a network transport and ThreadX + NetX | [`micro_ros_udp_threadx`](micro_ros_udp_threadx)   |

## Getting started

For using those demos, just clone recursively this repository in your computer with:

```bash
git clone --recurse-submodules https://github.com/micro-ROS/micro_ros_renesas_demos
```

Open Renesas e<sup>2</sup> studio, import some of them, and finally build and flash

## Using the micro-ROS Agent

It is possible to use a **micro-ROS Agent** just by using this docker command:

```bash
# UDPv4 micro-ROS Agent
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO udp4 --port 8888 -v6

# Serial micro-ROS Agent
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO serial --dev [YOUR BOARD PORT] -v6

# CAN FD micro-ROS Agent
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO canfd --dev [YOUR CAN INTERFACE] -v6
```

There are some other options for using the micro-ROS Agent:
 - Building it in a ROS 2 environment: [Details](https://micro.ros.org/docs/tutorials/core/first_application_linux/).
 - Using a [snap package](https://snapcraft.io/micro-ros-agent).
## License

This repository is open-sourced under the Apache-2.0 license. See the [LICENSE](LICENSE) file for details.
To find a list of other open-source components included in this repository,
see the file [3rd-party-licenses.txt](3rd-party-licenses.txt).

## Known Issues / Limitations

There are no known limitations.
