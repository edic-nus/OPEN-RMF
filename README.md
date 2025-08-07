# How to set up micro-ROS with PlatformIO on WSL2 (or Linux)

## System requirements:
The system was tested on:
* Ubuntu 24.04 LTS
* WSL 2
* ROS2 - Jazzy
* [ESP32-S3 3.16inch Display Development Board](https://www.waveshare.com/product/mcu-tools/development-boards/esp32/esp32-s3-lcd-3.16.htm)

## Installation
Before installing, make sure that **no ROS2 binaries are sourced** as it will disrupt the installation of micro-ROS when opening the PlatformIO project.
1. Install Visual Studio Code on your machine [here](https://code.visualstudio.com/download).
2. Open Visual Studio and install [PlatformIO IDE for VSCode](https://platformio.org/install/ide?install=vscode)
3. PlatformIO needs `git`, `cmake` and `pip3` to handle micro-ROS internal dependencies:
```shell 
apt install -y git cmake python3-pip
```
4. Set up the environment by running the following:
```shell
mkdir esp32Project
cd esp32Project
git clone https://github.com/edic-nus/OPEN-RMF.git -b esp32 .
code .
```


## How to flash onto ESP32

1. Plug a USB-C cable from your machine to the UART port of the ESP32.

    For WSL users, support for connecting USB devices is not natively available in WSL, so you will need to install the open-source [usbipd-win](https://learn.microsoft.com/en-us/windows/wsl/connect-usb) project and then attach the port from your Windows instance to WSL2.
2. In your VSCode project, [set the upload port](https://electropeak.com/learn/how-to-select-port-and-change-baud-rate-in-platform-io/) to the port connected to the ESP32.

3. Run the VSCode command `PlatformIO: Upload and Monitor`.

## How to connect micro-ROS to ROS2
Refer to [this issue](https://github.com/edic-nus/OPEN-RMF/issues/10).

The micro-ROS topic should be able to be discovered on the ROS Domain ID located in `include/micro_ros_config.h`
