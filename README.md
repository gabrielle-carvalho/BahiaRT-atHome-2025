# BahiaRT_atHome - Installation of Navigation Dependencies
This folder contains the guide to install the navigation firmware. In it, we will install the resources needed to integrate micro-ROS as a component in an Espressif ESP-IDF Build System.

## Step 1. ROS2 Humble and ESP-IDF:
First, install ROS2(Humble version) following the official documentation guide below:

https://docs.ros.org/en/humble/Installation.html

After install ROS2, install the Development tools:  

```bash
sudo apt install ros-dev-tools
```

After that, add the CLI Tools to the .bashrc:

```bash
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

Run this command and check the enviroment variables:

```bash
printenv | grep -i ROS
```

The output should be as follows:

```bash
ROS_VERSION=2
ROS_PYTHON_VERSION=3
ROS_DISTRO=humble
```

After that, install ESP-IDF by following the steps in the link below:

https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/linux-macos-setup.html#step-1-install-prerequisites

Repeating a step in the guide, confirm that the command has been added to final line of **.bashrc**:

```bash
nano ~/.bashrc

alias get_idf='. $HOME/esp/esp-idf/export.sh' (Don't add if exists)
```

If it does not appear, add it manually or with the command below:

```bash
echo alias get_idf='. $HOME/esp/esp-idf/export.sh' >> ~/.bashrc 
```

With the ESP-IDF dependencies installed, confirm the update of the environment variables:
```bash
source ~/.bashrc
```

Connect the ESP32 to the computer and check the port that the ESP32 is connected to (ttyUSB0 or ​​ttyACM0):
```bash
sudo dmesg | grep tty

ls -l /dev/tty/USB0
```

Change the port permission:
```bash
sudo chmod 777 /dev/ttyUSB0
```

## Step 2. Micro-ROS Installation:
Once everything is set up with ROS, we can start installing micro-ros. Follow these steps to install the micro-ROS build system

Create a workspace and download the micro-ROS tools:
```bash
mkdir microros_ws

cd microros_ws

git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
```

Update dependencies using rosdep:
```bash
sudo apt update && rosdep update
```

Build micro-ROS tools and source them:
```bash
colcon build

source install/local_setup.bash
```

## Step 3. Create the Workspace of Firmware for ESP32:

This step creates a firmware workspace that directs all the necessary code and tools. After the command is executed, a folder named firmware should be present on your desktop.

```bash
ros2 run micro_ros_setup create_firmware_ws.sh freertos esp32
```

## Step 4. Configuring the Created Firmware to use Serial (ttyUSB* or ttyACM*):

Install the zipped fox_t4 firmware code from the remote repository, and extract it into the apps folder::
```bash
wget -O fox_t4.zip https://github.com/gabrielle-carvalho/BahiaRT-atHome-2025/blob/nav_firmware/fox_t4/fox_t4.zip && unzip fox_t4.zip -d ~/microros_ws/firmware/freertos_apps/apps/fox_t4/

rm fox_t4.zip

cd ~/microros_ws/

colcon build

source install/local_setup.bash

ros2 run micro_ros_setup configure_firmware.sh fox_t4 --transport serial
```

## Step 5. Build Firmware:

```bash
cd ~/microros_ws/

source install/local_setup.bash

ros2 run micro_ros_setup build_firmware.sh
```

At the end of the compilation, you should get error-free output.

## Step 6. Flash Firmware:

```bash
ros2 run micro_ros_setup flash_firmware.sh
```

## Step 7. Create micro-ROS agent:

```bash
ros2 run micro_ros_setup create_agent_ws.sh

ros2 run micro_ros_setup build_agent.sh
```

## Step 8. Run and test micro-ROS aplication(Serial port):

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```
After run this command, press the EN(Reset) bottom on the physical robot

If the terminal output the creation of cmd_vel, odometry and other topics, congratulations! The firmware has installed if sucefully! After this, **return for this link:**

https://github.com/gabrielle-carvalho/BahiaRT-atHome-2025/blob/main/README.md