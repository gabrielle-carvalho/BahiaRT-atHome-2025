# BahiaRT_atHome

This repository contains the documentation of scientific contributions and work from the BahiaRT team.

## Scientific Publications
The team's publications can be found [here](https://www.acso.uneb.br/acso/uploads/Main/acsobibref.html).

---

# Object Detection Workspace

## Overview
This workspace is designed for **object detection and recognition** using the **YOLO (You Only Look Once)** model and **ROS2 (Robot Operating System 2)**. It includes two main nodes:
- `object_recognition_node`: Captures images from a camera, processes them to detect objects, and publishes the detected objects.
- `voice_node`: Subscribes to the detected objects and uses **text-to-speech** to announce the detected objects.

## Directory Structure
```plaintext
Object_Detection/
└── ObjectDetection_ws/
    └── src/
        └── object_recognition/
            ├── __pycache__/
            ├── yolo/
            │   ├── __init__.py
            │   └── yolov8n.pt
            ├── object_recognition_node.py
            └── voice_node.py
```
## Installing Dependencies

### Using pip
1. Open a terminal.
2. Navigate to the directory of your project where the `requirementsObject.txt` file is located.
3. Run the following command to install the dependencies:

    ```bash
    pip install -r requirements.txt
    ```

### ROS2 Dependencies
For ROS2-specific dependencies, such as `rclpy` and `cv_bridge`, you may need to install additional packages. Use the following command:

```bash
sudo apt-get update
sudo apt-get install ros-<your-ros2-distro>-cv-bridge
```

## How to Run

Follow these steps to run the object detection and voice nodes.

1. **Build the workspace**:

    To build the workspace, run the following command:

    ```bash
    colcon build
    ```

2. **Source the workspace**:

    After building, source the workspace to set up the environment:

    ```bash
    source install/setup.bash
    ```

3. **Run the object recognition node**:

    Launch the object recognition node using this command:

    ```bash
    ros2 run object_recognition object_recognition_node
    ```

4. **Run the voice node**:

    Start the voice node to announce the detected objects:

    ```bash
    ros2 run object_recognition voice_node
    ```

By following these steps, the system will be able to detect objects using YOLO and announce them using text-to-speech.

