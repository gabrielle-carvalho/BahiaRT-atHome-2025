# BahiaRT_atHome
This folder contains the BahiaRT team contribution in the personal recognition challenge.

# How to run?

1. Clone this repository;  

2. After cloning the repository, you can install the necessary Python dependencies using the requirementsPersonal.txt file:  

```bash
pip install -r requirementsPersonal.txt
```

3. Install [**Navigation Requeriments**](https://gitlab.com/bahiart/athome/BahiaRT-atHome-2025/-/blob/nav_firmware/README.md)   (Esp-IDF and firmware dependencies):

After successfully completing the last step of the firmware installation, keep the terminal with the agent in the background and proceed on a new terminal.

4. In the workspace:
```bash
colcon build
source install/setup.bash
cd src/personal_recognition/personal_recognition/
```

5. After that, run the nodes individually, in this order:

```bash
./image_capture_node.py
```
```bash
./personal_recognition_node.py
```
```bash
./turn_around_node.py
```
```bash
./voice_node.py
```
So, follow the program instructions until it's over. If you have any questions, contact us.


# Example of Detection Results
<img src="BahiaRT_2025/Personal_Recognition/PersonalRecognition_ws/src/personal_recognition/personal_recognition/predict/Unknown.jpg" alt="Reconhecendo 2 pessoas, uma como Pedro e outra como desconhecido" width="700">