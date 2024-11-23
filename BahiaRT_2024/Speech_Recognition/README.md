# BahiaRT_atHome

## Speech Recognition - Context

This project implements a *Question Answering* (QA) system using ROS2. It consists of two main modules:  
1. Answering questions based on a **predefined context** using a **DistilBERT**  model.  
2. Training the DistilBERT model using the `context.txt` file.  

---

## Prerequisites

Ensure your system has the following dependencies installed:  

## General Tools
- Python 3.8+  
- ROS2 (Humble)  
- `pip` para gerenciar pacotes Python  

## Python Libraries
Install the required libraries by running:  
```bash
pip install -r requirements.txt

```
## Configuration

1. Clone this repository:
   ```bash
   git clone https://github.com/gabrielle-carvalho/BahiaRT_atHome.git
   cd BahiaRT_atHome

2. Build the ROS2 package:
   ```bash
   colcon build
3. Set up the Workspace environment:
   ```bash
   source install/setup.bash
4. Configure the context:

   If needed, insert the context text into the file `speech_pkg/context.txt`. This will be used to answer questions.

---

## How to Run

1. Launch the main node:
   ```bash
   ros2 launch speech_pkg speech_launch.py
   ```
2. Interact with the system after the node starts:

- Speak a question in English using the microphone.
- The system will process the speech, identify the question, and provide an answer based on the loaded context.

3. Train the Model
If you want to train the DistilBERT model with a new context, follow the steps below:

- Edit the `speech_pkg/context.txt` file with the new context.
- Run the training script:
  ```bash
  python3 bert_treinamento.py
