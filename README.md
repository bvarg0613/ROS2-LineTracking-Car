# ROS2-LineTracking-Car
A GitHub with all the resources necessary to create your own small car that utilizes OpenCV and ROS2 to follow a line of a certain color!

# 🤖 ROS2 Line-Tracking Robot Car with RoboClaw + Arduino

This project is a **ROS2-powered robot car** that uses a USB camera and computer vision to detect a line (blue tape), then sends movement commands via serial to an **Arduino Uno** which controls the motors through a **RoboClaw motor controller**.

Built with 💻 ROS 2 (Jazzy), ⚙️ PlatformIO + Arduino, and 🔍 OpenCV.

---

## 🚘 Project Features

- 🔵 Detects blue tape on the ground to follow  
- 🧠 Decision-making with a custom ROS2 Python node  
- 🛰 Sends commands (`F`, `L`, `R`, `S`) over serial  
- ⚡ Arduino interprets commands and controls RoboClaw for motor output  
- 🛠️ Modular design using ROS2 publisher/subscriber architecture  
- 🧩 CAD files available for building your own chassis  

---

## 🗂 Project Structure

```bash
ROS2-LineTracking-Car/
├── ros2_ws/              # ROS2 workspace and nodes
│   └── src/
│       └── line_tracker/
│           ├── line_tracker_node.py     # Publishes "MOVE LEFT", "MOVE RIGHT", etc.
│           └── line_to_serial_node.py   # Subscribes and sends commands via serial
├── platformio_arduino/   # PlatformIO Arduino code to control RoboClaw
│   └── src/
│       └── main.cpp
├── cad/
│   └── robot_chassis.STL # 3D-printable chassis part(s)
└── README.md             # You are here
'''

