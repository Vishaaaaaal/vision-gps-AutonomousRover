# 🤖🚗 Vision-Assisted Autonomous Driving System

A scalable, hybrid autonomous navigation system for small-scale outdoor rovers using real-time GPS and AI-powered visual perception.

---

## 📌 Overview

This project showcases a **vision-first autonomous driving prototype** that combines:
- Global navigation via **u-blox NEO-M9N GPS** + **Google Maps Directions API**
- Local, reactive perception through **ZED 2i stereo vision**, **YOLOv5** obstacle detection, and **LaneNet**-based lane following
- A hybrid ROS 1 + ROS 2 architecture for modular control
- An Arduino-based actuator system (linear + stepper)
- Real-time GUI for visual feedback and interactive routing

Developed as a final-year B.Tech project under the **Department of Mechatronics Engineering**, SRM Institute of Science & Technology.

---

## 🎯 Key Features

- **Dynamic GPS path planning** with Google Maps API
- **YOLOv5 obstacle detection** (depth-filtered for precision)
- **LaneNet-based curved lane detection**
- **ROS 1/2 hybrid architecture** bridged via TCP sockets
- **Live GUI**: Map tracking, ETA, and manual rerouting
- **Actuator control**: Stepper-based throttle, linear actuator steering
- **Potentiometer-based closed-loop steering feedback**
- **Failsafe depth-only emergency stop logic**

---

## 🧠 System Architecture

![System Block Diagram](docs/system_architecture.png) <!-- Add diagram here -->

- **Global Planner**: Generates waypoints from current GPS location to destination using Google Maps.
- **Local Planner**: Analyzes ZED stereo camera feeds for obstacle avoidance and lane following.
- **Merger Module**: Prioritizes real-time visual threats and turn anticipation over GPS progression.
- **Actuation Control**: ROS Serial + Arduino Nano for translating motion commands to hardware.

---

## 🔩 Hardware Stack

| Component              | Functionality                                |
|-----------------------|-----------------------------------------------|
| ZED 2i Stereo Camera  | RGB, Depth, IMU, VIO                          |
| NEO-M9N GPS Module    | Live location for Google Maps API             |
| Linear Actuator       | Steering via 4-bar Ackermann linkage          |
| NEMA-23 Stepper Motor | Throttle actuation via twist grip             |
| Arduino Nano          | Low-level motor control and feedback loop     |
| Laptop (Ryzen 7 + RTX)| Runs ROS stack, detection, and GUI            |

---

## ⚙️ Software Stack

- **ROS Noetic (ROS 1)**: For planning, GPS, actuation, and GUI
- **ROS 2 Foxy**: For ZED vision and YOLOv5 inference
- **Socket Bridge**: ROS 1 ↔ ROS 2 communication (Python TCP)
- **YOLOv5**: Custom-trained model for road obstacles
- **LaneNet**: Curved lane detection using polynomial fits
- **RViz + OpenCV**: Live visualization and perception overlays
- **PyQt5 + Folium**: Interactive map GUI

---

## 🖥️ GUI Preview

![Map GUI](docs/gui_live.png)
- Route selection with map click
- Displays ETA, distance, and obstacle alerts
- HTML map using Leaflet.js embedded into PyQt5 GUI

---

## 🚗 Control Logic

| Input Type     | Action                                |
|----------------|----------------------------------------|
| GPS Position   | Starts global planner waypoint parsing |
| YOLO + Depth   | Triggers stop/slowdown                |
| Lane Curvature | Triggers pre-emptive turns            |
| Potentiometer  | Closes feedback loop for steering      |
| GUI/Socket     | Sends command strings to Arduino       |

---

## 📈 Results Snapshot

| Metric                    | Value                         |
|---------------------------|-------------------------------|
| Localization Accuracy     | Avg. 2.87m deviation          |
| YOLO Detection F1 Score   | 0.593                         |
| Lane Following            | Robust on straight paths      |
| GPS Navigation Test       | Route execution within margin |
| Depth-Only Failsafe       | Activated < 1.0m range        |

---

## 📂 Folder Structure

```bash
vision-gps-AutonomousRover/
├── arduino_code/            # Arduino sketch files
├── gui/                     # PyQt5 + Folium GUI scripts
├── perception/              # YOLOv5 + LaneNet vision scripts
├── planning/                # GPS decoding and Google API
├── ros1_ws/                 # ROS 1 workspace
├── ros2_ws/                 # ROS 2 workspace
├── docs/                    # Diagrams, system images, results
└── README.md
