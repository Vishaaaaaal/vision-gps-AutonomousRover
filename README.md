# vision-gps-AutonomousRover
Hybrid GPS + vision-based autonomous rover developed at SRMIST. Uses NEO-M9N GPS and Google Maps API for global navigation, with ZED 2i, YOLOv8, and LaneNet for local obstacle and lane detection. Built with ROS1/ROS2 and Arduino for real-time actuation and control.

# 🚗 vision-gps-AutonomousRover

A hybrid GPS + vision-based autonomous rover developed at SRM Institute of Science and Technology (SRMIST). This project demonstrates autonomous navigation for a scaled model vehicle using cutting-edge AI and robotics technologies.

---

## 📌 Project Overview

This system uses:
- **NEO-M9N GPS module** and **Google Maps API** for global navigation
- **ZED 2i stereo camera** for visual perception (RGB + depth)
- **YOLOv8** for object detection
- **LaneNet** for lane detection
- **RTAB-Map** for SLAM (Simultaneous Localization and Mapping)
- **ROS 1 Noetic + ROS 2 Foxy** for perception, planning, and control
- **Arduino Nano** for actuator control (stepper motor + linear actuator)

---

## 🧠 System Architecture

![System Overview](docs/system_architecture.png) <!-- Add your system image here -->

- Global Planner: GPS + Google Maps route
- Local Planner: Lane and obstacle-aware using vision
- Control: PID-based actuation with sensor feedback

---

## 🔩 Hardware Used

| Component             | Description                              |
|----------------------|------------------------------------------|
| NEO-M9N GPS          | High-precision GNSS module               |
| ZED 2i Camera        | Stereo vision + IMU                     |
| Linear Actuator      | Steering control                        |
| NEMA-23 Stepper      | Throttle actuator (coupled to twist grip)|
| Arduino Nano         | Real-time actuator control               |
| 24V DC Motor         | Propulsion via chain drive               |

---

## 🧰 Software Stack

- **Languages**: Python, C++
- **Frameworks**: ROS1, ROS2, Arduino
- **Libraries**: RTAB-Map, OpenCV, AccelStepper, ROS Serial
- **Planning**: A* Global Planner, DWA Local Planner
- **Detection**: YOLOv8 (Ultralytics), LaneNet
