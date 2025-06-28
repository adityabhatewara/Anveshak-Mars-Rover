# 🪐 Anveshak Mars Rover – Software Stack | IIT Madras

This repository contains the software framework for the **Anveshak Mars Rover** developed at **IIT Madras**. We're building a complete **navigation stack and core software modules from scratch** using **ROS 2**, designed for Autonomous and Manual operation for the University Rover Challenge help in USA.

## 🚀 Project Goals

- Build a modular, scalable software stack tailored for off-road planetary rovers
- Develop a custom navigation stack from scratch using ROS 2
- Ensure compatibility with sensors like GPS, IMU, LiDAR, and camera
- Integrate teleoperation and autonomous modes
- Simulate and test all systems in Gazebo/ignition environments

## 🧠 Key Modules (Planned)

- **Sensor Drivers & Integration** – IMU, GPS, LiDAR, and depth camera support
- **Localization & Mapping** – EKF-based sensor fusion, SLAM
- **Path Planning** – Global and local planners for rough terrain
- **Motion Control** – Differential drive + obstacle avoidance
- **Rover Interface Layer** – Interfaces for hardware and simulation
- **Simulation & Testing** – Full-stack testing in ROS 2 + Gazebo

## 🛠️ Tech Stack

- **Languages**: Python  
- **Middleware**: ROS 2 (Humble)  
- **Simulation**: Gazebo / Ignition  
- **Libraries**: OpenCV, Nav2, TF2, RViz2

## 🧑‍💻 Current Focus

We're in the early stages of setting up the architecture and building the base for the navigation stack from the ground up. The initial goal is to create a minimal working setup with sensor inputs and motion control in ROS 2.

## 📁 Repo Structure (WIP)

```bash
anveshak-mars-rover/
├── navstack/               # Custom navigation stack components
├── control/                # Rover control nodes
├── sensors/                # Sensor drivers and processing
├── launch/                 # Launch files for test/demo
├── sim/                    # Simulation configs and world files
├── docs/                   # Project notes and diagrams
└── README.md

