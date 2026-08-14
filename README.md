# 3-DOF Robotic Arm Simulation

I wanted to learn how robotic arms work from the ground up, so I decided to build a robotic arm and actually learn the math behind each part of the system first. 

This project is a Python-based 3-DOF robotic arm simulation implementing forward kinematics, inverse kinematics without libraries (using math I can visualize and write out in code), along with joint constraints, 3D visualization, and Xbox controller input.

The long-term goal is to use what I learn in simulation to build and control a physical robotic arm with 4 DOF or more.
## Overview

The simulated robot consists of:

- J1: Base rotation
- J2: Shoulder rotation
- J3: Elbow rotation

The system can calculate joint positions using forward kinematics and determine the joint angles required to reach a target XYZ position using inverse kinematics.

Two possible elbow configurations are supported, with joint limits and reachability checks.

## Features

- 3-DOF robotic arm simulation
- Forward kinematics using homogeneous transformation matrices
- Inverse kinematics for XYZ target positions
- Two elbow configurations
- Joint limit checking
- Workspace/reachability checking
- Smooth movement between joint configurations
- 3D visualization using Matplotlib
- Xbox controller input using Pygame

## Robot Configuration

| Component | Length |
|---|---:|
| L1 | 5 |
| L2 | 8 |
| L3 | 6 |

| Joint | Range |
|---|---|
| J1 Base | -180° to 180° |
| J2 Shoulder | -90° to 90° |
| J3 Elbow | -135° to 135° |

## Project Structure

```text
robotics/
│
├── robotSim/
│   ├── robot_3d_FWK.py
│   ├── robot_3d_IK.py
│   └── robot_3d_XBOX.py
│
├── .venv/
└── README.md
````

### Files

**`robot_3d_FWK.py`**

Forward kinematics simulation with manual joint controls and 3D visualization.

**`robot_3d_IK.py`**

Inverse kinematics simulation with XYZ target controls, elbow configuration selection, joint limit checking, and smooth movement.

**`robot_3d_XBOX.py`**

Xbox controller interface for controlling the robotic arm target position.

## Technologies

* Python
* NumPy
* Matplotlib
* Pygame
* Linear Algebra
* Forward & Inverse Kinematics
* Homogeneous Transformation Matrices

## Installation

Create and activate a virtual environment:

```bash
python -m venv .venv
```

```powershell
.\.venv\Scripts\Activate.ps1
```

Install dependencies:

```bash
pip install numpy matplotlib pygame
```

## Running

### Forward Kinematics

```bash
python robotSim/robot_3d_FWK.py
```

### Inverse Kinematics

```bash
python robotSim/robot_3d_IK.py
```

### Xbox Controller

Connect an Xbox controller and run:

```bash
python robotSim/robot_3d_XBOX.py
```

## Development Status

### Completed

- [x] 3-DOF robot model
- [x] Forward kinematics
- [x] Inverse kinematics
- [x] Two elbow configurations
- [x] Joint limit checking
- [x] Reachability checking
- [x] 3D visualization
- [x] Smooth IK movement
- [x] Xbox controller integration
- [x] Real-time XYZ target control
- [x] Continuous adaptive IK
- [x] Controller deadzone and sensitivity
- [x] Unreachable-target handling
- [x] Controller-based elbow configuration switching

### Next Steps

- [ ] Improve motion control and responsiveness
- [ ] Design and 3D print physical robot
- [ ] Select motors and motor drivers
- [ ] Integrate ESP32
- [ ] Send joint commands from Python to ESP32
- [ ] Control physical robot
- [ ] Add fourth degree of freedom
- [ ] Add end-effector/gripper

## Planned System

```text
Xbox Controller
       ↓
Target XYZ
       ↓
Inverse Kinematics
       ↓
Joint Angles
       ↓
ESP32
       ↓
Motor Controllers
       ↓
Physical Robot
```

The long-term goal is to use the same kinematics system for both the simulation and physical robot.

## Author

**Krish Patel**

Computer Engineering Student — University of Guelph

[GitHub](https://github.com/krishp2006)

[LinkedIn](https://linkedin.com/in/krishp22)
