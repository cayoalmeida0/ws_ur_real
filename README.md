# UR3e + ROS 2 Jazzy + MoveIt Integration

This repository contains the implementation and experimental validation of a real robotic system based on a Universal Robots UR3e manipulator integrated with ROS 2 Jazzy and MoveIt 2.

## System Overview

- Robot: UR3e (Universal Robots)
- Middleware: ROS 2 Jazzy
- Simulator: Gazebo Harmonic (for simulation stage)
- Motion Planning: MoveIt 2
- Control: ros2_control + custom joint velocity controller

## Features

- Real robot communication via RTDE
- External Control URCap integration
- Kinematic calibration using ur_calibration
- Trajectory execution via MoveIt
- Custom velocity-based control (in development)

## Setup

See `docs/setup.md` for full installation and configuration steps.

## Experiments

Validated:

- Driver connection with real robot
- MoveIt planning and execution
- Controller switching (trajectory vs velocity)

## Future Work

- Integration of custom Jacobian-based controller
- Constraint-based control (CBF)
- Comparative analysis with MoveIt trajectories
