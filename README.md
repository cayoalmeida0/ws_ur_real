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

## Control Strategies Implemented

### 1. Joint Velocity Control
- Direct velocity commands via ros2_control
- Topic: `/forward_velocity_controller/commands`

### 2. Cartesian Position Control
- TCP position control using Jacobian-based mapping
- Error: \( e_p = p_d - p \)

### 3. Cartesian Pose Control (Position + Orientation)
- Full pose control using geometric Jacobian
- Orientation error based on rotation matrices:
  
  \[
  e_o = \frac{1}{2}(R_c^x \times R_d^x + R_c^y \times R_d^y + R_c^z \times R_d^z)
  \]

- Control law:

  \[
  \dot{q} = J^T (J J^T + \lambda^2 I)^{-1} v
  \]

- Real-time feedback using:
  - `/joint_states`
  - `/tcp_pose_broadcaster/pose`
