# Alive Arm Project

## Overview

The Alive Arm Project is a mechatronic simulation platform that combines physics-based modeling, control, and reinforcement learning to enable a robotic arm to recover from a fallen pose and progressively acquire autonomous movement skills. Implemented in Unity with ML-Agents, it serves as a testbed for studying perception-action coupling, adaptive control, and curriculum learning in embodied agents.

## Features

### Implemented
- Fully modeled robotic arm with realistic kinematics and physics in Unity  
- Simulation environment initialized from a fallen state  
- Modular C# architecture separating environment, agent logic, and observation/action interfaces  

### Planned / In Progress
- Reinforcement learning pipeline to teach the arm to stand up using ML-Agents  
- Curriculum progression: **Stand → Balance → Move**  
- Python-based training backend for policy optimization, reward shaping, and data logging  
- Extensible Unity scenes and scenario definitions for future behaviors  

## Technical Stack

- **Unity Engine (2022+)** – simulation, physics, and agent hosting  
- **ML-Agents Toolkit** – reinforcement learning framework (specify version)  
- **C#** – environment and agent logic inside Unity  
- **Python** – training orchestration, policy management, and telemetry  

## Current Focus / Milestones

- [x] Design and animate the robotic arm  
- [*] Define and implement the first training scenario (stand-up task)  
- [ ] Train and validate standing behavior  
- [ ] Extend learned policy toward balance and locomotion  

## Project Status

Early development: the simulation environment and robotic arm model are complete. The current effort is on defining the learning scenario and initiating the first reinforcement learning training to achieve reliable stand-up behavior.

## Future Directions

- Extend to balance and movement policies  
- Introduce domain randomization for robustness  
- Simulate sensor noise and environmental perturbations  
- Instrument failure modes and visualize learned strategies  
