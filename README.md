## Alive Arm Project

This Unity project simulates a robotic arm that starts from a fallen position and learns to stand up using reinforcement learning powered by ML-Agents. The goal is to create lifelike, adaptive behavior where the arm can recover from the ground and eventually move autonomously.

Initially, the goal is for the arm — with a fixed base — to reach a target height from a default home position.  
Later, it will perform the same task starting from random poses, and eventually, the base will become movable.

### Visual Snippet
![Alive Arm Training](images/image_1.png) 

Using prefabs, six learning agents are trained simultaneously. This setup works well as long as the time scale is moderate — avoid setting it too high (e.g., >15) to prevent instability.

### Key Features (Planned)
- Robotic arm with realistic physics and a movable base
- Reinforcement learning via ML-Agents (PPO)
- Step-by-step learning pipeline: **Stand → Balance → Move**
- Modular Unity scene and C# architecture for easy expansion

### Tech Stack
- Unity Engine (60001.3f1)
- ML-Agents Toolkit (3.0.0)
- C# (for simulation logic)
- Python (training backend)

### How To
- Activate the conda environment
- Run: `mlagents-learn configs/arm_config.yaml --run-id=armx`
- Press **Play** in the Unity Editor to begin simulation

### Project Status
Early development – the arm simulation is complete, and the reinforcement learning pipeline is being set up and tuned.

### Credits

This project uses 3D models from [Preliy/Flange](https://github.com/Preliy/Flange), licensed under the MIT License.
