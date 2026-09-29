# Pick and Place with MuJoCo Sim

[demo.webm](https://github.com/user-attachments/assets/eb642fad-b6b0-4bbd-9abd-b33bc34a7aef)


## Contents

- [Introduction](#introduction)
- [Dependencies](#dependencies)
- [Setup](#setup)
- [Usage](#run-instructions)


## Introduction

This project contains a basic pick and place implementation with MuJoCo Sim and Franka Panda Arm using MoveIt

The code files are pretty self-explanatory. Here's a little info:
- `world.py` contains the model information where we import the Franka Panda Arm and setup a simulated environment with a table, a red cube and a green target zone.
- `mujoco_bridge.py` serves as the ROS bridge that connects the simulation to ROS and publishes the robot states and gripper info, which is used by moveit.
- `panda_mujoco.launch.py` launch file that starts the MoveIt side (move_group, the robot description, RViz, and the controller config), which does the actual motion planning and collision checking against the Panda model.
- <b>`pick_place.py`</b> is the high-level task script: it acts as a client of MoveIt and the gripper, sequencing the steps (approach, grasp, lift, transport, place, retreat), performing the Pick and Place motion.


## Dependencies

- mujoco, numpy, matplotlib, moveit
- You can install all the requirements using `requirements.txt`


## Setup

Clone the repository:

```bash
git clone https://github.com/Madhav2133/pick-and-place-mujoco-sim.git
```

The setup is pretty simple, you can install the requirements using:

```py
# Create a virtual environment (optional)
python3 -m venv env

# Activate the virtual environment
source env/bin/activate

# Install the requirements
pip3 install -r requirements.txt
```

``` bash
# Install panda moveit config
sudo apt install ros-$ROS_DISTRO-moveit ros-$ROS_DISTRO-moveit-resources-panda-moveit-config
```

## Run Instructions

For running the simulation, you have to launch the `mujoco_bridge` that starts the simulation: 

```bash
python3 mujoco_bridge.py
```

The `launch file` that starts the MoveIt side

```bash
ros2 launch ./panda_mujoco.launch.py 
```

Finally the main `pick_and_place` script that runs the task. (All in seperate terminals)

```bash
python3 pick_place.py
```

## Resources

- MuJoCo Sim
- [MuJoCo MENAGERIE](https://github.com/google-deepmind/mujoco_menagerie.git)
- ROS2, MoveIt2
