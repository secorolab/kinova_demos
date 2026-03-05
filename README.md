# grc26 kinova demo

## Task

### realworld

- A single kinova gen3 arm mounted on a table top.
- A tray setup on the table.
- The arm moves compliantly down till it made contact with the table.
- After contact, it slides forward to make contact with the tray (fixed position)
- It then waits for a human to hold the tray on the other side.
- Once human holds, the arm initiates movement upwards to place tray in a diff. place.
- The arm reacts compliantly to the human while in motion.
- Task completes, once the tray is placed in the target position on the table.

### simulation - isaacscim 5.1.0

- another kinova robot acts as an agent instead of human
- no compliance behaviour
- uses moveit for motion planning

## Setup

### clone the repo

```shell
mkdir -p ~/grc26/src

cd ~/grc26/src

git clone --depth=1 https://github.com/secorolab/grc26_kinova_demo.git
```

### pull the dependent repos

```shell
vcs import src < src/grc26_kinova_demo/grc26.repos
```

### setup flags

copy the [colcon.meta](colcon.meta) to the ros2 workspace folder.

```shell
cd ~/grc26

cp src/grc26_kinova_demo/colcon.meta .
```

### build

```shell
cd ~/grc26

colcon build
```

## Isaac Sim Setup for ROS2 bridge

You need to enable Isaac Sim's internal ROS2 libraries in the terminal before launching Isaac Sim:

```sh
export isaac_sim_package_path=$ISAAC_SIM_DIR # where your Isaac Sim install is located

export ROS_DISTRO=jazzy

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Can only be set once per terminal.
# Setting this command multiple times will append the internal library path again potentially leading to conflicts
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$isaac_sim_package_path/exts/isaacsim.ros2.bridge/jazzy/lib
```

> [!IMPORTANT]
> Do not source the system ROS2 before running Isaac Sim as this will most likely cause a Python version mismatch. Isaac Sim uses its built-in Python 3.11!

`launch_sim.py` then needs to be ran in the same terminal using the bundled `python.sh`.

To control the simulation itself, see [Isaac Sim ROS2 Simulation Control](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_simulation_control.html#using-the-ros-2-simulation-control-services).
