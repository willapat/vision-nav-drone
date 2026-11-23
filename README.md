# vision-nav-drone

A simulated quadcopter that will search a forest for a missing hiker using only its camera, then track and follow them while dodging trees. GPS is unreliable under thick tree cover, so the end goal is navigation from vision and inertial sensors alone.

**Status: work in progress.** The simulation stack and the first flight-control and path-planning nodes work. Camera-based perception is next.

<!-- Screenshot: add docs/forest_run.png and uncomment
![x500 flying through the forest world in Gazebo](docs/forest_run.png)
-->

## Stack

- Ubuntu 24.04, ROS 2 Jazzy
- PX4 autopilot (software-in-the-loop), x500 quadcopter
- Gazebo Harmonic
- Micro XRCE-DDS bridge between PX4 and ROS 2 (`px4_msgs`)
- Python today; C++ (rclcpp) for the performance-critical nodes to come

## What works so far

| Node | What it does |
|---|---|
| [position_listener.py](position_listener.py) | Subscribes to PX4's EKF2 odometry and prints position, converted from PX4's NED frame (z down) to altitude-positive. |
| [offboard_square.py](offboard_square.py) | Arms the drone, switches to offboard mode and flies a 5 m square at 5 m altitude by streaming position setpoints, then lands. |
| [agility_test.py](agility_test.py) | Measures the airframe's limits: takes off, sprints with velocity setpoints, brakes, flies a slalom, returns home and reports top speed and braking time. |
| [forest_run.py](forest_run.py) | Flies home → through a forest → a goal past the tree line → home, as fast as is safe. Details below. |

### forest_run.py

Tree positions currently come from the world file; the camera will replace this later.

1. **Obstacles:** each tree becomes a circle whose radius is the widest part of its collision mesh within the height band the drone occupies, plus the drone's own radius.
2. **Planning:** A* on a clearance grid, with a cost that prefers open space.
3. **Smoothing:** line-of-sight shortcutting that never reduces clearance, then corner rounding.
4. **Speed profile:** a top speed limited by path curvature (lateral acceleration) and clearance, then acceleration/braking limits.
5. **Following:** position setpoints on the nearest path point, with velocity and acceleration feed-forward from a lookahead point.

It logs per-leg times, average speed, closest approach to any tree and maximum cross-track error. Set `FOREST_CSV=run.csv` to log setpoints against actual state at 20 Hz.

## Running it

Prerequisites: ROS 2 Jazzy, Gazebo Harmonic, [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) cloned to `~/PX4-Autopilot`, and the Micro XRCE-DDS Agent.

```bash
# 1. Build px4_msgs in this workspace (clone it into src/ first)
git clone https://github.com/PX4/px4_msgs.git src/px4_msgs
colcon build
source install/setup.bash

# 2. Start the bridge
MicroXRCEAgent udp4 -p 8888

# 3. Start the simulator (another terminal); use PX4_GZ_WORLD=forest for forest_run.py
cd ~/PX4-Autopilot && PX4_GZ_WORLD=forest make px4_sitl gz_x500

# 4. Run a node (another terminal, after sourcing install/setup.bash)
python3 forest_run.py
```

## Roadmap

1. ~~Setup: PX4 + Gazebo + ROS 2 simulation, scripted takeoff and hover~~
2. Basic control: waypoint flight and camera streaming; port the offboard node to C++ and add a safety state machine (takeoff, hover, return home, land) with GoogleTest unit tests *(in progress)*
3. Perception: turn off GPS and estimate pose with visual-inertial odometry / visual SLAM
4. Mapping and avoidance: 3D occupancy map from a depth camera (OctoMap) and a local planner between trunks
5. Search: autonomous coverage / frontier exploration; a Gazebo plugin that randomizes forests and hiker positions
6. Detect and pin: YOLO person detection, projected to a 3D location on the map
7. Track and follow: Kalman-filter tracking that keeps the hiker in view and re-acquires them behind trees
8. Measure: hundreds of headless trials, scored automatically against ground truth

### Success metrics

- **Search:** time to find the hiker, area covered
- **Detection:** accuracy, and error between the pinned and true position
- **Tracking:** share of time the hiker is in view, losses and re-acquisitions
- **Safety:** collisions per flight
