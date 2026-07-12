# Autonomous Room Cleaning Robot Simulation

![ROS 2](https://img.shields.io/badge/ROS-2-22314E?logo=ros&logoColor=white)
![Gazebo](https://img.shields.io/badge/Gazebo-sim-orange)
![Python](https://img.shields.io/badge/Python-3.12-3776AB?logo=python&logoColor=white)

A ROS 2 + Gazebo simulation of a mobile robot that autonomously covers a room in a zig-zag (boustrophedon) pattern, using a simulated LiDAR to detect and bypass obstacles — similar in behavior to a robotic vacuum cleaner.

## Table of Contents

- [Features](#features)
- [Tech Stack](#tech-stack)
- [Architecture Overview](#architecture-overview)
- [Project Structure](#project-structure)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Running the Simulation](#running-the-simulation)
- [Build](#build)
- [Testing](#testing)
- [Usage / Behavior Notes](#usage--behavior-notes)
- [Security Considerations](#security-considerations)
- [Troubleshooting](#troubleshooting)
- [Known Issues / Inconsistencies](#known-issues--inconsistencies)
- [Contributing](#contributing)
- [License](#license)
- [Future Improvements](#future-improvements)

## Features

From `cleaning_robot/cleaning_node.py`:

- **Zig-zag room coverage** — the robot drives to a starting corner, then sweeps the room in rows (`FORWARD` → `TURN1` → `SHIFT` → `TURN2`, alternating direction each row) up to `max_rows` (14).
- **LiDAR-based obstacle detection** — reads the front arc of a `/scan` `LaserScan` message and flags an obstacle when the minimum range drops below a threshold (0.55 m).
- **Obstacle bypass state machine** — on detecting an obstacle mid-row, the robot executes a 4-step bypass (`BYPASS_TURN1` → `BYPASS_SHIFT` → `BYPASS_TURN2` → `BYPASS_FORWARD`) to go around it and rejoin the same row heading.
- **Coverage visualization** — publishes `visualization_msgs/MarkerArray` on `/coverage_trail`, dropping a cylinder marker at each newly-visited grid cell so cleaned area can be viewed in RViz/Gazebo.
- **Odometry-based pose tracking** — converts `/odom` quaternion orientation to yaw for heading control.

## Tech Stack

- **ROS 2** (`rclpy`) — robot control node, targeting Python 3.12 (per compiled `__pycache__` artifacts)
- **Gazebo (`gz sim`, modern/Ignition-based)** — simulation environment, launched via `ExecuteProcess(cmd=['gz', 'sim', ...])`
- **`ros_gz_bridge`** — bridges `/cmd_vel`, `/scan`, `/odom`, `/clock` between ROS 2 and Gazebo
- **URDF/SDF** — `robot.urdf` (differential-drive robot with a LiDAR sensor) and `room.sdf` (world definition)
- **ament_python** build type, packaged with `setuptools` (`setup.py`/`setup.cfg`)
- **ament_copyright / ament_flake8 / ament_pep257 / pytest** — declared test dependencies

## Architecture Overview

```
 Gazebo (gz sim, room.sdf world)
   │  spawns robot from robot.urdf
   ▼
 ros_gz_bridge  ── /cmd_vel ──▶ Gazebo DiffDrive plugin
                 ◀── /scan ─── Gazebo GPU LiDAR sensor
                 ◀── /odom ─── Gazebo DiffDrive odometry
                 ◀── /clock ──
   │
   ▼
 cleaning_node (rclpy)
   - scan_callback  -> obstacle_front flag
   - odom_callback  -> pose (x, y, yaw), triggers coverage-area tracking
   - move() @ 10 Hz -> state machine -> publishes /cmd_vel
                                      -> publishes /coverage_trail markers
```

The robot's motion is a finite-state machine: `GOTO_CORNER` (drive to start position) → `CLEAN` phase with row-sweep states (`ALIGN`, `FORWARD`, `TURN1`, `SHIFT`, `TURN2`) and a nested obstacle-bypass sub-state machine, ending in `DONE` once `max_rows` is reached.

## Project Structure

```
Autonomous-Room-Cleaning-Robot-Simulation-/
├── README.md
├── .gitignore                             # excludes colcon build/, install/, log/
└── src/
    └── cleaning_robot/                    # the ROS 2 package (ament_python)
        ├── cleaning_robot/
        │   ├── __init__.py
        │   └── cleaning_node.py           # the robot control node (see Features)
        ├── launch/
        │   └── simulation.launch.py       # brings up Gazebo, bridge, spawns robot, starts node
        ├── urdf/
        │   ├── robot.urdf                 # differential-drive robot + LiDAR
        │   └── room.sdf                   # Gazebo world
        ├── test/                          # ament_copyright/flake8/pep257 test stubs
        ├── package.xml
        └── setup.py / setup.cfg
```

The repo root is the colcon workspace root: run `colcon build` directly from it.

## Prerequisites

- Ubuntu with **ROS 2** installed (a distro supporting Python 3.12, e.g. Jazzy or newer)
- **Gazebo (`gz sim`)** — the modern Ignition-based Gazebo, not classic Gazebo (the launch file invokes `gz sim`, not `gazebo`)
- `ros_gz_bridge` and `ros_gz_sim` packages
- `colcon` build tools (`python3-colcon-common-extensions`)

## Installation

```bash
# From the repo root

# Source your ROS 2 installation first, e.g.:
# source /opt/ros/<distro>/setup.bash

colcon build --packages-select cleaning_robot
source install/setup.bash
```

## Running the Simulation

```bash
ros2 launch cleaning_robot simulation.launch.py
```

This launch file (see `launch/simulation.launch.py`):
1. Starts `gz sim -r` with the `room.sdf` world
2. Starts `robot_state_publisher` with the robot description from `robot.urdf`
3. Starts `ros_gz_bridge` to bridge `/cmd_vel`, `/scan`, `/odom`, `/clock`
4. After a 5s delay, spawns the robot in the world via `ros_gz_sim create`
5. After a 7s delay, starts the `cleaning_node`

## Build

Build is handled entirely through `colcon` (see [Installation](#installation)). There is no separate build step beyond `colcon build`.

## Testing

The package declares standard ROS 2 ament linting tests in `test/`:
- `test_copyright.py` — ament copyright header check
- `test_flake8.py` — ament flake8 style check
- `test_pep257.py` — ament pep257 docstring check

Run them with:
```bash
colcon test --packages-select cleaning_robot
colcon test-result --verbose
```

There are no behavioral/unit tests for the cleaning algorithm itself (e.g., no tests of the state machine transitions or coverage tracking).

## Usage / Behavior Notes

- The robot always starts by driving to a fixed corner (`corner_x = -3.2`, `corner_y = -3.2`) before beginning coverage.
- Row length (`row_length = 6.2`), row spacing (`row_spacing = 0.45`), and row count (`max_rows = 14`) are hardcoded in `cleaning_node.py` and tuned to the specific `room.sdf` world — changing the room geometry will require re-tuning these constants.
- Coverage visualization can be viewed by subscribing to `/coverage_trail` in RViz2.

## Security Considerations

This is a local simulation project with no network-exposed services, authentication, or handling of external/untrusted input — standard security review categories (auth, secrets, injection) don't apply. The main risk surface is running arbitrary `gz sim`/ROS processes locally, which is expected for this kind of project.

## Troubleshooting

- **`gz: command not found`** — you have classic Gazebo installed, not the newer `gz sim` binary this project requires.
- **Bridge topics not connecting** — confirm `ros_gz_bridge` and `ros_gz_sim` are installed for your ROS 2 distro and match the message-type mappings in `launch/simulation.launch.py`.
- **Robot doesn't move** — check that `cleaning_node` actually started (7s delay after launch) and that `/cmd_vel` is being bridged to Gazebo's `DiffDrive` plugin (see `robot.urdf`).
- **`colcon build` fails to find `cleaning_robot`** — make sure you are running `colcon build` from the repo root (the directory containing `src/`).

## Known Issues / Inconsistencies

1. **No license declared** — `package.xml` literally contains `<license>TODO: License declaration</license>`.
2. **No CI/CD or Docker** — no `.github/workflows/`, no Dockerfile; this is expected for a ROS 2/Gazebo simulation project run locally, but worth noting since it's absent.

(Resolved in July 2026: the project previously lived inside a single `mar_prj.zip` archive together with colcon `build/`/`install/`/`log/` artifacts, an unrelated Flex `lexer.l`, and a broken `simulation.launch.py4` draft. The source now lives under `src/` as normal tracked files and the artifacts are gone.)

## Contributing

1. Fork the repository
2. Make changes under `src/cleaning_robot/`
3. Run the ament lint tests (`colcon test`)
4. Submit a pull request

## License

No license file exists in the repository; `package.xml` marks the license as `TODO`. Add a `LICENSE` file and update `package.xml` if you intend this to be open-source under a specific license.

## Future Improvements

- Add unit tests for the coverage/bypass state machine logic
- Make room dimensions and row parameters configurable instead of hardcoded, so the node isn't tied to one specific `room.sdf`
