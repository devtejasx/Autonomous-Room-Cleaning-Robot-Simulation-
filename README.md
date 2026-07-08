# Autonomous Room Cleaning Robot Simulation

![ROS 2](https://img.shields.io/badge/ROS-2-22314E?logo=ros&logoColor=white)
![Gazebo](https://img.shields.io/badge/Gazebo-sim-orange)
![Python](https://img.shields.io/badge/Python-3.12-3776AB?logo=python&logoColor=white)

A ROS 2 + Gazebo simulation of a mobile robot that autonomously covers a room in a zig-zag (boustrophedon) pattern, using a simulated LiDAR to detect and bypass obstacles — similar in behavior to a robotic vacuum cleaner.

> **Note:** the entire project currently lives inside a single archive, `mar_prj.zip`, at the repo root rather than as tracked source files. See [Known Issues](#known-issues--inconsistencies) below — this README documents the project as it actually exists inside that archive.

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

The repository root currently contains only:

```
Autonomous-Room-Cleaning-Robot-Simulation-/
├── README.md
└── mar_prj.zip          # entire ROS 2 workspace, zipped (see Known Issues)
```

Inside `mar_prj.zip`, the actual ROS 2 colcon workspace is:

```
mar_prj/
├── src/
│   └── cleaning_robot/                    # the ROS 2 package (ament_python)
│       ├── cleaning_robot/
│       │   ├── __init__.py
│       │   └── cleaning_node.py           # the robot control node (see Features)
│       ├── launch/
│       │   └── simulation.launch.py       # brings up Gazebo, bridge, spawns robot, starts node
│       ├── urdf/
│       │   ├── robot.urdf                 # differential-drive robot + LiDAR
│       │   └── room.sdf                   # Gazebo world
│       ├── test/                          # ament_copyright/flake8/pep257 test stubs
│       ├── package.xml
│       ├── setup.py / setup.cfg
│       └── lexer.l                        # unrelated Flex lexer file — not part of this project
├── build/                                  # colcon build artifacts (should not be committed)
├── install/                                 # colcon install artifacts (should not be committed)
└── log/                                      # colcon build logs (should not be committed)
```

## Prerequisites

- Ubuntu with **ROS 2** installed (a distro supporting Python 3.12, e.g. Jazzy or newer)
- **Gazebo (`gz sim`)** — the modern Ignition-based Gazebo, not classic Gazebo (the launch file invokes `gz sim`, not `gazebo`)
- `ros_gz_bridge` and `ros_gz_sim` packages
- `colcon` build tools (`python3-colcon-common-extensions`)
- `unzip` (to extract `mar_prj.zip`)

## Installation

```bash
# From the repo root
unzip mar_prj.zip
cd mar_prj

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
- **`colcon build` fails to find `cleaning_robot`** — make sure you extracted the zip and are running `colcon build` from `mar_prj/` (the directory containing `src/`), not from the repo root.

## Known Issues / Inconsistencies

Flagged here rather than silently fixed, since some require a decision about repo layout:

1. **Entire project is a zipped archive, not tracked source** — `mar_prj.zip` is the only content in the repo besides this README. Source code isn't reviewable/diffable on GitHub, and there's no `.gitignore` or CI possible until it's unpacked into normal tracked files.
2. **Build/install/log artifacts are inside the zip** — `mar_prj/build/`, `mar_prj/install/`, and `mar_prj/log/` (with ~10 timestamped build-log folders) are colcon-generated artifacts that were zipped up along with the source. These should never be committed; only `mar_prj/src/` is actual source.
3. **Stray unrelated file** — `src/cleaning_robot/lexer.l` is a Flex lexer for a generic C-like language tokenizer, unrelated to the robot project (looks like it was copied in from a different course assignment).
4. **Broken duplicate launch file** — `launch/simulation.launch.py4` (note the `.py4` extension) is an older draft of `simulation.launch.py` with a trailing syntax error (`])s`) and different, since-corrected ROS↔Gazebo bridge direction operators. It's dead weight and not referenced by `setup.py`'s glob (which only picks up `*.py`).
5. **No license declared** — `package.xml` literally contains `<license>TODO: License declaration</license>`.
6. **No CI/CD or Docker** — no `.github/workflows/`, no Dockerfile; this is expected for a ROS 2/Gazebo simulation project run locally, but worth noting since it's absent.

## Contributing

1. Fork the repository
2. Extract `mar_prj.zip` and make changes under `mar_prj/src/cleaning_robot/`
3. Run the ament lint tests (`colcon test`)
4. Submit a pull request

## License

No license file exists in the repository; `package.xml` marks the license as `TODO`. Add a `LICENSE` file and update `package.xml` if you intend this to be open-source under a specific license.

## Future Improvements

- Unpack `mar_prj.zip` into normal tracked source files and add a `.gitignore` excluding `build/`, `install/`, and `log/`
- Remove the unrelated `lexer.l` file and the broken `simulation.launch.py4` draft
- Add unit tests for the coverage/bypass state machine logic
- Make room dimensions and row parameters configurable instead of hardcoded, so the node isn't tied to one specific `room.sdf`
