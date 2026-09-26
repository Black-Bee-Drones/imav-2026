<p align="center">

| 🇧🇷 [Português](README.pt-BR.md) | 🇺🇸 [English](README.md) |
|:---:|:---:|

</p>

# IMAV 2026 — Black Bee Drones

**7th Place — Indoor Competition**

This repository contains the autonomous mission software developed by [Black Bee Drones](https://github.com/Black-Bee-Drones) for the [International Micro Air Vehicle Conference and Competition (IMAV 2026)](https://2026.imavs.org/), held September 21–25, 2026 in Strasbourg, France. The outdoor competition took place at the Haguenau military training ground.

<div align="center">
  <img src="assets/overview/blackbee-imav.jpeg" alt="Black Bee Drones team at IMAV 2026"/>
</div>

## Competition Overview

This edition focused on rescue operations in fire conditions. The indoor missions are inspired by firefighter training courses; the outdoor missions follow a forest-fire reconnaissance and first-response operation. The full rules are in [Rulebook_IMAV2026_v5.pdf](Rulebook_IMAV2026_v5.pdf).

| # | Indoor | | Outdoor | |
|:---:|---|:---:|---|:---:|
| 1 | Obstacle course: window entry, bars, tubes | ✓ | Mapping and vehicle identification | |
| 2 | Dark room inspection and baby count | ✓ | Fire detection | |
| 3 | Cone drop into the hot-spot box | ✓ | Collect and drop water | |
| 4 | Wind turbine inspection | | First aid kit drop on a mannequin with a “dead man” device | ✓ |
| + | Precision landing bonus | ✓ | Precision landing bonus | |

✓ implemented in this repository

## Packages

| Package | Competition | What it does | Docs |
|---|---|---|---|
| [`indoor`](indoor/) | Indoor | Flies Missions 1 → 2 → 3 in sequence and ends with a precision landing on an ArUco platform. Named presets select which stages run. | [EN](indoor/README.md) · [PT](indoor/README.pt-BR.md) · [Simulation](indoor/simulation/README.md) |
| [`manequim`](manequim/) | Outdoor | Searches for the mannequin with a square spiral, aligns over it, drops the first aid kit, and returns to launch. | [EN](manequim/README.md) · [PT](manequim/README.pt-BR.md) |

Each package README covers the hardware, state machine diagrams, the parameters worth tuning, and how to run in simulation.

## Technical Stack

- **[Nectar SDK v1.1.0](https://github.com/Black-Bee-Drones/nectar-sdk/releases/tag/v1.1.0)** — the team's ROS 2 SDK for drone control, cameras, and detection
- **[ROS 2](https://docs.ros.org/)** — middleware
- **[Yasmin](https://github.com/uleroboticsgroup/yasmin)** — hierarchical state machines for mission sequencing
- **[Ultralytics YOLO](https://docs.ultralytics.com/)** and **[OpenCV](https://opencv.org/)** — gate, bar, box, baby, and mannequin detection; ArUco markers
- **[ArduPilot](https://ardupilot.org/)** with **Gazebo** — flight controller firmware and software-in-the-loop testing

## Repository Layout

```text
imav-2026/
├── indoor/                    # Indoor package (ament_python)
│   ├── indoor/                # Mission code, config.py, presets.py
│   ├── launch/                # Mission and Gazebo/SITL launch files
│   ├── simulation/            # Indoor arena for Gazebo
│   └── share/models/          # Detector weights
├── manequim/                  # Outdoor package (ament_python)
│   ├── manequim/              # Mission code and constants
│   └── Simulation/            # Gazebo world files
├── assets/                    # README images
└── Rulebook_IMAV2026_v5.pdf
```

## Getting Started

Requires a ROS 2 workspace with the [Nectar SDK](https://github.com/Black-Bee-Drones/nectar-sdk) installed.

```bash
cd ~/ros2_ws
colcon build --packages-select indoor manequim --symlink-install
source install/setup.bash

ros2 run indoor mangalarga --preset FULL      # indoor mission
ros2 run manequim mangalarga                  # outdoor mission
```

Both packages expose an executable named `mangalarga`; the package name selects which mission runs. Read the package README before flying: presets and SITL setup are in [indoor](indoor/README.md#running-the-mission), and `SIM_MODE` (on by default) is in [manequim](manequim/README.md#running-the-mission).
