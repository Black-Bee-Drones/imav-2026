# Indoor simulation (IMAV 2026)

Competition scenery only for the IMAV 2026 indoor cage.

## Layout

| Path | Purpose |
|------|---------|
| `models/imav/` | Static arena (`model.config` + `model.sdf` + meshes/textures) |
| `imav_scenery.sdf` | Scenery-only  |
| `../launch/sitl_gazebo.launch.py` | Nectar indoor + `model://imav` with default spawn |

Arena footprint: **14 × 7 m**, centered at origin. First gate ~`x=-5.5`. Default spawn **`-6.2 0 0.25 0 0 0`** (before the gate, inside the floor box).

## Launch with Nectar

```bash
cd ~/ros2_ws
colcon build --paths src/imav-2026/indoor --symlink-install
source install/setup.bash
```

**Terminal 1** (nectar-sdk):

```bash
make sim-start FIRMWARE=ardupilot ENV=indoor
```

**Terminal 2** — IMAV defaults (scenery + spawn + resource path):

```bash
ros2 launch indoor sitl_gazebo.launch.py
```

Override spawn if needed:

```bash
ros2 launch indoor sitl_gazebo.launch.py spawn_pose:="-6.0 0 0.25 0 0 0"
```

Equivalent via nectar `make` (explicit args):

```bash
make sim-bridge FIRMWARE=ardupilot ENV=indoor \
  ARGS="scenery:=model://imav spawn_pose:='-6.2 0 0.25 0 0 0' resource_path:=$HOME/ros2_ws/install/indoor/share/indoor/simulation/models"
```

Cameras: `/front_camera/image`, `/down_camera`.

## Scenery-only check

```bash
cd ~/ros2_ws/src/imav-2026/indoor
export GZ_SIM_RESOURCE_PATH=$PWD/simulation/models:$PWD/simulation:${GZ_SIM_RESOURCE_PATH:-}
gz sim -v4 -r simulation/imav_scenery.sdf
```
