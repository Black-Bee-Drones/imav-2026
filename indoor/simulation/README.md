# Indoor simulation (IMAV 2026)

Competition arena for the IMAV 2026 indoor cage (`imav_world.sdf`).

## Layout

| Path | Purpose |
|------|---------|
| `models/imav/` | Arena meshes + `imav_world.sdf` (full Gazebo world with iris) |
| `imav_scenery.sdf` | Scenery-only check (no vehicle) |
| `../launch/sitl_gazebo.launch.py` | Loads `imav_world.sdf` via Nectar with vision on |

Arena footprint: **14 × 7 m**, centered at origin. First gate ~`x=-5.5`.
Spawn is set inside `imav_world.sdf` (currently `-6 3.2 0.195`).

## Launch with Nectar

```bash
cd ~/ros2_ws
rm -rf build/indoor install/indoor
colcon build --paths src/imav-2026/indoor --symlink-install
source install/setup.bash
nectar-activate   # or: source ~/ros2_ws/.venv/bin/activate
```

**Terminal 1** (nectar-sdk) — ArduPilot SITL with indoor ExternalNav:

```bash
make sim-start FIRMWARE=ardupilot ENV=indoor
```

**Terminal 2** — preferred: package launch (direct MAVLink). Feeder on SERIAL0
(`tcp:…:5760`); mission / Nectar UI on SERIAL1 (`tcp:…:5762`). Use
`nectar-activate` so `vision_pose_node` can import pymavlink; confirm
`/vision_pose_node` with `ros2 node list` before the mission.

```bash
nectar-activate
ros2 launch indoor sitl_gazebo.launch.py
```

Optional MAVROS:

```bash
ros2 launch indoor sitl_gazebo.launch.py mavros:=true
```

### Equivalent via nectar `make sim-bridge`

Nectar defaults to `PROTOCOL=mavlink`. Pass the installed world path and force
`vision:=true` (required when `world` is a file path, not `world:=indoor`):

```bash
make sim-bridge FIRMWARE=ardupilot ENV=indoor \
  ARGS="world:=$HOME/ros2_ws/install/indoor/share/indoor/simulation/models/imav/imav_world.sdf vision:=true resource_path:=$HOME/ros2_ws/install/indoor/share/indoor/simulation/models"
```

Expected nodes: Gazebo (`imav_world`), `gz_pose_bridge`, `gz_vision_source`,
`vision_pose_node` (mavlink backend). Pose topic:
`/world/imav_world/dynamic_pose/info` → `/visual_slam/tracking/vo_pose_covariance`.

In ideal case, `gz_vision_source.py` is started by the launch above. However, if you want to start it manually, you can do so with the following command:
```bash
python3 ~/ros2_ws/src/nectar-sdk/scripts/simulation/gz_vision_source.py
```


Cameras: `/north_camera/image`, `/down_camera`.

## Scenery-only check

```bash
cd ~/ros2_ws/src/imav-2026/indoor
export GZ_SIM_RESOURCE_PATH=$PWD/simulation/models:$PWD/simulation:${GZ_SIM_RESOURCE_PATH:-}
gz sim -v4 -r simulation/imav_scenery.sdf
```

