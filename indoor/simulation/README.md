# Indoor simulation (IMAV 2026)

Competition arena for the IMAV 2026 indoor cage (`imav_world.sdf`).

<p align="center">
  <img src="../../assets/overview/indoor-sitl.png" alt="Indoor mission in Gazebo" width="800">
</p>

## Layout

| Path | Purpose |
|------|---------|
| `models/imav/` | Arena mesh + SDF overlays + `imav_world.sdf` (full Gazebo world with iris) |
| `imav_scenery.sdf` | Scenery-only check (no vehicle) |
| `../launch/sitl_gazebo.launch.py` | Loads `imav_world.sdf` via Nectar with vision on |

Arena footprint: **14 × 7 m**, centered at origin.

**Frames.** World origin is the cage center. Spawn / takeoff is on ArUco Id0 at `(-6, 2.29)`.
Takeoff-frame `+X` is world `+X`; takeoff-frame `y = -2.29` is the center strip (`world y = 0`).

| Element | World `x, y` | Takeoff-frame `x, y` |
|---------|--------------|----------------------|
| Spawn / ArUco Id0 (left strip) | -6.0, 2.29 | 0.0, 0.0 |
| First gate | -5.5, 0.0 | 0.5, -2.29 |
| Red bar (step 3, 1.98 m) | -4.5, 0.0 | 1.5, -2.29 |
| Red–blue gap center | -4.0, 0.0 | 2.0, -2.29 |
| Blue bar 1 (step 2, 0.80 m) | -3.5, 0.0 | 2.5, -2.29 |
| Blue bar 2 (step 2, 0.80 m) | -2.5, 0.0 | 3.5, -2.29 |
| Blue–tube gap (depth stop) | -2.2, 0.0 | 3.8, -2.29 |
| Tubes (front cluster) | -1.5, 0.0 | 4.5, -2.29 |
| Tubes (rear vertical) | -0.5, 0.0 | 5.5, -2.29 |
| Second gate | 0.25, 0.0 | 6.25, -2.29 |
| ArUco Id1 (landing strip) | -6.0, -2.30 | 0.0, -4.59 |

Wallpaper (Fig. 8) is tiled along the mission-1 path: center **1.2 m** on `y=0` (two 0.6 m rows), side **0.6 m** at `y=±2.3`. ArUco **5×5**, **0.4 m**, Ids 0–3 on the side strips. Sandy **2.5×2.5 m** at `x=5.75, y=2.25`. Windows (Fig. 12a): OSB panel, inner span **1.5 m**, blue **0.60×0.50 m** and red **0.40×0.40 m**, centers at **z = 1.20 m**.

Spawn is set inside `imav_world.sdf` (`-6 2.29 0.195`).

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


Cameras: `/front_camera/image`, `/front_camera/depth_image`, `/down_camera`.
SITL maps those onto `camera_north_*` / `camera_down_*`. There is no south/C920 camera in Gazebo; `--sitl` skips it.

## Scenery-only check

```bash
cd ~/ros2_ws/src/imav-2026/indoor
export GZ_SIM_RESOURCE_PATH=$PWD/simulation/models:$PWD/simulation:${GZ_SIM_RESOURCE_PATH:-}
gz sim -v4 -r simulation/imav_scenery.sdf
```
