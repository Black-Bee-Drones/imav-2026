# Indoor Drone State Machine — Config & Custom Wizard

This covers how to run `mangalarga` with a built-in preset, and how to use
the new interactive `custom` wizard to build and save your own preset.

## Files

- `mangalarga.py` — entry point / ROS2 node runner, parses CLI args.
- `config.py` — `Config` (a Yasmin `Blackboard`) holding every tunable
  parameter, plus preset/arg application logic.
- `presets.py` — named presets (`CLEITINHO`, `JORGE`, `TESTGATE`,
  `TESTBABIES`, ...) as plain dicts of `Config` overrides. New presets
  saved from the wizard are appended here automatically.
- `customization.py` — the interactive terminal wizard used by
  `--preset custom`.

## Running with a built-in preset

```bash
ros2 run indoor mangalarga --preset JORGE
```

Any name from `presets.py` works: `CLEITINHO`, `JORGE`, `TESTGATE`,
`TESTBABIES`, or any preset you've previously saved via the wizard.

Other useful flags:

```bash
ros2 run indoor mangalarga --sitl                # use simulation connection/topics
ros2 run indoor mangalarga --preset JORGE --sitl # combine with a preset
ros2 run indoor mangalarga -o-skip -i-skip        # skip individual stages directly
```

Run `--help` to see everything:

```bash
ros2 run indoor mangalarga --help
```

## Running the custom wizard

```bash
ros2 run indoor mangalarga --preset custom
```

This launches an arrow-key terminal menu (`↑`/`↓` to move, `Space` to
toggle checkboxes, `Enter` to confirm). It walks you through:

1. **Reuse or start fresh** — pick an existing preset from `presets.py` to
   reuse as-is, or start a brand-new configuration.
2. **Select stages** — choose any combination of Obstacle course, Inspect
   dark room, Dropping on hot spot.
3. **Per-stage questions** — only asked for the stages you selected:
   - *Obstacle course*: window 1 color, red bar height, blue bar 1/2
     heights, whether to attempt tube avoidance, window 2 color.
   - *Inspect*: entry window color, whether to run baby-counting
     inference, exit window color.
   - *Dropping*: whether to drop the cone.
4. **Landing behavior** — Land, RTL, Precision land (fixed platform), or
   Precision land (moving platform).
5. **Optional save** — you'll be asked if you want to save this run's
   configuration. If yes, give it a name (e.g. `custom1`); it's sanitized
   into `SCREAMING_SNAKE_CASE` and appended to `presets.py` as a new
   `dict`, right alongside `CLEITINHO`/`JORGE`/etc.

Once saved, you can skip the wizard next time and run it directly:

```bash
ros2 run indoor mangalarga --preset CUSTOM1
```

It will also appear as a "reuse existing" option the next time you run
`--preset custom`.