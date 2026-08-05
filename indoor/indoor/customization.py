import importlib
import inspect
import os
import pathlib
import re
import sys
import termios
import tty

from indoor import Config, SITLConfig, Mission
from indoor import presets
from indoor.config import LandingMode


def get_key_input():
    """Capture a single keypress without needing Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
        if key == '\x1b':
            key += sys.stdin.read(2)
        return key
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def select_single(title, options):
    """Arrow-key single-choice menu.

    `options` is a list of (label, value) pairs. Returns the chosen value.
    """
    selected_index = 0
    while True:
        os.system('clear')
        print("=" * 50)
        print(title)
        print("=" * 50)
        print("\nUse \u2191/\u2193 to navigate, Enter to confirm\n")
        for i, (label, _value) in enumerate(options):
            if i == selected_index:
                print(f"  \u25b6 {label} \u25c0")
            else:
                print(f"    {label}")

        key = get_key_input()
        if key == '\x1b[A':  # Up arrow
            selected_index = (selected_index - 1) % len(options)
        elif key == '\x1b[B':  # Down arrow
            selected_index = (selected_index + 1) % len(options)
        elif key in ('\r', '\n'):
            return options[selected_index][1]
        elif key == '\x03':
            raise KeyboardInterrupt("Selection cancelled by user")


def select_multi(title, options):
    """Arrow-key multi-choice checklist. Space toggles, Enter confirms.

    `options` is a list of (label, value) pairs. Returns the list of
    chosen values, in `options` order.
    """
    checked = [False] * len(options)
    selected_index = 0
    while True:
        os.system('clear')
        print("=" * 50)
        print(title)
        print("=" * 50)
        print("\nUse \u2191/\u2193 to navigate, Space to toggle, Enter to confirm\n")
        for i, (label, _value) in enumerate(options):
            box = '[x]' if checked[i] else '[ ]'
            cursor = '\u25b6' if i == selected_index else ' '
            print(f"  {cursor} {box} {label}")

        key = get_key_input()
        if key == '\x1b[A':
            selected_index = (selected_index - 1) % len(options)
        elif key == '\x1b[B':
            selected_index = (selected_index + 1) % len(options)
        elif key == ' ':
            checked[selected_index] = not checked[selected_index]
        elif key in ('\r', '\n'):
            return [options[i][1] for i, c in enumerate(checked) if c]
        elif key == '\x03':
            raise KeyboardInterrupt("Selection cancelled by user")


# --- Reusable option sets -------------------------------------------------

_COLOR_OR_SKIP = [
    ("Blue window", "blue"),
    ("Red window", "red"),
    ("Skip this checker", None),
]

_BAR_STEP_OR_SKIP = [
    ("Low", 1),
    ("Middle", 2),
    ("High", 3),
    ("Skip this checker", None),
]

_YES_NO = [
    ("Yes", True),
    ("No", False),
]


# --- Per-mission customization steps --------------------------------------

def customize_mission1():
    """Mission 1 - Obstacle course: window 1, red bar, blue bar 1/2,
    obstacle avoidance, window 2."""
    window1 = select_single("Mission 1: Window 1 - which color?", _COLOR_OR_SKIP)
    red_bar = select_single("Mission 1: Red bar - which height? (skip = don't attempt)", _BAR_STEP_OR_SKIP)
    blue_bar1 = select_single("Mission 1: Blue bar 1 - which height?", _BAR_STEP_OR_SKIP)
    blue_bar2 = select_single("Mission 1: Blue bar 2 - which height?", _BAR_STEP_OR_SKIP)
    obstacle = select_single("Mission 1: Attempt the tube obstacle avoidance?", _YES_NO)
    window2 = select_single("Mission 1: Window 2 - which color?", _COLOR_OR_SKIP)

    return {
        "first_color_window": window1,
        "red_step": red_bar,
        "blue_step1": blue_bar1,
        "blue_step2": blue_bar2,
        "obstacle_avoid_enabled": obstacle,
        "second_color_window": window2,
    }


def customize_mission2():
    """Mission 2 - Inspect a dark room: entry color, baby inference, exit color."""
    entry = select_single("Mission 2: Enter through which window color?", _COLOR_OR_SKIP)
    inference = select_single("Mission 2: Run baby-counting inference?", _YES_NO)
    exit_color = select_single("Mission 2: Leave through which window color?", _COLOR_OR_SKIP)

    return {
        "room_entry_color": entry,
        "run_baby_inference": inference,
        "room_exit_color": exit_color,
    }


def customize_mission3():
    """Mission 3 - Dropping on hot spot: drop the cone or not."""
    drop = select_single("Mission 3: Drop the cone?", _YES_NO)
    return {"drop_cone_enabled": drop}


def customize_landing():
    """Landing behavior after the selected missions are done."""
    return select_single(
        "Landing behavior",
        [
            ("Land after mission completion", LandingMode.LAND),
            ("Return To Launch (RTL)", LandingMode.RTL),
            ("Precision land - fixed platform", LandingMode.PRECISION_FIXED),
            ("Precision land - moving platform", LandingMode.PRECISION_MOVING),
        ],
    )


# --- Discovering and reusing pre-built profiles ---------------------------

def discover_saved_profiles():
    profiles = {}
    for name, obj in inspect.getmembers(presets):
        if name.startswith('_'):
            continue
        if isinstance(obj, Config):
            profiles[name] = obj
    return profiles


def _describe_profile(name, profile):
    missions_str = ", ".join(m.value for m in profile.missions)
    return f"{name}  [{missions_str}]"


def select_initial_profile():
    profiles = discover_saved_profiles()
    options = [("Start a new custom configuration", None)]
    for name in sorted(profiles):
        options.append((_describe_profile(name, profiles[name]), name))

    chosen_name = select_single(
        "Custom configuration - start fresh or reuse an existing one?",
        options,
    )
    return chosen_name, profiles


# --- Saving a freshly-built configuration back into config.py -------------

def _format_value(value):
    if isinstance(value, Mission):
        return f"Mission.{value.name}"
    if isinstance(value, LandingMode):
        return f"LandingMode.{value.name}"
    if isinstance(value, tuple):
        inner = ", ".join(_format_value(v) for v in value)
        return f"({inner},)" if len(value) == 1 else f"({inner})"
    return repr(value)


def _sanitize_variable_name(raw_name):
    """Turn free-typed input into a valid, SCREAMING_SNAKE_CASE Python
    module-level variable name."""
    name = re.sub(r"[^A-Za-z0-9_]", "_", raw_name.strip()).upper()
    name = name.strip("_") or "CUSTOM"
    if name[0].isdigit():
        name = f"CFG_{name}"
    return name


def render_config_block(var_name, overrides, class_name):
    """Render `VAR_NAME = ClassName(...)` source text ready to append to
    config.py, matching the style of CLEITINHO/JORGE/etc."""
    lines = [f"\n\n{var_name} = {class_name}("]
    for key, value in overrides.items():
        lines.append(f"    {key}={_format_value(value)},")
    lines.append(")\n")
    return "\n".join(lines)


def save_profile_to_config(var_name, overrides, class_name):
    """Append the new profile to the end of presets.py on disk."""
    presets_file = pathlib.Path(presets.__file__).resolve()
    if presets_file.suffix in ('.pyc', '.pyo'):
        presets_file = presets_file.with_suffix('.py')

    block = render_config_block(var_name, overrides, class_name)
    with presets_file.open("a") as f:
        f.write(block)

    importlib.reload(presets)


def prompt_save(overrides, class_name):
    """Ask whether to persist this run's configuration, and if so, under
    what name. Writes it to config.py so it shows up in
    select_initial_profile() on future runs."""
    save = select_single("Save this configuration for future runs?", _YES_NO)
    if not save:
        return

    while True:
        os.system('clear')
        print("=" * 50)
        print("Save configuration")
        print("=" * 50)
        raw_name = input("\nName for this configuration (e.g. custom1): ").strip()
        if not raw_name:
            print("Name cannot be empty.")
            input("Press Enter to try again...")
            continue

        var_name = _sanitize_variable_name(raw_name)
        if hasattr(presets, var_name):
            print(f"\n'{var_name}' is already used in config.py.")
            retry = select_single("Pick a different name?", _YES_NO)
            if retry:
                continue
            return
        break

    save_profile_to_config(var_name, overrides, class_name)
    os.system('clear')
    print(f"Saved as {var_name} in config.py.")
    print(f"Run with --config custom and pick '{var_name}' next time to reuse it.")
    input("\nPress Enter to continue...")


def run_customization_wizard():
    """Full interactive wizard used by `--config custom`.

    First offers a choice between reusing an existing profile (built-in or
    previously saved) and building a new one from scratch. Building a new
    one ends with an optional save step that appends it to config.py.
    Returns a Config either way.
    """
    chosen_name, profiles = select_initial_profile()
    if chosen_name is not None:
        return profiles[chosen_name]

    # Added selection for Environment Type (Config vs SITLConfig)
    config_class = select_single(
        "Select Environment Type",
        [
            ("Real Hardware (Config)", Config),
            ("Simulation (SITLConfig)", SITLConfig),
        ]
    )

    mission_choices = select_multi(
        "Select missions to attempt (Space to toggle, Enter to confirm)",
        [
            ("Mission 1 - Obstacle course", Mission.OBSTACLES),
            ("Mission 2 - Inspect a dark room", Mission.INSPECT),
            ("Mission 3 - Dropping on hot spot", Mission.DROPPING),
        ],
    )

    if not mission_choices:
        raise ValueError("At least one mission must be selected for a custom run.")

    overrides = {}

    if Mission.OBSTACLES in mission_choices:
        overrides.update(customize_mission1())
    if Mission.INSPECT in mission_choices:
        overrides.update(customize_mission2())
    if Mission.DROPPING in mission_choices:
        overrides.update(customize_mission3())

    landing_mode = customize_landing()
    overrides["landing_mode"] = landing_mode
    if landing_mode == LandingMode.PRECISION_FIXED:
        overrides["fixed_base"] = True
    elif landing_mode == LandingMode.PRECISION_MOVING:
        overrides["fixed_base"] = False

    overrides["missions"] = tuple(mission_choices)

    # Instantiate using the user-selected class
    config = config_class(**overrides)

    # Pass the class name so it writes correctly to config.py
    prompt_save(overrides, config_class.__name__)

    os.system('clear')
    return config