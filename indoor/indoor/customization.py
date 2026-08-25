import importlib
import os
import pathlib
import re
import sys
import termios
import tty

from indoor import presets


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


# --- Per-stage customization steps -----------------------------------------
# Each function returns a dict of overrides using the *exact* attribute
# names declared on indoor.Config, so the result can be applied directly
# via `config.set(k, v)`.

def customize_obstacle_stage():
    """Obstacle stage: window 1, red bar, blue bar 1/2, tube avoidance,
    window 2."""
    window1 = select_single("Obstacle stage: Window 1 - which color?", _COLOR_OR_SKIP)
    red_bar = select_single("Obstacle stage: Red bar - which height? (skip = don't attempt)", _BAR_STEP_OR_SKIP)
    blue_bar1 = select_single("Obstacle stage: Blue bar 1 - which height?", _BAR_STEP_OR_SKIP)
    blue_bar2 = select_single("Obstacle stage: Blue bar 2 - which height?", _BAR_STEP_OR_SKIP)
    tubes = select_single("Obstacle stage: Attempt the tube obstacle avoidance?", _YES_NO)
    window2 = select_single("Obstacle stage: Window 2 - which color?", _COLOR_OR_SKIP)

    overrides = {
        "obstacle_gate_first_skip": window1 is None,
        "obstacle_red": red_bar,
        "obstacle_blue_1": blue_bar1,
        "obstacle_blue_2": blue_bar2,
        "obstacle_tubes_skip": not tubes,
        "obstacle_gate_second_skip": window2 is None,
    }
    # model_gate_class_name is a single global setting (the detector only
    # tracks one gate color at a time); use whichever window color was
    # actually selected, preferring window1.
    color = window1 or window2
    if color is not None:
        overrides["model_gate_class_name"] = color
    return overrides


def customize_inspect_stage():
    """Inspect stage: entry color, baby inference, exit color."""
    entry = select_single("Inspect stage: Enter through which window color?", _COLOR_OR_SKIP)
    inference = select_single("Inspect stage: Run baby-counting inference?", _YES_NO)
    exit_color = select_single("Inspect stage: Leave through which window color?", _COLOR_OR_SKIP)

    overrides = {
        "obstacle_gate_room_skip": entry is None and exit_color is None,
    }
    if not inference:
        overrides["inspect_babies_count"] = 0
    color = entry or exit_color
    if color is not None:
        overrides["model_gate_class_name"] = color
    return overrides


def customize_dropping_stage():
    """Dropping stage: drop the cone or not."""
    drop = select_single("Dropping stage: Drop the cone?", _YES_NO)
    return {"drop_cone_enabled": drop}


def customize_landing():
    """Landing behavior after the selected stages are done."""
    return select_single(
        "Landing behavior",
        [
            ("Land after mission completion", "land"),
            ("Return To Launch (RTL)", "rtl"),
            ("Precision land - fixed platform", "precision_fixed"),
            ("Precision land - moving platform", "precision_moving"),
        ],
    )


# --- Discovering and reusing pre-built profiles ---------------------------

def discover_saved_profiles():
    """Presets in presets.py are plain UPPER_CASE dicts (see CLEITINHO,
    JORGE, ...). Return {name: dict} for all of them."""
    profiles = {}
    for name in dir(presets):
        if not name.isupper():
            continue
        obj = getattr(presets, name)
        if isinstance(obj, dict):
            profiles[name] = obj
    return profiles


def _describe_profile(name, profile):
    summary = ", ".join(f"{k}={v}" for k, v in profile.items())
    return f"{name}  [{summary}]"


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


# --- Saving a freshly-built configuration back into presets.py ------------

def _format_value(value):
    return repr(value)


def _sanitize_variable_name(raw_name):
    """Turn free-typed input into a valid, SCREAMING_SNAKE_CASE Python
    module-level variable name."""
    name = re.sub(r"[^A-Za-z0-9_]", "_", raw_name.strip()).upper()
    name = name.strip("_") or "CUSTOM"
    if name[0].isdigit():
        name = f"CFG_{name}"
    return name


def render_preset_block(var_name, overrides):
    """Render `VAR_NAME = dict(...)` source text ready to append to
    presets.py, matching the style of CLEITINHO/JORGE/etc."""
    lines = [f"\n\n{var_name} = dict("]
    for key, value in overrides.items():
        lines.append(f"    {key}={_format_value(value)},")
    lines.append(")\n")
    return "\n".join(lines)


def save_profile_to_presets(var_name, overrides):
    """Append the new profile to the end of presets.py on disk."""
    presets_file = pathlib.Path(presets.__file__).resolve()
    if presets_file.suffix in ('.pyc', '.pyo'):
        presets_file = presets_file.with_suffix('.py')

    block = render_preset_block(var_name, overrides)
    with presets_file.open("a") as f:
        f.write(block)

    importlib.reload(presets)


def prompt_save(overrides):
    """Ask whether to persist this run's configuration, and if so, under
    what name. Writes it to presets.py so it shows up in both
    select_initial_profile() (next `--preset custom` run) and directly as
    `--preset NAME` afterwards."""
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
        if var_name == 'CUSTOM' or hasattr(presets, var_name):
            print(f"\n'{var_name}' is already used in presets.py (or reserved).")
            retry = select_single("Pick a different name?", _YES_NO)
            if retry:
                continue
            return
        break

    save_profile_to_presets(var_name, overrides)
    os.system('clear')
    print(f"Saved as {var_name} in presets.py.")
    print(f"Run with --preset {var_name} next time to reuse it directly.")
    input("\nPress Enter to continue...")


def run_customization_wizard():
    """Full interactive wizard used by `--preset custom`.

    First offers a choice between reusing an existing profile (one of the
    dicts already in presets.py) and building a new one from scratch.
    Building a new one ends with an optional save step that appends it to
    presets.py. Always returns a dict of Config attribute overrides.
    """
    chosen_name, profiles = select_initial_profile()
    if chosen_name is not None:
        return profiles[chosen_name]

    stage_choices = select_multi(
        "Select stages to attempt (Space to toggle, Enter to confirm)",
        [
            ("Obstacle course", "obstacle"),
            ("Inspect a dark room", "inspect"),
            ("Dropping on hot spot", "dropping"),
        ],
    )

    if not stage_choices:
        raise ValueError("At least one stage must be selected for a custom run.")

    overrides = {
        "obstacle_skip": "obstacle" not in stage_choices,
        "inspect_skip": "inspect" not in stage_choices,
        "droping_skip": "dropping" not in stage_choices,
    }

    if "obstacle" in stage_choices:
        overrides.update(customize_obstacle_stage())
    if "inspect" in stage_choices:
        overrides.update(customize_inspect_stage())
    if "dropping" in stage_choices:
        overrides.update(customize_dropping_stage())

    landing_mode = customize_landing()
    overrides["rtl"] = landing_mode == "rtl"
    overrides["precise_skip"] = landing_mode not in ("precision_fixed", "precision_moving")
    if landing_mode == "precision_fixed":
        overrides["fixed_base"] = True
    elif landing_mode == "precision_moving":
        overrides["fixed_base"] = False

    prompt_save(overrides)

    os.system('clear')
    return overrides