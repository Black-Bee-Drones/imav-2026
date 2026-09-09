from dataclasses import asdict, fields
import os
import pathlib
import re
import sys
import termios
import tty

from indoor import presets
from indoor.config import Config


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
# Each function returns a dict of overrides using the *exact* field names
# declared on indoor.config.Config, so the result can be applied directly
# via `dataclasses.replace(config, **overrides)`.

def customize_obstacle_stage():
    """Obstacle stage: window 1, red bar, blue bar 1/2, tube avoidance,
    window 2."""
    window1 = select_single("Obstacle stage: Window 1 - which color?", _COLOR_OR_SKIP)
    red_bar = select_single("Obstacle stage: Red bar - which height? (skip = don't attempt)", _BAR_STEP_OR_SKIP)
    blue_bar1 = select_single("Obstacle stage: Blue bar 1 - which height?", _BAR_STEP_OR_SKIP)
    bar_center_skip = select_single("Obstacle stage: Skip bar centralization?", _YES_NO)
    tubes = select_single("Obstacle stage: Attempt the tube obstacle avoidance?", _YES_NO)
    window2 = select_single("Obstacle stage: Window 2 - which color?", _COLOR_OR_SKIP)

    overrides = {
        "obstacle_gate_first_skip": window1 is None,
        "obstacle_red": red_bar,
        "obstacle_blue_1": blue_bar1,
        "obstacle_bar_center_skip": bar_center_skip,
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

# Fields that are runtime state, not something a saved preset should pin.
_RUNTIME_FIELDS = {
    'inspect_babies_output_path',
    'obstacle_start_time',
    'inspect_start_time',
    'precise_start_time',
}


def _describe_profile(name, cls):
    # Only show fields the preset actually overrides vs. base Config, to
    # keep the menu readable.
    base = asdict(Config())
    this = asdict(cls())
    diff = {k: v for k, v in this.items() if k not in _RUNTIME_FIELDS and base.get(k) != v}
    summary = ", ".join(f"{k}={v}" for k, v in diff.items())
    return f"{name}  [{summary}]"


def select_initial_profile():
    options = [("Start a new custom configuration", None)]
    for name in sorted(presets.PRESETS):
        options.append((_describe_profile(name, presets.PRESETS[name]), name))

    chosen_name = select_single(
        "Custom configuration - start fresh or reuse an existing one?",
        options,
    )
    if chosen_name is None:
        return None

    # Return as a plain overrides dict (relative to base Config) so the
    # caller can apply it the same way as a freshly-built configuration.
    base = asdict(Config())
    this = asdict(presets.PRESETS[chosen_name]())
    return {k: v for k, v in this.items() if k not in _RUNTIME_FIELDS and base.get(k) != v}


# --- Saving a freshly-built configuration back into presets.py ------------

def _sanitize_class_name(raw_name):
    """Turn free-typed input into a valid, SCREAMING_SNAKE_CASE Python
    class name (matches the style of CLEITINHO/JORGE/... in presets.py)."""
    name = re.sub(r"[^A-Za-z0-9_]", "_", raw_name.strip()).upper()
    name = name.strip("_") or "CUSTOM"
    if name[0].isdigit():
        name = f"CFG_{name}"
    return name


def _annotation_for(value):
    if value is None:
        return "object"
    return type(value).__name__


def render_preset_class(class_name, overrides):
    """Render `@dataclass\nclass NAME(Config): ...` source text ready to
    append to presets.py, matching the style of CLEITINHO/JORGE/etc."""
    lines = ["\n\n@dataclass", f"class {class_name}(Config):"]
    for key, value in overrides.items():
        lines.append(f"    {key}: {_annotation_for(value)} = {value!r}")
    return "\n".join(lines) + "\n"


def save_profile_to_presets(class_name, overrides):
    """Append the new preset class to the end of presets.py on disk, and
    register it in presets.PRESETS so it's usable for the rest of this
    process too."""
    presets_file = pathlib.Path(presets.__file__).resolve()

    block = render_preset_class(class_name, overrides)
    with presets_file.open("a") as f:
        f.write(block)

    # Build the equivalent class in-memory rather than re-importing the
    # module (simpler than reload()-ing something that may already have
    # live instances floating around).
    from dataclasses import make_dataclass
    field_specs = [(k, _eval_annotation(v), v) for k, v in overrides.items()]
    new_cls = make_dataclass(class_name, [(k, t) for k, t, _ in field_specs],
                              bases=(Config,))
    for k, _t, v in field_specs:
        new_cls.__dataclass_fields__[k].default = v
    presets.register_preset(class_name, new_cls)


def _eval_annotation(value):
    return object if value is None else type(value)


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

        class_name = _sanitize_class_name(raw_name)
        if class_name == 'CUSTOM' or class_name in presets.PRESETS:
            print(f"\n'{class_name}' is already used in presets.py (or reserved).")
            retry = select_single("Pick a different name?", _YES_NO)
            if retry:
                continue
            return
        break

    save_profile_to_presets(class_name, overrides)
    os.system('clear')
    print(f"Saved as {class_name} in presets.py.")
    print(f"Run with --preset {class_name} next time to reuse it directly.")
    input("\nPress Enter to continue...")


def run_customization_wizard():
    """Full interactive wizard used by `--preset custom`.

    First offers a choice between reusing an existing preset (one of the
    dataclasses already in presets.py) and building a new one from
    scratch. Building a new one ends with an optional save step that
    appends it to presets.py. Always returns a dict of Config field
    overrides, ready for `dataclasses.replace(Config(), **overrides)`.
    """
    initial = select_initial_profile()
    if initial is not None:
        return initial

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

    landing_mode = customize_landing()
    overrides["rtl"] = landing_mode == "rtl"
    overrides["precise_skip"] = landing_mode not in ("precision_fixed", "precision_moving")
    if landing_mode == "precision_fixed":
        overrides["precise_fixed"] = True
    elif landing_mode == "precision_moving":
        overrides["precise_fixed"] = False

    prompt_save(overrides)

    os.system('clear')
    return overrides