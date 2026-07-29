import os
import sys
import termios
import tty

from indoor import Config, Mission
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


def run_customization_wizard():
    """Full interactive wizard used by `--config custom`. Returns a Config."""
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

    os.system('clear')
    return Config(**overrides)