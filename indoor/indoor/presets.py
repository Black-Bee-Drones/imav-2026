from dataclasses import dataclass
from typing import Optional, Type

from indoor.config import Config


@dataclass
class CLEITINHO(Config):
    droping_skip: bool = True
    precise_skip: bool = True


@dataclass
class JORGE(Config):
    obstacle_skip: bool = True
    inspect_skip: bool = True
    precise_skip: bool = True


@dataclass
class TESTGATE(Config):
    inspect_skip: bool = True
    droping_skip: bool = True
    precise_skip: bool = True
    rtl: bool = False
    obstacle_gate_first_skip: bool = True
    obstacle_gate_second_skip: bool = True


@dataclass
class TESTWINDOWS(Config):
    inspect_skip: bool = True
    droping_skip: bool = True
    precise_skip: bool = True
    rtl: bool = False
    obstacle_gate_first_skip: bool = False
    obstacle_gate_second_skip: bool = False


@dataclass
class TESTBABIES(Config):
    obstacle_skip: bool = True
    droping_skip: bool = True
    precise_skip: bool = True
    rtl: bool = False


@dataclass
class COMPLETE_MISSION1(Config):
    obstacle_skip: bool = False
    inspect_skip: bool = True
    droping_skip: bool = True
    obstacle_gate_first_skip: bool = False
    obstacle_red: Optional[int] = 3
    obstacle_blue_1: Optional[int] = 2
    obstacle_blue_2: Optional[int] = 2
    obstacle_tubes_skip: bool = False
    obstacle_gate_second_skip: bool = False
    model_gate_class_name: str = 'blue'
    rtl: bool = False
    precise_skip: bool = False
    precise_fixed: bool = True


PRESETS: dict[str, Type[Config]] = {
    'CLEITINHO': CLEITINHO,
    'JORGE': JORGE,
    'TESTGATE': TESTGATE,
    'TESTWINDOWS': TESTWINDOWS,
    'TESTBABIES': TESTBABIES,
    'COMPLETE_MISSION1': COMPLETE_MISSION1,
}


def get_preset(name: str) -> Type[Config]:
    if name not in PRESETS:
        raise ValueError(f'Unknown preset "{name}"')
    return PRESETS[name]


def list_presets() -> list[str]:
    return list(PRESETS.keys()) + ['custom']


def register_preset(name: str, cls: Type[Config]) -> None:
    PRESETS[name] = cls
