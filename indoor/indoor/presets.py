CLEITINHO = dict(
    droping_skip=True,
    precise_skip=True,
)

JORGE = dict(
    obstacle_skip=True,
    inspect_skip=True,
    precise_skip=True,
)

TESTGATE = dict(
    inspect_skip=True,
    droping_skip=True,
    precise_skip=True,
    rtl=False,
)

TESTBABIES = dict(
    obstacle_skip=True,
    droping_skip=True,
    precise_skip=True,
    rtl=False,
)


COMPLETE_MISSION1 = dict(
    obstacle_skip=False,
    inspect_skip=True,
    droping_skip=True,
    obstacle_gate_first_skip=False,
    obstacle_red=3,
    obstacle_blue_1=2,
    obstacle_blue_2=2,
    obstacle_tubes_skip=False,
    obstacle_gate_second_skip=False,
    model_gate_class_name='blue',
    rtl=False,
    precise_skip=False,
    fixed_base=True,
)
