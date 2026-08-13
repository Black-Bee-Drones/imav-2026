from .config import Config, SITLConfig, Mission, LandingMode

CLEITINHO = Config(
    missions=(
        Mission.OBSTACLES,
        Mission.INSPECT,
    )
)

JORGE = Config(
    missions=(
        Mission.DROPPING,
    )
)

SITL_CLEITINHO = SITLConfig(
    missions=(
        Mission.OBSTACLES,
        Mission.INSPECT,
    )
)

SITL_JORGE = SITLConfig(
    missions=(
        Mission.DROPPING,
    )
)


TESTGATE = Config(
    first_color_window='blue',
    red_step=None,
    blue_step_1=None,
    blue_step_2=None,
    tubes_avoid_enabled=False,
    second_color_window=None,
    landing_mode=LandingMode.LAND,
    missions=(Mission.OBSTACLES,),
)
