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


SKIP_WINDOW = SITLConfig(
    first_color_window=None,
    red_step=3,
    blue_step1=2,
    blue_step2=2,
    obstacle_avoid_enabled=True,
    second_color_window=None,
    landing_mode=LandingMode.LAND,
    missions=(Mission.OBSTACLES,),
)
