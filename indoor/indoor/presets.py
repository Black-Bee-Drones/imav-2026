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
