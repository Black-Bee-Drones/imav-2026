import random
import math
from pathlib import Path




import yaml



NUMBER_OF_BASES = 3

HOME = Path.home()

MAPPING_CONFIG = Path(
    HOME / "ros2_ws/src/black-bee-challenge-2026/mapping/mapping/config.yml"
)

X_MAX = 14 / 2
X_MIN = -X_MAX

Y_MAX = 14 / 2
Y_MIN = -Y_MAX

BASE_RADIOUS = 2 # meters

MIN_DISTANCE = 9 # meters

Z = 0.2

TEMPLATE_WORLD = Path(
    HOME / "ros2_ws/src/imav-2026/manequim/Simulation/world/empty_world.sdf"
)

OUTPUT_WORLD = Path(
    HOME / "ros2_ws/src/imav-2026/manequim/Simulation/world/world_test.sdf"
)

NECTAR_WORLD = Path(
    HOME / "ros2_ws/install/nectar/share/nectar/simulation/world/world_test.sdf"
)


MODELS_PATH = Path(
    HOME / "ros2_ws/src/imav-2026/manequim/Simulation/models"
)

BASE_MODELS = [
    "firefighter",
    "firefighter",
    "firefighter",
]




def valid_position (x, y, positions):

    for px, py in positions:
        distance = math.sqrt((x - px) ** 2 + (y - py) ** 2)


        if distance < MIN_DISTANCE:
            return False

    return True


def generate_positions(number):
    positions = []

    while len(positions) < number:

        x = random.uniform(
            X_MIN + BASE_RADIOUS,
            X_MAX - BASE_RADIOUS
        )

        y = random.uniform(
            Y_MIN + BASE_RADIOUS,
            Y_MAX - BASE_RADIOUS
        )

        if valid_position(x, y, positions):

            positions.append((x, y))

    return positions



def create_base_includes(models, positions):

    includes = []

    for i, (model_name, (x, y)) in enumerate(
        zip(models, positions)
    ):

        yaw = random.uniform(
            0,
            2 * math.pi
        )

        include = f"""
        <include>

            <name>random_base_{i}</name>

            <uri>model://{model_name}</uri>

            <pose>
                {x} {y} {Z}
                -1.5708 0 {yaw}
            </pose>

        </include>
"""

        includes.append(include)

    return "\n".join(includes)

def generate_world():

    print("Gerando posições das bases...")

    positions = generate_positions(
        NUMBER_OF_BASES
    )

    chosen_models = random.sample(BASE_MODELS, NUMBER_OF_BASES)

    for i, (model_name, (x, y)) in enumerate(zip(chosen_models, positions)):

        print(
            f"Base {i + 1}: "
            f"{model_name} "
            f"x={x:.2f}, "
            f"y={y:.2f}"
        )

    # Lê o mundo original
    world = TEMPLATE_WORLD.read_text()

    # Gera os includes
    includes = create_base_includes(
        chosen_models, positions
    )

    # Procura o marcador
    marker = "<!-- FIREFIGHTER -->"

    # if marker not in world:

    #     raise RuntimeError(
    #         "Marcador <!-- RANDOM_BASES --> "
    #         "não encontrado no mundo."
    #     )

    # # Substitui o marcador
    # world = world.replace(
    #     marker,
    #     includes
    # )

    world_end = "</world>"

    if world_end not in world:
        raise RuntimeError(
            "Tag </world> não encontrada no mundo."
        )

    world = world.replace(
        world_end,
        includes + "\n\n" + world_end,
        1
    )

    OUTPUT_WORLD.write_text(world)

    NECTAR_WORLD.parent.mkdir(
        parents=True,
        exist_ok=True
    )

    NECTAR_WORLD.write_text(world)


    print()
    print("Mundo gerado com sucesso!")
    print()
    print(f"Projeto:")
    print(OUTPUT_WORLD)
    print()
    print(f"Nectar-SDK:")
    print(NECTAR_WORLD)


if __name__ == "__main__":

    generate_world()
