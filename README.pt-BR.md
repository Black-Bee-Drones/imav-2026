<p align="center">

| 🇧🇷 [Português](README.pt-BR.md) | 🇺🇸 [English](README.md) |
|:---:|:---:|

</p>

# IMAV 2026 — Black Bee Drones

**7º lugar — Competição Indoor**

Este repositório contém o software de missão autônoma desenvolvido pela [Black Bee Drones](https://github.com/Black-Bee-Drones) para a [International Micro Air Vehicle Conference and Competition (IMAV 2026)](https://2026.imavs.org/), realizada de 21 a 25 de setembro de 2026 em Estrasburgo, França. A competição outdoor aconteceu no campo de treinamento militar de Haguenau.

<div align="center">
  <img src="assets/overview/blackbee-imav.jpeg" alt="Equipe Black Bee Drones no IMAV 2026"/>
</div>

## Visão Geral da Competição

Esta edição teve como tema operações de resgate em situações de incêndio. As missões indoor são inspiradas em treinamentos de bombeiros; as outdoor seguem uma operação de reconhecimento e primeira resposta a incêndios florestais. As regras completas estão no [Rulebook_IMAV2026_v5.pdf](Rulebook_IMAV2026_v5.pdf).

| # | Indoor | | Outdoor | |
|:---:|---|:---:|---|:---:|
| 1 | Pista de obstáculos: entrada pela janela, barras, tubos | ✓ | Mapeamento e identificação de veículos | |
| 2 | Inspeção da sala escura e contagem de bebês | ✓ | Detecção de focos de incêndio | |
| 3 | Lançamento de cone na caixa com o ponto quente | ✓ | Coleta e lançamento de água | |
| 4 | Inspeção de turbina eólica | | Lançamento de kit de primeiros socorros sobre um manequim com dispositivo "dead man" | ✓ |
| + | Bônus de pouso de precisão | ✓ | Bônus de pouso de precisão | |

✓ implementado neste repositório

## Pacotes

| Pacote | Competição | O que faz | Documentação |
|---|---|---|---|
| [`indoor`](indoor/) | Indoor | Executa as Missões 1 → 2 → 3 em sequência e termina com pouso de precisão em uma plataforma ArUco. Presets nomeados escolhem quais etapas rodam. | [PT](indoor/README.pt-BR.md) · [EN](indoor/README.md) · [Simulação](indoor/simulation/README.md) |
| [`manequim`](manequim/) | Outdoor | Busca o manequim em espiral quadrada, alinha sobre ele, lança o kit de primeiros socorros e retorna à base. | [PT](manequim/README.pt-BR.md) · [EN](manequim/README.md) |

O README de cada pacote traz o hardware, os diagramas das máquinas de estado, os parâmetros que valem ajuste e como rodar em simulação.

## Stack Técnica

- **[Nectar SDK v1.1.0](https://github.com/Black-Bee-Drones/nectar-sdk/releases/tag/v1.1.0)** — SDK ROS 2 da equipe para controle do drone, câmeras e detecção
- **[ROS 2](https://docs.ros.org/)** — middleware
- **[Yasmin](https://github.com/uleroboticsgroup/yasmin)** — máquinas de estado hierárquicas para sequenciar as missões
- **[Ultralytics YOLO](https://docs.ultralytics.com/)** e **[OpenCV](https://opencv.org/)** — detecção de gates, barras, caixas, bebês e manequim; marcadores ArUco
- **[ArduPilot](https://ardupilot.org/)** com **Gazebo** — firmware da controladora de voo e testes software-in-the-loop

## Estrutura do Repositório

```text
imav-2026/
├── indoor/                    # Pacote indoor (ament_python)
│   ├── indoor/                # Código da missão, config.py, presets.py
│   ├── launch/                # Launch da missão e do Gazebo/SITL
│   ├── simulation/            # Arena indoor para o Gazebo
│   └── share/models/          # Pesos dos detectores
├── manequim/                  # Pacote outdoor (ament_python)
│   ├── manequim/              # Código da missão e constantes
│   └── Simulation/            # Mundos do Gazebo
├── assets/                    # Imagens dos READMEs
└── Rulebook_IMAV2026_v5.pdf
```

## Primeiros Passos

Requer um workspace ROS 2 com o [Nectar SDK](https://github.com/Black-Bee-Drones/nectar-sdk) instalado.

```bash
cd ~/ros2_ws
colcon build --packages-select indoor manequim --symlink-install
source install/setup.bash

ros2 run indoor mangalarga --preset FULL      # missão indoor
ros2 run manequim mangalarga                  # missão outdoor
```

Os dois pacotes expõem um executável chamado `mangalarga`; o nome do pacote escolhe qual missão roda. Leia o README do pacote antes de voar: presets e SITL estão no [indoor](indoor/README.pt-BR.md#executando-a-missão), e o `SIM_MODE` (ligado por padrão) está no [manequim](manequim/README.pt-BR.md#executando-a-missão).
