<p align="center">

| 🇧🇷 [Português](README.md) | 🇺🇸 [English](README-EN.md) |
|:---:|:---:|

</p>

# IMAV 2026

Software de missão ROS 2 para as competições indoor e outdoor do IMAV 2026. Este repositório contém dois pacotes Python: `indoor` e `manequim` (a missão outdoor de entrega de primeiros socorros).


<div align="center">
  <img src="assets/overview/blackbee-imav.jpeg" alt="BlackBee Drones UNIFEI team at IMAV 2026"/>
</div>

## Documentação

| Seção | 🇧🇷 PT | 🇺🇸 EN |
| --- | --- | --- |
| Overview | [README.md](README.md) | [README-EN.md](README-EN.md) |
| Indoor | [indoor/README.md](indoor/README.md) | [indoor/README-EN.md](indoor/README-EN.md) |
| Outdoor | [manequim/README.md](manequim/README.md) | [manequim/README-EN.md](manequim/README-EN.md) |


## Regras da Competição

A referência oficial incluída neste repositório é o [Rulebook_IMAV2026_v5.pdf](https://www.google.com/search?q=Rulebook_IMAV2026_v5.pdf&utm_source=gemini). Leia-o antes de testar ou voar: ele define o campo de competição, os procedimentos das tarefas, a pontuação e os requisitos de segurança. O software é um auxílio de implementação, não um substituto para as regras ou para as verificações de segurança obrigatórias.

O código atual abrange tarefas selecionadas, não todas as tarefas do livro de regras:

* **Indoor:** pista de obstáculos, inspeção de sala escura, lançamento de pacote em ponto quente (hot-spot) e comportamento de pouso de precisão.
* **Outdoor:** busca por um manequim e entrega de um pacote de primeiros socorros, correspondendo à Missão Outdoor 4 (o manequim possui um dispositivo "dead man").

O livro de regras também descreve outras missões indoor e outdoor que não são representadas por estas máquinas de estado.

## Tecnologias

* **ROS 2 / `rclpy`:** nós de missão, descoberta de pacotes, arquivos de launch e tópicos de imagem do ROS.
* **YASMIN:** máquinas de estado hierárquicas e blackboards para sequenciamento e transições de missão.
* **Nectar SDK:** controle do drone, integração MAVLink, fontes de câmera/visão e interfaces de detecção usadas pelos estados da missão.
* **ArduPilot SITL e Gazebo Sim:** testes de voo software-in-the-loop; o pacote indoor inclui uma arena Gazebo personalizada e integração de launch do Nectar.
* **Visão computacional:** detectores de objetos baseados em aprendizado para portões, barras, pessoas, bebês e alvos de lançamento; dados de câmera/profundidade e marcadores ArUco também são usados pela navegação indoor e estados de pouso.

## Estrutura do Repositório

```text
indoor/
    indoor/indoor_sm.py          Máquina de estados de missão indoor de nível superior
    indoor/missions/             Submáquinas de obstáculo, inspeção, lançamento e pouso
    indoor/states/               Estados de voo e missão reutilizáveis
    indoor/config.py             Parâmetros da missão, predefinições e configuração da CLI
    indoor/presets.py            Configurações nomeadas
    launch/                      Arquivos de launch da missão e Gazebo/SITL
    simulation/                  Modelos SDF e Gazebo da arena indoor
    share/models/                Arquivos de modelo do detector

manequim/
    manequim/mangalarga.py       Nó ROS 2 outdoor e máquina de estados de nível superior
    manequim/searchSM/           Busca, navegação e detecção de manequim
    manequim/packageSM/          Aproximação, alinhamento e liberação do pacote
    manequim/core/               Inicialização, decolagem, retorno à base (RTL) e limpeza
    Simulation/world/            Arquivos SDF do mundo outdoor no Gazebo
    Simulation/models/           Modelo local do bombeiro

```

Ambos os pacotes usam o tipo de build `ament_python` e expõem um executável de console chamado `mangalarga`; selecione o nome do pacote ROS ao executá-lo.

## Configuração e Build

Use um ambiente ROS 2 com o `colcon` e o Nectar SDK instalados e configurados (sourced). As dependências Python/ROS do `nectar` são fornecidas por esse ambiente, em vez de estarem embutidas neste repositório.

```bash
cd ~/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select indoor manequim --symlink-install
source install/setup.bash

```

Para o fluxo de trabalho SITL indoor, ative o ambiente Nectar conforme descrito nas [notas de simulação indoor](https://www.google.com/search?q=indoor/simulation/README.md&utm_source=gemini). Essas notas incluem a ponte SITL esperada, os tópicos e uma verificação de cenário isolada do Gazebo.

## Missão Indoor

A máquina de nível superior indoor inicializa o drone, decola e, em seguida, executa as submáquinas de obstáculos, inspeção de sala e lançamento em hot-spot, antes do comportamento configurado de pouso de precisão/pouso normal. As opções e os parâmetros de controle/detector estão reunidos em `indoor/config.py`; as configurações nomeadas residem em `indoor/presets.py`.

Execute o nó da missão diretamente:

```bash
ros2 run indoor mangalarga --help
ros2 run indoor mangalarga --sitl --preset JORGE

```

Use qualquer predefinição (preset) listada pelo pacote, ou inicie o assistente de configuração interativo com `--preset custom`. É possível pular etapas usando `--obstacle-skip`, `--inspect-skip`, `--droping-skip` e `--precise-skip`; a opção `--no-takeoff` é destinada a fluxos de trabalho controlados onde a decolagem é tratada separadamente. Revise as configurações selecionadas antes do voo.

O pacote também possui pontos de entrada (entry points) de launch para os nós da missão:

```bash
ros2 launch indoor mangalarga.launch.py
ros2 launch indoor simulation.launch.py

```

Esses arquivos de launch iniciam o **nó da missão**; eles não iniciam o Gazebo. Há um conflito de argumentos atualmente: os wrappers passam `--config`, enquanto o nó aceita `--preset`, portanto `config:=...` não seleciona uma predefinição. Use `ros2 run` para escolher uma predefinição até que os wrappers sejam alinhados. Para iniciar o mundo indoor e o SITL, siga o procedimento completo, terminal por terminal, em [indoor/simulation/README.md](https://www.google.com/search?q=indoor/simulation/README.md&utm_source=gemini). O seu comando principal de launch do Gazebo é:

```bash
ros2 launch indoor sitl_gazebo.launch.py

```

## Missão Outdoor do Manequim

A máquina de estados outdoor inicializa e decola, busca pelo manequim alvo e, em seguida, executa a sequência de entrega do pacote: descida, alinhamento, liberação e retorno à base. Os estados de busca e entrega estão separados em `searchSM/` e `packageSM/` para que a navegação e o comportamento de novas tentativas (retry) possam ser desenvolvidos de forma independente.

Execute-a depois que a conexão com o veículo outdoor e o ambiente de câmera/detector estiverem prontos:

```bash
ros2 run manequim mangalarga --help
ros2 run manequim mangalarga --manequim-number 1

```

A opção `--manequim-number` seleciona o índice do alvo (1–3). Opcionalmente, passe tanto `--latitude LATITUDE` quanto `--longitude LONGITUDE` para definir a coordenada de busca inicial. O modelo do detector pode ser selecionado com `MANEQUIM_DETECTOR_MODEL`; caso contrário, a configuração de simulação atual espera `~/ros2_ws/yolo26n.pt`.

### Status da Simulação Outdoor

O pacote outdoor contém arquivos de mundo do Gazebo em `manequim/Simulation/world/` e um modelo local de bombeiro, mas atualmente não fornece um arquivo de launch ROS 2 outdoor ou um processo de inicialização (bring-up) SITL de ponta a ponta documentado. O arquivo `world_test.sdf` também referencia recursos do Gazebo, como `outdoor_field_scenery` e `iris_with_gimbal`, que devem estar disponíveis a partir do ambiente de simulação Nectar/ArduPilot. Assim que os caminhos apropriados de recursos do modelo e SITL estiverem em execução, inicie o nó da missão com `ros2 run manequim mangalarga ...` como mostrado acima.

A configuração outdoor atual define `SIM_MODE = True` em `manequim/core/constants.py`. Trate isso como algo orientado para simulação: não presuma que o pacote está pronto para voos outdoor reais sem antes revisar e configurar seus parâmetros de conexão, câmera, detector e segurança. O script auxiliar `manequim/Simulation/script/random_base_location.py` depende adicionalmente de um pacote de mapeamento separado e de caminhos do workspace local, portanto não é um comando independente para geração de mundo.

> **Espaço para imagem:** Adicione uma visão do Gazebo do campo outdoor com os manequins alvos e a área de entrega. Substitua esta nota por uma imagem quando estiver disponível.

## Desenvolvimento e Visualização

O nó outdoor publica a visualização de sua máquina de estados como `MANGALARGA_FSM`; o visualizador YASMIN pode ser usado para inspecionar as transições de estado durante a execução. Para verificações rápidas da geometria da arena indoor sem o SITL, use o comando exclusivo de cenário mencionado nas notas de simulação indoor. Para simulação de voo, inicie o simulador/ponte primeiro e inicie o nó da missão correspondente em um terminal separado.

Após alterações no código, recompile o pacote afetado com `colcon build --packages-select indoor` ou `colcon build --packages-select manequim` e, em seguida, faça o source de `install/setup.bash` novamente.