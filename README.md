Aqui está a tradução do documento para o português:

# Máquina de Estados de Drone Indoor — Configuração e Assistente Customizado

Este documento aborda como executar o `mangalarga` com um preset integrado (*built-in*) e como usar o novo assistente interativo customizado (*custom wizard*) para criar e salvar seu próprio preset.

## Arquivos

* `mangalarga.py` — ponto de entrada / executor do nó ROS2, analisa os argumentos de linha de comando (CLI).
* `config.py` — `Config` (um `Blackboard` do Yasmin) que armazena todos os parâmetros ajustáveis, além da lógica de aplicação de presets e argumentos.
* `presets.py` — presets nomeados (`CLEITINHO`, `JORGE`, `TESTGATE`, `TESTBABIES`, ...) como dicionários (*dicts*) simples de substituições (*overrides*) de `Config`. Novos presets salvos pelo assistente são anexados aqui automaticamente.
* `customization.py` — o assistente de terminal interativo utilizado pela flag `--preset custom`.

## Executando com um preset integrado

```bash
ros2 run indoor mangalarga --preset JORGE
```

Qualquer nome do `presets.py` funcionará: `CLEITINHO`, `JORGE`, `TESTGATE`, `TESTBABIES`, ou qualquer preset que você tenha salvo anteriormente através do assistente.

Outras flags úteis:

```bash
ros2 run indoor mangalarga --sitl                # usa conexão/tópicos de simulação
ros2 run indoor mangalarga --preset JORGE --sitl # combina com um preset
ros2 run indoor mangalarga -o-skip -i-skip       # pula etapas individuais diretamente
```

Execute `--help` para ver todas as opções:

```bash
ros2 run indoor mangalarga --help
```

## Executando o assistente customizado

```bash
ros2 run indoor mangalarga --preset custom
```

Isso inicia um menu de terminal navegável pelas setas do teclado (`↑`/`↓` para mover, `Espaço` para marcar/desmarcar caixas de seleção, `Enter` para confirmar). Ele guiará você pelas seguintes etapas:

1. **Reutilizar ou começar do zero** — escolha um preset existente no `presets.py` para reutilizar como está, ou inicie uma configuração totalmente nova.
2. **Selecionar etapas** — escolha qualquer combinação entre Circuito de obstáculos (*Obstacle course*), Inspecionar sala escura (*Inspect dark room*) e Lançamento no ponto quente (*Dropping on hot spot*).
3. **Perguntas por etapa** — feitas apenas para as etapas que você selecionou:
* *Circuito de obstáculos*: cor da janela 1, altura da barra vermelha, alturas das barras azuis 1 e 2, se deve tentar desviar do tubo, cor da janela 2.
* *Inspeção*: cor da janela de entrada, se deve executar a inferência de contagem de bebês, cor da janela de saída.
* *Lançamento*: se deve soltar o cone.


4. **Comportamento de pouso** — Pousar (*Land*), Retornar ao local de decolagem (*RTL*), Pouso de precisão em plataforma fixa, ou Pouso de precisão em plataforma móvel.
5. **Salvar opcionalmente** — será perguntado se você deseja salvar a configuração desta execução. Se sim, dê um nome a ela (ex: `custom1`); o nome será formatado para `SCREAMING_SNAKE_CASE` e adicionado ao `presets.py` como um novo `dict`, logo ao lado de `CLEITINHO`/`JORGE`/etc.

Uma vez salvo, você pode pular o assistente na próxima vez e executá-lo diretamente:

```bash
ros2 run indoor mangalarga --preset CUSTOM1
```

Ele também aparecerá como uma opção em "reutilizar existente" na próxima vez que você executar `--preset custom`.
