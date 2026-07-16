# IMAV 2026

## Indoor State Machine

The `IndoorSM` is the main orchestrator for the IMAV 2026 Indoor Competition. Instead of being a hardcoded sequence of events, this state machine is **dynamically configurable**. 

You can pass any combination of mission tasks into the `config.missions` array. The state machine will automatically link them together, ensuring that upon `SUCCEED`, the drone progresses to the next configured task. If a task is `CANCEL`led, the drone will automatically attempt a precise landing. 

For the competition, we are deploying two drones with distinct mission configurations. 

### 1. First Drone: Obstacle & Inspection Mission
**Configuration:** `['OBSTACLESM', 'INSPECTSM', 'PRECISELANDINGSM']`

Drone 1 is tasked with navigating the obstacle course and inspecting the designated room before returning for a precise landing.

```mermaid
stateDiagram-v2
    %% Custom styles
    classDef success fill:#2ea043,stroke:#238636,stroke-width:2px,color:#fff
    classDef danger fill:#da3633,stroke:#b62324,stroke-width:2px,color:#fff
    classDef warning fill:#d29922,stroke:#9e6a03,stroke-width:2px,color:#fff

    [*] --> INITIALIZE

    INITIALIZE --> TAKEOFF : SUCCEED
    INITIALIZE --> ABORT : ABORT

    TAKEOFF --> OBSTACLESM : SUCCEED
    TAKEOFF --> LAND : ABORT

    OBSTACLESM --> INSPECTSM : SUCCEED
    OBSTACLESM --> PRECISELANDINGSM : CANCEL
    OBSTACLESM --> LAND : ABORT
    OBSTACLESM --> TIMEOUT : TIMEOUT

    INSPECTSM --> PRECISELANDINGSM : SUCCEED
    INSPECTSM --> PRECISELANDINGSM : CANCEL
    INSPECTSM --> LAND : ABORT
    INSPECTSM --> TIMEOUT : TIMEOUT

    PRECISELANDINGSM --> LAND : SUCCEED
    PRECISELANDINGSM --> LAND : ABORT
    PRECISELANDINGSM --> TIMEOUT : TIMEOUT

    LAND --> SUCCEED : SUCCEED
    LAND --> ABORT : ABORT

    %% Apply styles to outcomes
    class SUCCEED success
    class ABORT danger
    class TIMEOUT warning

```

---

### 2. Second Drone: Payload Drop Mission

**Configuration:** `['DROPPINGSM', 'PRECISELANDINGSM']`

Drone 2 bypasses the obstacles and focuses entirely on navigating to the target zone to drop the payload before initiating its precise landing sequence.

```mermaid
stateDiagram-v2
    %% Custom styles
    classDef success fill:#2ea043,stroke:#238636,stroke-width:2px,color:#fff
    classDef danger fill:#da3633,stroke:#b62324,stroke-width:2px,color:#fff
    classDef warning fill:#d29922,stroke:#9e6a03,stroke-width:2px,color:#fff

    [*] --> INITIALIZE

    INITIALIZE --> TAKEOFF : SUCCEED
    INITIALIZE --> ABORT : ABORT

    TAKEOFF --> DROPPINGSM : SUCCEED
    TAKEOFF --> LAND : ABORT

    DROPPINGSM --> PRECISELANDINGSM : SUCCEED
    DROPPINGSM --> PRECISELANDINGSM : CANCEL
    DROPPINGSM --> LAND : ABORT
    DROPPINGSM --> TIMEOUT : TIMEOUT

    PRECISELANDINGSM --> LAND : SUCCEED
    PRECISELANDINGSM --> LAND : ABORT
    PRECISELANDINGSM --> TIMEOUT : TIMEOUT

    LAND --> SUCCEED : SUCCEED
    LAND --> ABORT : ABORT

    %% Apply styles to outcomes
    class SUCCEED success
    class ABORT danger
    class TIMEOUT warning

```


## Obstacle State Machine (Mission 1)
```mermaid
stateDiagram-v2
    %% Custom styles
    classDef success fill:#2ea043,stroke:#238636,stroke-width:2px,color:#fff
    classDef cancel fill:#f28c28,stroke:#b25f12,stroke-width:2px,color:#fff

    [*] --> FIRST_WINDOW

    FIRST_WINDOW --> RED_BAR : SUCCEED
    FIRST_WINDOW --> CANCEL : TIMEOUT

    RED_BAR --> BLUE_BAR : SUCCEED
    RED_BAR --> CANCEL : TIMEOUT

    BLUE_BAR --> TUBES : SUCCEED
    BLUE_BAR --> CANCEL : TIMEOUT

    TUBES --> SECOND_WINDOW : SUCCEED
    TUBES --> CANCEL : TIMEOUT

    SECOND_WINDOW --> SUCCEED : SUCCEED
    SECOND_WINDOW --> CANCEL : TIMEOUT

    %% Apply styles to outcomes
    class SUCCEED success
    class CANCEL cancel
```

## Dropping State Machine (Mission 2)
``` mermaid
stateDiagram-v2
    %% Custom styles
    classDef success fill:#2ea043,stroke:#238636,stroke-width:2px,color:#fff
    classDef warning fill:#d29922,stroke:#9e6a03,stroke-width:2px,color:#fff
    classDef cancel fill:#f28c28,stroke:#b25f12,stroke-width:2px,color:#fff

    [*] --> GO_TO_BOX

    GO_TO_BOX --> SUCCEED : SUCCEED
    GO_TO_BOX --> CANCEL : TIMEOUT

    CENTER --> DROP : SUCCEED
    CENTER --> REACQUIRE : FAIL
    CENTER --> CANCEL : TIMEOUT

    REACQUIRE --> CENTER : SUCCEED
    REACQUIRE --> TIMEOUT : TIMEOUT
    REACQUIRE --> CANCEL : CANCEL

    DROP --> SUCCEED : SUCCEED
    DROP --> CANCEL : CANCEL

    %% Apply styles to outcomes
    class SUCCEED success
    class CANCEL cancel
    class TIMEOUT warning
```

## Inspect State Machine (Mission 3)
``` mermaid
stateDiagram-v2
    %% Custom styles
    classDef success fill:#2ea043,stroke:#238636,stroke-width:2px,color:#fff
    classDef cancel fill:#f28c28,stroke:#b25f12,stroke-width:2px,color:#fff

    [*] --> GO_TO_WINDOW

    GO_TO_WINDOW --> FIND_WINDOW : SUCCEED
    GO_TO_WINDOW --> CANCEL : TIMEOUT

    FIND_WINDOW --> WINDOW : SUCCEED
    FIND_WINDOW --> CANCEL : TIMEOUT

    WINDOW --> COUNT_BABIES : SUCCEED
    WINDOW --> CANCEL : TIMEOUT

    COUNT_BABIES --> GO_OUT : SUCCEED

    GO_OUT --> SUCCEED : SUCCEED
    GO_OUT --> CANCEL : TIMEOUT

    %% Apply styles to outcomes
    class SUCCEED success
    class CANCEL cancel
```

## Precise Landing
```mermaid
stateDiagram-v2
    %% Custom styles
    classDef success fill:#2ea043,stroke:#238636,stroke-width:2px,color:#fff
    classDef danger fill:#da3633,stroke:#b62324,stroke-width:2px,color:#fff

    [*] --> GO_TO_LAND

    GO_TO_LAND --> CENTER : SUCCEED
    GO_TO_LAND --> ABORT : TIMEOUT

    CENTER --> SUCCEED : SUCCEED
    CENTER --> REACQUIRE : FAIL
    CENTER --> ABORT : TIMEOUT

    REACQUIRE --> CENTER : SUCCEED
    REACQUIRE --> ABORT : CANCEL

    %% Apply styles to outcomes
    class SUCCEED success
    class ABORT danger
```