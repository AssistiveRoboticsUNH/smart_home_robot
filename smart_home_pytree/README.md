# smart_home_pytree

`smart_home_pytree` is the behavior-tree runtime for the Smart Home Robot system.
It owns protocol execution, trigger evaluation, robot state integration, blackboard-backed configuration loading, and the reusable behavior-tree building blocks used by the orchestrator.

This package is the core runtime package. A new developer should read this README first before touching protocol logic, tree composition, or orchestration.

## What This Package Does

At a high level, the package does four jobs:

1. Load house configuration and protocol definitions from YAML.
2. Build reusable py_trees subtrees and protocol trees.
3. Monitor robot state and triggers to decide what protocol should run.
4. Run protocol trees through the orchestrator and persist protocol history/state.

## Mental Model

The package is easiest to understand with this dependency flow:

```text
house_info.yaml
    -> protocols.schema / protocols.loader / protocols.registry
    -> py_trees blackboard
    -> trees (subtrees for unit actions)
    -> protocols.builders (full protocol trees)
    -> protocol_orchestrator
    -> protocol_tracker / dashboard history
```

And inside the BT code itself:

```text
behaviors/  -> leaf nodes
trees/      -> reusable subtrees for one unit action
protocols/  -> full protocol tree builders and config loading
triggers/   -> high-level trigger evaluation and scheduling
```

## Current Package Structure

```text
smart_home_pytree/
├── config/                 # house/protocol YAML files and Nav2 config
├── launch/                 # real robot / sim / action-server bringup
├── robot_actions/          # ROS action server executables used by trees
├── smart_sensors/          # sensor/bridge ROS nodes used by runtime
├── test/                   # unit, integration, mock helpers
└── smart_home_pytree/
    ├── behaviors/          # py_trees leaf nodes only
    ├── trees/              # reusable subtrees for one unit action
    ├── protocols/
    │   ├── builders/       # full protocol tree builders
    │   ├── loader.py       # load validated house YAML
    │   ├── normalizer.py   # config normalization before validation/use
    │   ├── registry.py     # publish locations/protocols into blackboard
    │   ├── schema.py       # house/protocol schema validation
    │   └── models.py       # typed config models for protocol concepts
    ├── triggers/
    │   ├── engine.py       # TriggerMonitor runtime entry point
    │   ├── evaluator.py    # pure trigger evaluation helpers
    │   ├── scheduler.py    # time/reset helpers
    │   ├── persistence.py  # trigger state restore/sync helpers
    │   └── models.py       # typed trigger state models
    ├── protocol_orchestrator.py
    ├── protocol_tracker.py
    ├── robot_interface.py
    ├── render_protocol_tree.py
    └── utils.py
```

## Key Runtime Modules

### `robot_interface.py`
ROS-facing runtime state adapter.

Responsibilities:
- subscribe to robot/environment topics
- maintain the in-memory robot state model
- expose robot state to trees and orchestrator
- resolve YAML-driven initialization such as `person_init`

This is the main entry point when tree code needs live robot state.

### `protocols/schema.py`
Validation layer for house YAML.

Responsibilities:
- validate top-level house config
- validate locations
- validate protocol structure
- validate GenericProtocol step definitions
- validate confirmation branches and trigger rules

Canonical import:
```python
from smart_home_pytree.protocols.schema import validate_house_config, RUN_TREE_SPECS
```

### `protocols/loader.py`
Validated YAML loading entry point.

Responsibilities:
- read YAML
- normalize config
- validate config
- return safe runtime data structure

Canonical import:
```python
from smart_home_pytree.protocols.loader import load_house_config_yaml
```

### `protocols/registry.py`
Blackboard population layer.

Responsibilities:
- load locations onto the py_trees blackboard
- load protocol data onto the blackboard
- synthesize helper keys used by GenericProtocol trees
- update protocol config values already loaded on the blackboard

Canonical import:
```python
from smart_home_pytree.protocols.registry import (
    load_locations_to_blackboard,
    load_protocols_to_bb,
)
```

### `protocols/builders/generic_protocol.py`
Dynamic protocol builder for `GenericProtocol`.

Responsibilities:
- read validated protocol step definitions from blackboard
- build final protocol trees from action steps
- handle confirmation branching and wait/yield flow
- compose reusable subtrees from `trees/`

This is the main builder most developers will touch.

### `triggers/engine.py`
High-level trigger runtime.

Responsibilities:
- evaluate time/event/location trigger conditions
- monitor waiting/yielded protocols
- handle reset patterns and cooldown behavior
- expose currently satisfied protocols to the orchestrator
- restore and sync persisted trigger-related state

Canonical import:
```python
from smart_home_pytree.triggers.engine import TriggerMonitor
```

### `protocol_orchestrator.py`
Top-level runtime coordinator.

Responsibilities:
- initialize robot interface, trigger monitor, and human interface
- decide what protocol to run next
- start, stop, and preempt protocol trees
- persist run results and session-level outcomes

If the system is "running protocols automatically", this is the entry point doing that work.

### `protocol_tracker.py`
SQLite-backed protocol state and run-history storage.

Responsibilities:
- store live protocol state
- store run history and session outcomes
- support dashboard/history queries
- restore state needed after restart

## BT Layers

### `behaviors/`
Leaf py_trees nodes only.

These should:
- do one thing
- return a py_trees status
- avoid owning larger composition logic

Examples:
- move action clients
- blackboard checks
- state checks
- media/question action leaves

### `trees/`
Reusable subtrees for one unit action.

These should:
- compose a few leaf behaviors into one meaningful action block
- be reusable by multiple protocols
- not decide full protocol behavior

Examples:
- move to location
- move to person
- ask question
- read text
- play audio/video
- charge robot

### `protocols/builders/`
Full protocol tree builders.

These should:
- build complete protocol-level trees
- use subtrees from `trees/`
- interpret protocol definitions, not raw ROS topics

## Configuration Model

The active configuration is usually provided through:
- `house_yaml_path`
- or `HOUSE_YAML_PATH`

The house YAML is expected to contain at least:
- `locations`
- `protocols`
- optional `person_init`

### `locations`
Named landmarks used by movement and trigger logic.

### `protocols`
Protocol definitions. The main supported styles today are:
- `GenericProtocol`
- `ExerciseProtocol`
- `ExerciseRandomProtocol`

### `person_init`
Optional startup initialization for `person_location`.
- `null`: do not initialize
- landmark name: initialize `person_location` to that landmark

## Preferred Imports

Use these canonical import paths:

```python
from smart_home_pytree.protocols.schema import ...
from smart_home_pytree.protocols.loader import ...
from smart_home_pytree.protocols.registry import ...
from smart_home_pytree.protocols.builders.generic_protocol import GenericProtocolTree
from smart_home_pytree.triggers.engine import TriggerMonitor
```

## Main Launch / Run Paths

### Real robot support services
```bash
ros2 launch smart_home_pytree real_robot.launch.py
```

This starts the support services used on the real robot bringup side, such as:
- pimu monitor
- display node
- video player action server
- human interaction node
- discord logger

### Simulation support services
```bash
ros2 launch smart_home_pytree sim_robot.launch.py
```

This brings up the simulation-side support stack, including Nav2 sim, display/video helpers, human interaction, sim charging bridge, and logger.

### Protocol orchestrator
```bash
ros2 run smart_home_pytree protocol_orchestrator
```

Useful flags:
```bash
ros2 run smart_home_pytree protocol_orchestrator --debug --test_time 10:30 --env_yaml_file_name house_yaml_path
```

### Render a protocol tree
```bash
export house_yaml_path=/path/to/house_info.yaml
ros2 run smart_home_pytree render_protocol_tree --protocol_name medicine_am
```

## Developer Workflows

### 1. Add a new unit action
Usually touch:
- `behaviors/` for a new leaf if needed
- `trees/` for the reusable subtree
- `protocols/schema.py` if new GenericProtocol params are needed
- `protocols/builders/shared_builder_utils.py` if the action should be available in GenericProtocol

### 2. Add a new GenericProtocol step type
Usually touch:
- `protocols/schema.py` to declare/validate the tree name and params
- `protocols/builders/shared_builder_utils.py` to map the step type to a subtree builder
- a subtree in `trees/` if one does not already exist

### 3. Change trigger behavior
Usually touch:
- `triggers/evaluator.py` for trigger semantics
- `triggers/scheduler.py` for time/reset behavior
- `triggers/engine.py` only if runtime coordination needs to change

### 4. Add a new trigger event key
Use this when you want YAML triggers such as `high_level.triggers.event` to depend on a new robot state field.

Usually touch:
- `robot_interface.py` to subscribe to the source topic and update `RobotState` with the new key
- `triggers/evaluator.py` only if the new key needs special comparison logic beyond the existing `state`/`value`/`op` checks
- `protocols/schema.py` only if the event shape itself changes; a new state key alone does not require schema edits
- one or more protocol YAML entries in `config/*.yaml` to use the new key under `high_level.triggers.event`
- tests covering both robot-state ingestion and trigger satisfaction

Recommended implementation order:
1. Add the new state key to `RobotInterface` update flow.
2. Confirm the value appears in `robot_interface.state`.
3. Add or update a protocol trigger using that key.
4. Add tests in `test/unit/` for trigger evaluation.
5. Run a focused orchestrator or trigger-monitor smoke test if the key is safety-critical.

### 5. Change protocol loading or blackboard shape
Usually touch:
- `protocols/loader.py`
- `protocols/normalizer.py`
- `protocols/registry.py`

## Tests

The package has both unit and integration tests under `test/`.

Run targeted unit tests during development:
```bash
pytest -q smart_home_pytree/test/unit/test_execution_location_schema.py
pytest -q smart_home_pytree/test/unit/test_generic_protocol_tree.py
pytest -q smart_home_pytree/test/unit/test_person_init.py
```

Run a focused subset before merging protocol/runtime changes:
```bash
pytest -q \
  smart_home_pytree/test/unit/test_execution_location_schema.py \
  smart_home_pytree/test/unit/test_person_init.py \
  smart_home_pytree/test/unit/test_generic_protocol_tree.py \
  smart_home_pytree/test/unit/test_execution_location_selector.py \
  smart_home_pytree/test/unit/test_media_execution_location.py
```

## Common Gotchas

- `behaviors/` and `trees/` are not the same layer. Do not move subtree logic into behaviors.
- Generic protocol builders live under `protocols/builders/`.
- Schema, loader, and registry code live under `protocols/`.
- The orchestrator dynamically imports protocol builders from `smart_home_pytree.protocols.builders`.
- Blackboard keys are synthesized by `protocols/registry.py`; if a tree depends on a key, verify the registry is producing it.

## Suggested Reading Order For New Developers

1. This README
2. `config/Readme.md`
3. `smart_home_pytree/protocol_orchestrator.py`
4. `smart_home_pytree/triggers/engine.py`
5. `smart_home_pytree/protocols/schema.py`
6. `smart_home_pytree/protocols/registry.py`
7. `smart_home_pytree/protocols/builders/generic_protocol.py`
8. one subtree in `trees/` and one leaf in `behaviors/`

That sequence gives the fastest path to understanding the package end-to-end.
