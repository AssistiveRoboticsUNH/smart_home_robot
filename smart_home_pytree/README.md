# smart_home_pytree

`smart_home_pytree` is the behavior-tree runtime for the Smart Home Robot system.

It owns:
- house-config loading and validation
- blackboard config registration
- reusable subtree construction
- protocol orchestration
- trigger evaluation and cooldown handling
- live protocol state and run-history persistence

## Data Boundary

This package reads its active runtime data from `SHR_USER_DIR`.

Expected layout:

```text
$SHR_USER_DIR/
├── configs/
│   └── house_config.yaml
├── audios/
├── videos/
├── images/
├── logs/
├── database/
└── map/
```

Important defaults:
- active YAML: `"$SHR_USER_DIR/configs/house_config.yaml"`
- protocol tracker DB: `"$SHR_USER_DIR/database/protocol_tracker.db"`
- user audio assets: `"$SHR_USER_DIR/audios/"`
- user video assets: `"$SHR_USER_DIR/videos/"`

## Mental Model

```text
behaviors/  -> py_trees leaf nodes
trees/      -> reusable unit-action subtrees
protocols/  -> config loading + protocol tree builders
triggers/   -> trigger evaluation + cooldown/wait state
```

## Package Structure

```text
smart_home_pytree/
├── config/                 # sample configs and package-shared config assets
├── launch/                 # launch files
├── robot_actions/          # ROS action servers used by trees
├── smart_sensors/          # helper sensor/bridge nodes
├── test/
└── smart_home_pytree/
    ├── behaviors/
    ├── trees/
    ├── protocols/
    │   ├── builders/
    │   ├── loader.py
    │   ├── normalizer.py
    │   ├── registry.py
    │   ├── schema.py
    │   └── models.py
    ├── triggers/
    ├── protocol_orchestrator.py
    ├── protocol_tracker.py
    ├── robot_interface.py
    ├── render_protocol_tree.py
    └── utils.py
```

## Key Modules

### `protocols/loader.py`
Loads house config from the active user profile, normalizes it, validates it, and returns runtime-safe data.

### `protocols/registry.py`
Pushes validated locations and protocol definitions onto the py_trees blackboard.

### `protocols/builders/generic_protocol.py`
Builds `GenericProtocol` trees dynamically from `action.steps`.

### `triggers/engine.py`
Owns trigger evaluation, waiting/yielded state, cooldown handling, and satisfied-protocol selection.

### `protocol_orchestrator.py`
Coordinates trigger monitor, robot interface, and tree execution.

### `protocol_tracker.py`
Stores live protocol state and run history in sqlite.

## Config Rules

Top-level house config lives in `house_config.yaml` and includes:
- `locations`
- `person_init`
- `protocols`

### Locations
Each location must provide:
- `x`
- `y`
- `yaw`

`yaw` is stored in YAML as degrees. It is converted to radians during runtime loading.

### User Media
For `GenericProtocol` media actions, prefer portable asset names:

```yaml
- tree_name: play_audio
  tree_params:
    audio_path: food_reminder.mp3

- tree_name: play_video
  tree_params:
    video_path: maggie_coffee.mp4
```

These resolve against:
- `"$SHR_USER_DIR/audios/"`
- `"$SHR_USER_DIR/videos/"`

### Exercise Assets
Exercise protocol config can use portable values such as:
- `exercise_yaml_file: exercise_routine_test.yaml`
- `video_dir_path: videos`

Runtime resolution looks in:
- `"$SHR_USER_DIR/configs/"` first for relative config files
- package `config/` for bundled shared config files such as exercise routines
- `"$SHR_USER_DIR/videos/"` for video directories

## Run

Start the orchestrator:

```bash
ros2 run smart_home_pytree protocol_orchestrator
```

Render a tree using the active user config:

```bash
python3 smart_home_pytree/smart_home_pytree/render_protocol_tree.py \
  --protocol_name medicine_am \
  --output_dir /tmp
```

## Developer Workflows

### Add a new GenericProtocol action tree
1. Add or reuse a subtree in `trees/`.
2. Register the tree in `protocols/builders/shared_builder_utils.py`.
3. Add schema metadata in `protocols/schema.py`.
4. Add at least one schema test and one builder/runtime-oriented test.

### Add a new trigger event key
1. Add the new robot-state key to `robot_interface.py` if it comes from live robot state.
2. Ensure the trigger evaluator can compare that value shape in `triggers/evaluator.py`.
3. If the dashboard needs dedicated UI treatment, expose metadata from the dashboard config service.
4. Add tests for both robot-state ingestion and trigger satisfaction.

## Testing

Focused maintained test subset:

```bash
pytest -q \
  smart_home_pytree/test/unit/test_execution_location_schema.py \
  smart_home_pytree/test/unit/test_person_init.py \
  smart_home_pytree/test/unit/test_generic_protocol_tree.py \
  smart_home_pytree/test/unit/test_execution_location_selector.py \
  smart_home_pytree/test/unit/test_media_execution_location.py
```
