# smart_home_pytree test

This directory contains the maintained test and test-support files for `smart_home_pytree`.

## Layout

```text
test/
├── integration/   # multi-component runtime tests
├── mock/          # mock ROS nodes and action servers used by tests
├── unit/          # focused unit tests for schema, builders, and trees
├── gui_for_testing.py
├── test_copyright.py
├── test_flake8.py
└── test_pep257.py
```

## Maintained Test Categories

### `unit/`
Use these for fast validation while developing.

Current maintained unit coverage includes:
- GenericProtocol schema validation
- execution location selection
- GenericProtocol tree building
- person initialization from house YAML
- move-to and charge tree behavior
- guarded move-to-person behavior

### `integration/`
Use these when changing orchestration or multi-component runtime behavior.

Current maintained integration coverage includes:
- protocol orchestrator behavior

### `mock/`
Reusable helpers for tests and manual local bringup.

Most commonly used helper:
- `mock/mock_action_server.py`

## Recommended Test Commands

Run the focused maintained subset used most often during development:

```bash
pytest -q \
  smart_home_pytree/test/unit/test_execution_location_schema.py \
  smart_home_pytree/test/unit/test_person_init.py \
  smart_home_pytree/test/unit/test_generic_protocol_tree.py \
  smart_home_pytree/test/unit/test_execution_location_selector.py \
  smart_home_pytree/test/unit/test_media_execution_location.py
```

Run a broader maintained unit pass:

```bash
pytest -q smart_home_pytree/test/unit
```

Run orchestrator integration coverage:

```bash
pytest -q smart_home_pytree/test/integration/test_protocol_orchestrator.py
```

## Notes

- Avoid running live robot topics while unit tests are executing.
- Prefer focused pytest commands over running the whole repository blindly.
- If you add a new GenericProtocol step type, add at least one schema test and one builder/runtime-oriented unit test.
- If you add a new trigger event key, add tests for both robot-state ingestion and trigger satisfaction.
