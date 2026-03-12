# smart_home_pytree test

Maintained test and test-support files for `smart_home_pytree`.

## Layout

```text
test/
├── integration/
├── mock/
├── unit/
├── gui_for_testing.py
├── test_copyright.py
├── test_flake8.py
└── test_pep257.py
```

## Recommended Workflow

Before running tests that depend on config resolution, set a temporary `SHR_USER_DIR` or override paths explicitly in the test.

Focused maintained subset:

```bash
pytest -q \
  smart_home_pytree/test/unit/test_execution_location_schema.py \
  smart_home_pytree/test/unit/test_person_init.py \
  smart_home_pytree/test/unit/test_generic_protocol_tree.py \
  smart_home_pytree/test/unit/test_execution_location_selector.py \
  smart_home_pytree/test/unit/test_media_execution_location.py
```

Broader unit pass:

```bash
pytest -q smart_home_pytree/test/unit
```
