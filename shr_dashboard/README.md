# shr_dashboard

Web dashboard package for SHR (robot-hosted, LAN-accessible).

## Current scope (v1)
- Blank Home page shell
- Protocol Designer page for editing `GenericProtocol` entries in the active house YAML
- Schema validation using `smart_home_pytree.protocol_schema`
- Atomic save with timestamped backup (`*.bak.YYYYMMDD_HHMMSS`)

## Run (development)
Ensure the robot environment has:
- `house_yaml_path` set to the active YAML file
- Python deps installed (`Flask`, `PyYAML`)
- `smart_home_pytree` available in the Python environment

Example:
```bash
export house_yaml_path=/path/to/smart_home_pytree/config/house_info.yaml
ros2 run shr_dashboard dashboard_server
```

Default URL:
- `http://<robot-ip>:5080`
