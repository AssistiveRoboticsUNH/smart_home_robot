### simple_logger

ROS log subscriber and Discord/file logger support.

What it does:
- subscribes to `/rosout`
- filters for the runtime messages you care about
- stores log output locally
- can forward important events to Discord

### Log location

Logs are written under:

```text
$SHR_USER_DIR/logs/discord_logs/
```

`SHR_USER_DIR` must be set before starting the logger.

### Run

```bash
ros2 run simple_logger simple_logger_discord
```
