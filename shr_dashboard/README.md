# shr_dashboard

Web dashboard for the Smart Home Robot.

Current scope:
- live robot-state view
- protocol history view
- GenericProtocol editor
- YAML validate/save flow with runtime reload handshake

## Required Environment

The dashboard expects a configured user profile:

```bash
export SHR_USER_DIR="$HOME/shr_user/<user_name>"
```

It uses:
- `"$SHR_USER_DIR/configs/house_config.yaml"` as the active config
- `"$SHR_USER_DIR/audios/"` to populate `play_audio` dropdowns
- `"$SHR_USER_DIR/videos/"` to populate `play_video` dropdowns

If `SHR_USER_DIR` is not set, the dashboard will report a setup error instead of guessing a repo-local config.

## Run

```bash
ros2 run shr_dashboard dashboard_server
```

Default URL:
- `http://<robot-ip>:5080`

## Save Behavior

When the dashboard saves YAML:
1. it validates the config
2. it writes `house_config.yaml`
3. it creates a timestamped backup in `configs/`
4. it requests a runtime reload
5. it shows pending/success/error notification state in the UI

Runtime reload is applied when protocols are quiescent:
- `idle`
- `cooldown`

## Notes

- `play_audio` and `play_video` fields in the editor are driven from files currently present in the active user's media folders.
- Closing a notification does not clear the saved runtime reload state; it only dismisses the banner from view.
