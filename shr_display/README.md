# shr_display

Bridge between ROS topics and the Android display app using `ZMQ`.

## Video Delivery Contract

Robot-side video playback commands are now sent as:

```text
play_video:<relative_path>
```

Examples:

```text
play_video:maggie_coffee.mp4
play_video:exercise/arms/demo1.mp4
```

The relative path is resolved from the robot user profile root:

```text
$SHR_USER_DIR/videos/<relative_path>
```

`stop_video` remains unchanged.

## Robot Media HTTP Service

`display_node` starts a read-only HTTP service for tablet downloads.

Default port:

```text
8008
```

Endpoints:

```text
GET /media/videos/<relative_path>
GET /media/manifest/videos
```

The video endpoint serves files from `"$SHR_USER_DIR/videos"` and blocks path traversal.

The manifest endpoint returns JSON with:
- `relative_path`
- `file_name`
- `size_bytes`
- `modified_time`

## Android App Update Prompt

Use this prompt in the Android app codebase:

```text
Update the Android display app to support robot-hosted video assets using the existing ZMQ string command style.

Current behavior:
- The app receives ZMQ messages.
- A video play command is currently treated as a tablet-local absolute path.

New required behavior:
1. Keep support for the command format:
   play_video:<relative_path>
   Example:
   - play_video:maggie_coffee.mp4
   - play_video:exercise/arms/demo1.mp4

2. Treat the path after `play_video:` as a relative path under the robot video library.

3. Add an app setting for the robot video base URL, for example:
   http://<robot-ip>:<port>/media/videos/

4. Add a local tablet cache root for robot videos.
   Store downloaded files under the same relative folder structure.
   Example:
   - command: play_video:exercise/arms/demo1.mp4
   - local cache path: <app-cache-root>/exercise/arms/demo1.mp4

5. Playback flow:
   - parse the command
   - extract the relative path
   - if the file exists locally, play it immediately
   - if it does not exist locally, download it from:
     {robot_video_base_url}/{relative_path}
   - create parent directories as needed
   - save the file locally
   - then play the local file

6. Keep cached videos after download. Do not delete local copies automatically in this pass.

7. Maintain backward compatibility if practical:
   - if a message does not start with `play_video:`, keep existing behavior unchanged
   - if old absolute local-tablet path messages are still used during migration, continue supporting them temporarily

8. Keep `video_finished` response behavior unchanged so the robot still completes the action as it does today.

9. Handle failures clearly:
   - malformed command
   - empty relative path
   - HTTP 404 or network failure
   - partial/corrupt download
   - playback failure

10. If the app already has a response channel, optionally emit status strings such as:
   - video_download_started
   - video_download_complete
   - video_download_failed
   - video_finished

11. Keep implementation modular:
   - command parser
   - cache/path manager
   - HTTP downloader
   - playback controller
```
