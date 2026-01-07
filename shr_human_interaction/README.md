# SHR Human Interaction

# Human Interaction Node

ROS 2 node for speech-based human–robot interaction. It wires the `aalap` dialogue manager into ROS topics, exposes a text-to-speech subscriber, and provides an action server for asking yes/no questions.

## Features
- Publishes voice status, user transcripts, and robot speech as `std_msgs/String`.
- Accepts text-to-speech requests on a topic (always speaks, regardless of current state).
- Yes/no question action server (`/ask_question`) with configurable retry count.
- Programmatic wakeword triggering handled inside the question flow.
- External control topics to trigger a session or force-stop an active session (volatile QoS).

## Node
Entry point: `shr_human_interaction.human_interaction_node:main` (console script `human_interaction_node`).

## Topics
- `/voice/status` (`std_msgs/String`, publisher): current dialog state (IDLE, LISTENING, RECORDING, TRANSCRIBING, THINKING, SPEAKING, …).
- `/voice/user` (`std_msgs/String`, publisher): all transcribed user utterances.
- `/voice/robot` (`std_msgs/String`, publisher): all spoken robot utterances (including those initiated via `/voice/speak`).
- `/voice/speak` (`std_msgs/String`, subscriber): publish any text here to have it spoken immediately and mirrored on `/voice/robot`.
- `/voice/system_trigger` (`std_msgs/Empty`, subscriber): when idle, triggers a wakeword session programmatically (QoS: reliable/volatile, depth 1).
- `/voice/stop_session` (`std_msgs/Empty`, subscriber): when active, force-stops the current session and TTS playback (QoS: reliable/volatile, depth 1).

## Action Server
- Name: `/ask_question`
- Type: `shr_msgs/action/QuestionRequest`
  - Goal: `string question`
  - Result: `string response` (normalized yes/no or empty if nothing heard)
  - Feedback: `bool running`
- Behavior:
  - Speaks the question with an appended “Please answer yes or no.”
  - Triggers wakeword, listens, and normalizes replies to yes/no/other.
  - Retries up to `ask_attempts` times (parameter below) before returning.

### Example action client invocation (CLI)
```bash
ros2 action send_goal /ask_question shr_msgs/action/QuestionRequest "{question: 'Do you want coffee?'}"
```

## Parameters
All parameters can be set via a launch file or CLI:
- `model` (string, default `base.en`): ASR model name/path for `aalap`.
- `device` (string, default `auto`): compute device for ASR.
- `wakeword_keywords` (string, default `hey_jarvis`): wakeword(s).
- `wakeword_model_paths` (list of strings, default `[]`): custom wakeword model paths; empty disables override.
- `vad_silero_threshold` (double, default `0.5`): VAD sensitivity.
- `no_speech_timeout_ms` (int, default `3000`): inactivity timeout passed to `aalap` dialog manager.
- `no_speech_timeout_ms_action_server` (int, default `8000`): override inactivity timeout while `/ask_question` is active.
- `publish_hz` (double, default `20.0`): timer frequency for draining/publishing queues.
- `ask_attempts` (int, default `3`): max retries for the `/ask_question` action.

## Usage
Build and source your workspace, then run the node:
```bash
colcon build --packages-select shr_human_interaction
source install/setup.bash
ros2 run shr_human_interaction human_interaction_node
```

Send speech from the CLI:
```bash
ros2 topic pub /voice/speak std_msgs/msg/String "data: 'Hello there'" --once
```

Monitor topics:
```bash
ros2 topic echo /voice/status
ros2 topic echo /voice/user
ros2 topic echo /voice/robot
```

Control sessions from CLI:
```bash
# Trigger listening when idle
ros2 topic pub /voice/system_trigger std_msgs/msg/Empty "{}" --once

# Stop an active session and TTS
ros2 topic pub /voice/stop_session std_msgs/msg/Empty "{}" --once
```

## Dependencies
- `aalap` (dialogue manager library; provides wakeword, VAD, ASR, TTS).
- `shr_msgs` for `QuestionRequest.action`.
- ROS 2 with `rclpy` and `std_msgs`.

### Installing `aalap`
Install the Python package directly from the repo:
```bash
pip install git+https://github.com/MnAkash/aalap.git
```

## Notes
- The external policy hook is currently a no-op; only explicit calls to `speak()` or the action server produce robot speech.
- `speak()` publishes to `/voice/robot` after TTS, so downstream consumers can log or display outgoing speech.
