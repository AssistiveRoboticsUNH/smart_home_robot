# smart_home_robot

ROS 2 workspace for the Smart Home Robot system.

This repository contains the runtime packages, dashboard, human-interaction node, message definitions, and logging utilities. User-specific deployment data does not belong in git and is loaded from `SHR_USER_DIR`.

## Workspace

Expected workspace path:

```bash
~/smarthome_ws/src/smart_home_robot
```

## User Data Layout

Each deployed user gets a private data root:

```text
$SHR_USER_DIR/
├── configs/
│   ├── house_config.yaml
│   └── house_config.yaml.bak.*
├── audios/
├── videos/
├── images/
├── logs/
├── database/
└── map/
```

Use:
- `configs/house_config.yaml` for the active house/protocol YAML
- `audios/` for `play_audio` assets
- `videos/` for `play_video` assets
- `logs/` for runtime log output
- `database/` for sqlite files such as `protocol_tracker.db`

## Install

```bash
sudo apt-get install -y mpg321

mkdir -p ~/smarthome_ws/src
cd ~/smarthome_ws/src
git clone https://github.com/AssistiveRoboticsUNH/smart_home_robot
cd smart_home_robot
pip install -r requirements.txt
cd ~/smarthome_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## `.env` Setup

`.env` is part of the required local setup. 
Use it for secrets and API credentials only.

Create it from the template:

```bash
cd ~/smarthome_ws/src/smart_home_robot
cp env.example .env
```

Edit `.env` with your API keys, tokens, and passwords.

## Create a User Profile

```bash
cd ~/smarthome_ws/src/smart_home_robot
./setup_user.sh
```

The script will:
- create `~/shr_user/<user_name>/`
- create `configs/`, `audios/`, `videos/`, `images/`, `logs/`, `database/`, and `map/`
- copy `smart_home_pytree/config/house_info.yaml` into `configs/house_config.yaml` if it does not already exist
- print the exact `SHR_USER_DIR` export line for `~/.bashrc`
- print what to source for `.env` in `~/.bashrc`

Recommended `~/.bashrc` lines:

```bash
export SHR_USER_DIR="$HOME/shr_user/akash"
source $HOME/smarthome_ws/src/smart_home_robot/.env
```

Reload your shell:

```bash
source ~/.bashrc
```

Verify:

```bash
echo "$SHR_USER_DIR"
ls "$SHR_USER_DIR/configs/house_config.yaml"
```

## Common Commands

Start the dashboard:
```bash
# it initializes the robot state interface by default
ros2 run shr_dashboard dashboard_server
```

Start the orchestrator:

```bash
# if dashboard is not started, orchestrator will initialize robot state interface
ros2 run smart_home_pytree protocol_orchestrator [--test_time "15:30"] [--debug]
```

Run a single GenericProtocol directly:

```bash
# run necessary navigation and robot launch files and then
python3 smart_home_pytree/smart_home_pytree/protocols/generic_protocol.py --protocol_name medicine_am
```


Render a GenericProtocol tree png image:

```bash
python3 smart_home_pytree/smart_home_pytree/render_protocol_tree.py --protocol_name medicine_am [--output_dir /tmp]
```


## Simulation Setup

Jazzy is the default ROS 2 distro for this robot. Install Jazzy simulation dependencies first:

```bash
sudo apt install ros-$ROS_DISTRO-navigation2
sudo apt install ros-$ROS_DISTRO-nav2-bringup
sudo apt install ros-$ROS_DISTRO-nav2-minimal-tb3*
```

For Iron and older:

```bash
sudo apt install ros-$ROS_DISTRO-turtlebot3-gazebo
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/$ROS_DISTRO/share/turtlebot3_gazebo/models
```

Run simulation:

```bash
ros2 launch smart_home_pytree sim_robot.launch.py
```

Then run the orchestrator in another terminal:

```bash
ros2 run smart_home_pytree protocol_orchestrator --test_time "15:30" --debug
```

## Set Up Display Tablet App

Install the application from:

```text
https://github.com/AssistiveRoboticsUNH/Hello_Face
```

Steps:
- Connect the tablet to your laptop and copy `Hello_Face.apk` from the apk folder to the tablet `Download` folder.
- Open the `Download` folder on the tablet and tap the apk file to install it.
- Set the Robot IP and Help Number:
  Swipe from left -> Settings -> enter values -> Save.

## Packages

- `smart_home_pytree`: protocol runtime, tree builders, triggers, persistence
- `shr_dashboard`: operator dashboard and protocol designer
- `shr_human_interaction`: speech and yes/no question interface
- `simple_logger`: Discord and file logging helpers
- `shr_msgs`: shared ROS interfaces

## Rule

Do not keep deployed user data in git.

That includes:
- active `house_config.yaml`
- user audio/video files
- backups
- sqlite databases
- logs
- `.env`

## Linting

```bash
autopep8 --in-place --recursive --aggressive smart_home_pytree/
```
