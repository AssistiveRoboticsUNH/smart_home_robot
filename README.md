# smart_home_robot

## Setup

``` 
mkdir -p smart_home_pytree_ws/src
cd ~/smart_home_pytree_ws/src
sudo apt-get install mpg321
pip install -r requirements.txt

cd ~/smart_home_pytree_ws
rosdep install --from-paths src --ignore-src -r -y

if you run rosdep then no need for this

---
git clone https://github.com/splintered-reality/py_trees
git clone https://github.com/splintered-reality/py_trees_ros/
git clone https://github.com/splintered-reality/py_trees_ros_viewer
git clone https://github.com/splintered-reality/py_trees_ros_interfaces
---

git clone https://github.com/AssistiveRoboticsUNH/smart_home_robot

pip install -r requirements.txt

cd ~/smarthome_ws 
colcon build --symlink-install
source install/setup.bash

the yaml file to be used that contains information about the house should be stored as an env variable under 
** house_yaml_path**

If you plan on using the smart plug you need to define the plug_ip  as an environment variable
```

For simulation you will need 


```
sudo apt install ros-$ROS_DISTRO-navigation2
sudo apt install ros-$ROS_DISTRO-nav2-bringup

Jazzy and newer
sudo apt install ros-$ROS_DISTRO-nav2-minimal-tb*

iron and older
sudo apt install ros-$ROS_DISTRO-turtlebot3-gazebo

source /opt/ros/<ros2-distro>/setup.
export TURTLEBOT3_MODEL=waffle  # Iron and older only with Gazebo Classic
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/<ros2-distro>/share/turtlebot3_gazebo/models # Iron and older only with Gazebo Classic

```
### Environment Setup Instructions (Optional)

1. **Use the template file**

   This project provides a template for required environment variables: .env.example

2. **Create your personal `.env` file**

Copy the template into an actual `.env` file that will hold your values:

```bash
cp env.example .env 
```

3. **Fill in the required variables**
Open .env and replace all placeholder values with your own configuration
(API keys, tokens, paths, etc.).

4. **Load the environment variables**
Before running the project, load your .env file into your shell:
```
source .env
```
or add it to your .bashrc : ```source /home/hello-robot/smarthome_ws/src/smart_home_robot/.env```
to check that things are setup:
echo $house_yaml_path you should get your path

5. **Do not commit `.env`**
The `.env` file contains sensitive information and must stay private.
It is already included in `.gitignore` to prevent accidental commits.



## Running in Simulation
note you will need the turtlebot package installed

Run the launch script
```
ros2 launch smart_home_pytree sim_robot.launch.py
```

then in another terminal run the orchestrator

Prerequisites: Before running, you must set an environment variable pointing to your configuration file. By default, the script looks for house_yaml_path.
```bash
export house_yaml_path="/absolute/path/to/your/config.yaml"
```
Basic Execution:
```
python3 protocol_orchestrator.py 
```

Testing Specific Times: Run with time override for 2:30 PM and debug mode
```
python3 protocol_orchestrator.py --test_time "14:30" --debug
```

| Argument | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `--test_time` | `str` | `""` | Optional time override in `HH:MM` format. If provided, the system ignores the wall clock and uses this static time. |
| `--debug` | `flag` | `False` | Enable detailed debug output to the console. |
| `--env_yaml_file_name` | `str` | `"house_yaml_path"` | The **name** of the environment variable to read the YAML path from. Change this if you use a different env var name. |


You can also run with ros

```
ros2 run smart_home_pytree protocol_orchestrator -- --debug --test_time 10:30 --env_yaml_file_name house_yaml_path
```

## Instructions to run planner. Will make it nicer later
TO TEST AT OLSON

run the following to have the action servers needed 
ros2 launch stretch_nav2 navigation_mppi_dual_hesai.launch.py map:=${HELLO_FLEET_PATH}/maps/map_olson.yaml

cd ~/smarthome_ws/src/smart_home_robot/smart_home_pytree/test/mock and run python3 mock_run_actions_no_nav.py 

to log to discord run

```
ros2 run simple_logger simple_logger_discord 
```

currently you need to publish person_location and charging topics. you can use the gui for that 
cd ~/smarthome_ws/src/smart_home_robot/smart_home_pytree/test and run python3 gui_for_testing.py 


to run a specific protocol
```
python3 two_reminder_protocol.py --protocol_name medicine_am
```
look in the yaml file to figure out the requirements for each protocol

for coffee in addtion to time being between 9:30 and 1 it requires two topics to be true

```
ros2 topic pub /coffee std_msgs/msg/Bool "data: True"
ros2 topic pub /coffee_pot std_msgs/msg/Bool "data: True"
```
will support later to change the time dynamically you can also pub a string with to the /sim_time topic

helper commands:
to find free ports:
```
 for port in {5556..5600}; do   if ! ss -tuln | grep -q ":$port "; then     echo "Port $port is free";   fi; done
```

run video action:
```
ros2 action send_goal /play_video shr_msgs/action/PlayVideoRequest "file_name: 'file:///storage/emulated/0/Download/maggie_coffee.mp4'"
```
To trigger start_exercise its should be either for the display or ```ros2 topic pub /display_rx std_msgs/msg/String "data: 'exercise_requested'" ```

### Set up Webapp
Install the application in https://github.com/AssistiveRoboticsUNH/Hello_Face on your tablet

1. Connect the tablet to your laptop and copy the Hello_Face.apk in the apk folder to the Download folder on your tablet.
2. go to the Download folder on your tablet and click on the apk file to install it.
3. Set your Robot IP & Help Number (Swipe from left → Settings → enter values → Save.)

### Linting command
autopep8 --in-place --recursive --aggressive smart_home_pytree/
