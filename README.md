# smart_home_robot

## Setup

``` 
mkdir -p smart_home_pytree_ws/src
cd ~/smart_home_pytree_ws/src
sudo apt-get install mpg321
pip install -r requirements.txt

git clone https://github.com/splintered-reality/py_trees
git clone https://github.com/splintered-reality/py_trees_ros/
git clone https://github.com/splintered-reality/py_trees_ros_viewer
git clone https://github.com/splintered-reality/py_trees_ros_interfaces
git clone https://github.com/AssistiveRoboticsUNH/smart_home_robot

pip install -r requirements.txt

cd ~/smarthome_ws 
colcon build --symlink-install
source install/setup.bash

the yaml file to be used that contains information about the house should be stored as an env variable under 
** house_yaml_path**

If you plan on using the smart plug you need to defiin ethe plug_ip  as an environment variable
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

## Instructions to run planner. Will make it nicer later
TO TEST AT OLSON

run the following to have the action servers needed 
ros2 launch stretch_nav2 navigation_mppi_dual_hesai.launch.py map:=${HELLO_FLEET_PATH}/maps/map_olson.yaml

cd ~/smarthome_ws/src/smart_home_robot/smart_home_pytree/test/mock and run python3 mock_run_actions_no_nav.py 

to log to discord run
ros2 run simple_logger simple_logger_discord 

currently you need to publish person_location and charging topics. you can use the gui for that 
cd ~/smarthome_ws/src/smart_home_robot/smart_home_pytree/test and run python3 gui_for_testing.py 


to run specific protocol
python3 two_reminder_protocol.py --protocol_name medicine_am

or python3 two_reminder_protocol.py --protocol_name medicine_pm


to run the orchestrator:

 python3 protocol_orchestrator.py this takes test_time variable. if none it will use actual time.
 look in the yaml file to figure out the requirements for each protocol

 for coffee in addtion to time being between 9:30 and 1 it requires two topics to be true

ros2 topic pub /coffee std_msgs/msg/Bool "data: True"
ros2 topic pub /coffee_pot std_msgs/msg/Bool "data: True"

will support later to change the time dynamically you can also pub a string with to the /sim_time topic

helper commands:
to find free ports
 for port in {5556..5600}; do   if ! ss -tuln | grep -q ":$port "; then     echo "Port $port is free";   fi; done

### Set up Webapp
Install the application in https://github.com/AssistiveRoboticsUNH/Hello_Face on your tablet

1. Connect the tablet to your laptop and copy the Hello_Face.apk in the apk folder to the Download folder on your tablet.
2. go to the Download folder on your tablet and click on the apk file to install it.
3. Set your Robot IP & Help Number (Swipe from left → Settings → enter values → Save.)
