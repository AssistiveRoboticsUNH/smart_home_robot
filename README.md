# smart_home_robot

## Setup

``` 
mkdir -p smart_home_pytree_ws/src
cd ~/smart_home_pytree_ws/src

pip install -r requirements.txt

git clone https://github.com/splintered-reality/py_trees
git clone https://github.com/splintered-reality/py_trees_ros/
git clone https://github.com/splintered-reality/py_trees_ros_viewer
git clone https://github.com/splintered-reality/py_trees_ros_interfaces
git clone https://github.com/olaghattas/smart_home_pytree

cd ~/smart_home_pytree_ws 
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
cp .env.example .env 
```

3. **Fill in the required variables**
Open .env and replace all placeholder values with your own configuration
(API keys, tokens, paths, etc.).

4. **Load the environment variables**
Before running the project, load your .env file into your shell:
```
source .env
```

5. **Do not commit `.env`**
The `.env` file contains sensitive information and must stay private.
It is already included in `.gitignore` to prevent accidental commits.
