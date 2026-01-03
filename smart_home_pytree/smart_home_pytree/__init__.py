# Imports
# from . import move_to_tree
# from . import subscription_tree

# from smart_home_pytree.trees import charge_robot_tree
# TREE_REGISTRY = {
#     "charge_robot_tree": charge_robot_tree.run_tree,
#     "move_to_tree": move_to_tree.run_tree,
# }
# INFINITE_TREE_REGISTRY = {
#     "charge_robot_tree": charge_robot_tree.run_tree_infinite,
#     "move_to_tree": move_to_tree.run_tree_infinite,
# }
from smart_home_pytree.behaviors import (
    check_robot_state_key,
    get_person_location,
    logging_behavior,
    move_to_behavior,
    robot_person_same_location,
)
from smart_home_pytree.trees import base_tree_runner, move_to_tree

from . import registry, robot_interface
