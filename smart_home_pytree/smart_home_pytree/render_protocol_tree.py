#!/usr/bin/env python3

"""Render a protocol behavior tree image from house YAML config."""

import argparse
import os

from py_trees import display

from smart_home_pytree.protocols.loader import load_house_config_yaml
from smart_home_pytree.protocols.builders.generic_protocol import GenericProtocolTree
from smart_home_pytree.protocols.registry import load_locations_to_blackboard, load_protocols_to_bb
from smart_home_pytree.utils import get_house_yaml_path


def _resolve_house_yaml_path() -> str:
    """Resolve house YAML path from environment variables."""
    yaml_path = get_house_yaml_path()
    if not yaml_path:
        raise EnvironmentError(
            "Missing house config path. Set SHR_USER_DIR so configs/house_config.yaml can be resolved."
        )
    if not os.path.isfile(yaml_path):
        raise FileNotFoundError(f"house YAML path not found: {yaml_path}")
    return yaml_path


def main(args=None):
    parser = argparse.ArgumentParser(
        description="Render a protocol behavior tree from house YAML into DOT/PNG/SVG files.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--protocol_name",
        type=str,
        required=True,
        help="Protocol name from house YAML (e.g., medicine_am).",
    )
    parser.add_argument(
        "--output_dir",
        type=str,
        default=".",
        help="Directory where rendered files are written.",
    )
    parser.add_argument(
        "--output_name",
        type=str,
        default="",
        help="Base file name without extension. Defaults to '<protocol_name>_tree'.",
    )
    parser.add_argument(
        "--with_blackboard_variables",
        action="store_true",
        help="Include blackboard variables in graph labels.",
    )

    parsed, _ = parser.parse_known_args(args=args)

    yaml_path = _resolve_house_yaml_path()
    house_cfg = load_house_config_yaml(yaml_path)

    protocols = house_cfg.get("protocols", {})
    protocol_cfg = protocols.get(parsed.protocol_name)
    if protocol_cfg is None:
        raise KeyError(
            f"Protocol '{parsed.protocol_name}' not found. Available: {sorted(protocols.keys())}"
        )

    runner = protocol_cfg.get("runner")
    if runner != "GenericProtocol":
        raise ValueError(
            f"Protocol '{parsed.protocol_name}' uses runner '{runner}'. "
            "Only GenericProtocol rendering is supported by this script."
        )

    output_dir = os.path.abspath(parsed.output_dir)
    os.makedirs(output_dir, exist_ok=True)
    output_name = parsed.output_name.strip() or f"{parsed.protocol_name}_tree"

    load_locations_to_blackboard(yaml_path)
    load_protocols_to_bb(yaml_path)

    tree_runner = GenericProtocolTree(
        node_name=f"render_{parsed.protocol_name}",
        protocol_name=parsed.protocol_name,
        debug=False,
    )

    try:
        root = tree_runner.create_tree()
        outputs = display.render_dot_tree(
            root=root,
            name=output_name,
            target_directory=output_dir,
            with_blackboard_variables=parsed.with_blackboard_variables,
        )
    finally:
        tree_runner.cleanup()

    print("Rendered files:")
    for _, path in outputs.items():
        print(path)


if __name__ == "__main__":
    main()
