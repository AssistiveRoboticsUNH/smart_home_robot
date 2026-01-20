import os
from glob import glob

from setuptools import find_packages, setup

package_name = "smart_home_pytree"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(
        include=[
            "smart_home_pytree",
            "smart_home_pytree.*",
            "robot_actions",
            "robot_actions.*",
            "smart_sensors",
            "smart_sensors.*",
        ],
        exclude=["test", "test.*"],
    ),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="olagh",
    maintainer_email="olaghattas@hotmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "undocking = robot_actions.undocking:main",
            "docking = robot_actions.docking:main",
            "play_video = robot_actions.play_video:main",
            "smart_plug = smart_sensors.smart_plug_node:main",
            "charge_monitor = smart_sensors.charge_monitor:main",
            "mock_dock_undock = robot_actions.mock_dock_undock:main",
            "protocol_orchestrator = smart_home_pytree.protocol_orchestrator:main",
        ],
    },
)
