"""setup.py — ament_python packaging for follow_me_nodes (entry points, data files)."""

import os
from glob import glob

from setuptools import find_packages, setup

package_name = "follow_me_nodes"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    # Non-Python assets must be listed here or they never reach share/, and
    # get_package_share_directory() lookups at launch time fail.
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.urdf")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jack",
    maintainer_email="jacksteketee1@gmail.com",
    description="Python nodes for the follow-me RC car (serial bridge, fusion, nav).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "serial_bridge = follow_me_nodes.serial_bridge:main",
            "pose_estimator = follow_me_nodes.pose_estimator:main",
            "tag_broadcaster = follow_me_nodes.tag_broadcaster:main",
            "tag_estimator = follow_me_nodes.tag_estimator:main",
            "nav_controller = follow_me_nodes.nav_controller:main",
        ],
    },
)
