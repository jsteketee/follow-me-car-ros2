from setuptools import find_packages, setup

package_name = "follow_me_nodes"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
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
        ],
    },
)
