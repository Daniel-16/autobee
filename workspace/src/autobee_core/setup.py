from setuptools import find_packages, setup

package_name = "autobee_core"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/demo.launch.py", "launch/demo_tello.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Daniel Oloruntoba",
    maintainer_email="danieloloruntoba681@gmail.com",
    description="Mission executor and waypoint generators for AutoBee.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mission_executor_node = autobee_core.mission_executor_node:main",
            "mission_logger_node = autobee_core.mission_logger_node:main",
        ],
    },
)
