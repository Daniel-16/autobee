from setuptools import find_packages, setup

package_name = "autobee_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Daniel Oloruntoba",
    maintainer_email="danieloloruntoba681@gmail.com",
    description="Open-loop patrol behaviours for the AutoBee drone.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "patrol_node = autobee_control.patrol_node:main",
            "tello_odometry_node = autobee_control.tello_odometry_node:main",
            "sim_odometry_node = autobee_control.sim_odometry_node:main",
            "flight_controller_node = autobee_control.flight_controller_node:main",
        ],
    },
)
