from setuptools import find_packages, setup

package_name = "autobee_tello"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/tello.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Daniel Oloruntoba",
    maintainer_email="danieloloruntoba681@gmail.com",
    description="Real-Tello driver bridging djitellopy to the AutoBee /drone/* topic graph.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "tello_driver_node = autobee_tello.tello_driver_node:main",
        ],
    },
)
