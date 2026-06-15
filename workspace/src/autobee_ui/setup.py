import os
from glob import glob
from setuptools import setup

package_name = "autobee_ui"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Daniel Oloruntoba",
    maintainer_email="danieloloruntoba681@gmail.com",
    description="Web dashboard bridge: ROS topic graph <-> HTTP/SSE API for the Next.js control panel.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "web_ui_node = autobee_ui.web_ui_node:main",
        ],
    },
)
