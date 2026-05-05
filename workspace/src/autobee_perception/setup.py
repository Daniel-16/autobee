from setuptools import find_packages, setup

package_name = "autobee_perception"

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
    description="YOLOv8-based aerial perception emitting structured detections.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "yolov8_perception_node = autobee_perception.yolov8_perception_node:main",
            "crowd_analytics_node = autobee_perception.crowd_analytics_node:main",
            "anomaly_detection_node = autobee_perception.anomaly_detection_node:main",
            "looming_detector_node = autobee_perception.looming_detector_node:main",
        ],
    },
)
