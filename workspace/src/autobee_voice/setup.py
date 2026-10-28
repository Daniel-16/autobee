from setuptools import find_packages, setup

package_name = "autobee_voice"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/voice.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Daniel Oloruntoba",
    maintainer_email="danieloloruntoba681@gmail.com",
    description="Voice I/O for AutoBee: Whisper STT + gTTS TTS.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "whisper_stt_node = autobee_voice.whisper_stt_node:main",
            "tts_node = autobee_voice.tts_node:main",
        ],
    },
)
