from setuptools import setup

package_name = "autobee_agent"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Daniel Oloruntoba",
    maintainer_email="danieloloruntoba681@gmail.com",
    description="LLM mission planner: natural-language intent -> mission JSON.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "llm_agent_node = autobee_agent.llm_agent_node:main",
            "report_node = autobee_agent.report_node:main",
        ],
    },
)
