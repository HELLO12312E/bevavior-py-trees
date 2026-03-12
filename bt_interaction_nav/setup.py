from setuptools import find_packages, setup

package_name = "bt_interaction_nav"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            ["launch/run.launch.py"],
        ),
        (
            "share/" + package_name + "/config",
            ["config/params.yaml"],
        ),
    ],
    install_requires=["setuptools", "py_trees"],
    zip_safe=True,
    maintainer="HELLO12312E",
    maintainer_email="you@example.com",
    description=(
        "Reactive py_trees behavior tree: approach person A once, "
        "wait 10 s, then navigate to goal B (Nav2)."
    ),
    license="MIT",
    entry_points={
        "console_scripts": [
            "bt_runner_node = bt_interaction_nav.bt_runner_node:main",
        ],
    },
)
