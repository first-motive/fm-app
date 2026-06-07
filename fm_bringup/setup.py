from glob import glob

from setuptools import find_packages, setup

package_name = "fm_bringup"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        (
            "share/" + package_name + "/launch/sim_backends",
            glob("launch/sim_backends/*.launch.py"),
        ),
        (
            "share/" + package_name + "/config/openarm",
            glob("config/openarm/*.yaml") + glob("config/openarm/*.srdf"),
        ),
        (
            "share/" + package_name + "/config/so101",
            glob("config/so101/*.yaml") + glob("config/so101/*.srdf"),
        ),
        (
            "share/" + package_name + "/config/g1_d",
            glob("config/g1_d/*.yaml") + glob("config/g1_d/*.srdf"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="First Motive",
    maintainer_email="nish@ubundi.co.za",
    description="First Motive bringup: launch files and runtime configs",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bringup = fm_bringup.bringup:main",
            "joy_to_servo = fm_bringup.joy_to_servo:main",
            "spacenav_to_servo = fm_bringup.spacenav_to_servo:main",
        ],
    },
)
