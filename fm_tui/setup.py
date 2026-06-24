from setuptools import find_packages, setup

package_name = "fm_tui"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "textual==0.74.0", "rich"],
    zip_safe=True,
    maintainer="First Motive",
    maintainer_email="nish@ubundi.co.za",
    description="First Motive terminal UI: a colour-coded ROS2 monitor, themed by nish-tui when present.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fm_tui = fm_tui.app:main",
            "fm_tui_launcher = fm_tui.launcher:main",
        ],
    },
)
