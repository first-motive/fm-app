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
    install_requires=[
        "setuptools",
        "textual==0.74.0",
        "rich",
        # Brand, widgets, and the pick menu — carved out into the shared wheel.
        # SHA-pinned git install (== tag v0.1.0), matching the sibling externals'
        # immutable pinning; pip resolves it, colcon does not, so the Dockerfile
        # installs it too. PyPI-ready: swap for a version spec once published.
        "fm-tools @ git+https://github.com/first-motive/fm-tools@5d9ef62f9449321730b8ebcacef7be3bc13448f5",
    ],
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
