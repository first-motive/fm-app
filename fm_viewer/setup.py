from glob import glob

from setuptools import find_packages, setup

package_name = "fm_viewer"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # The web viewer panel (index.html) — a dependency-free page that talks to the
        # foxglove_bridge websocket. Installed so a launch/server can serve it from the
        # package share; a host path can open the source copy directly. Glob files
        # per-type (not "webgui/*") so the fonts/ subdir is not picked up as a data file.
        ("share/" + package_name + "/webgui",
         glob("webgui/*.html") + glob("webgui/*.js") + glob("webgui/*.svg")),
        # Bundled fonts for the panel (offline; no CDN).
        ("share/" + package_name + "/webgui/fonts", glob("webgui/fonts/*")),
        # Vendored 3D libs (three.js + loaders + urdf-loader) for the in-panel arm viewer.
        ("share/" + package_name + "/webgui/vendor", glob("webgui/vendor/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="First Motive",
    maintainer_email="nish@ubundi.co.za",
    description="First Motive web viewer panel (Foxglove WS): 3D robot, camera feed, plots, recordings",
    license="Apache-2.0",
    tests_require=["pytest"],
)
