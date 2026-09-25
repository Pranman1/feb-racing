import os
from glob import glob
from setuptools import setup

package_name = "feb_cone_racer"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="FEB Autonomous",
    maintainer_email="pranavbhatttheonlyone@gmail.com",
    description="Cone-track racer: GraphSLAM map, raceline, nonlinear MPC",
    license="MIT",
    entry_points={"console_scripts": ["racer = feb_cone_racer.racer:main"]},
)
