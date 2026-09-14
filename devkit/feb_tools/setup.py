import os
from glob import glob
from setuptools import setup

package_name = "feb_tools"

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
    maintainer="Formula Electric at Berkeley",
    maintainer_email="pranavbhatttheonlyone@gmail.com",
    description="FEB devkit extras",
    license="BSD-2-Clause",
    entry_points={"console_scripts": ["sensor_noise = feb_tools.sensor_noise:main",
                                      "house_driver = feb_tools.house_driver:main"]},
)
