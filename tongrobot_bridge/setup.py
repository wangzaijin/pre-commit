from setuptools import setup, find_packages
import os
from glob import glob

package_name = "tongrobot_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools<=79.0.1"],
    zip_safe=True,
    author="TongRobot",
    description="ROS2 bridge node + gRPC server for TongRobot",
    license="MIT",
    entry_points={
        "console_scripts": [
            "bridge_node = tongrobot_bridge.bridge_node:main",
        ],
    },
)
