from setuptools import setup, find_packages
import os
from glob import glob

package_name = "asv_nav2"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*"),
        ),
        (
            os.path.join("share", package_name, "behavior_trees"),
            glob("behavior_trees/*.xml"),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ammar",
    maintainer_email="ammar@example.com",
    description="Nav2 full-stack for ASV KKI 2026",
    license="MIT",
    entry_points={
        "console_scripts": [],
    },
)
