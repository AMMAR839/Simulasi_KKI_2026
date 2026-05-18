import os
from setuptools import find_packages, setup

package_name = "asv_navigation"


def package_files(directory):
    data_files = []
    if not os.path.isdir(directory):
        return data_files
    for root, dirs, files in os.walk(directory):
        dirs[:] = [dirname for dirname in dirs if dirname != "__pycache__"]
        paths = [
            os.path.join(root, filename)
            for filename in files
            if not filename.endswith((".pyc", ".pyo"))
        ]
        if paths:
            data_files.append((os.path.join("share", package_name, root), paths))
    return data_files


data_files = [
    ("share/ament_index/resource_index/packages", [os.path.join("resource", package_name)]),
    (os.path.join("share", package_name), ["package.xml"]),
]
for directory in ["config", "launch"]:
    data_files.extend(package_files(directory))

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ammar",
    maintainer_email="ammar@example.com",
    description="GPS waypoint follower and navigation configuration for ASV simulation.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "gps_waypoint_follower = asv_navigation.gps_waypoint_follower:main",
            "mission_supervisor = asv_navigation.mission_supervisor:main",
        ],
    },
)
