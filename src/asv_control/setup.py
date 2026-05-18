import os
from setuptools import find_packages, setup

package_name = "asv_control"


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
data_files.extend(package_files("config"))

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ammar",
    maintainer_email="ammar@example.com",
    description="Manual joystick, command mux, and thruster command nodes for ASV simulation.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "cmd_vel_to_thrusters = asv_control.cmd_vel_to_thrusters:main",
            "manual_autonomy_mux = asv_control.manual_autonomy_mux:main",
            "joystick_mode_manager = asv_control.joystick_mode_manager:main",
            "simple_joystick_teleop = asv_control.simple_joystick_teleop:main",
            "planar_pose_controller = asv_control.planar_pose_controller:main",
        ],
    },
)
