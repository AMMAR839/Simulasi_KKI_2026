import os

from setuptools import find_packages, setup


package_name = "asv_perception"


def package_files(directory):
    data_files = []
    for root, dirs, files in os.walk(directory):
        dirs[:] = [name for name in dirs if name != "__pycache__"]
        paths = [
            os.path.join(root, name)
            for name in files
            if not name.endswith((".pyc", ".pyo"))
        ]
        if paths:
            data_files.append((os.path.join("share", package_name, root), paths))
    return data_files


data_files = [
    (
        "share/ament_index/resource_index/packages",
        [os.path.join("resource", package_name)],
    ),
    (os.path.join("share", package_name), ["package.xml"]),
]
for directory in ("config", "launch"):
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
    description="HSV perception and persistent survey mapping for ASV KKI 2026.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "hsv_target_detector = asv_perception.hsv_target_detector:main",
            "semantic_landmark_mapper = asv_perception.semantic_landmark_mapper:main",
            "survey_mapper = asv_perception.survey_mapper:main",
            "preferred_lane_server = asv_perception.preferred_lane_server:main",
        ],
    },
)
