import os
from glob import glob

from setuptools import setup


package_name = "my_ros2_examples"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="y",
    maintainer_email="y@example.com",
    description="ROS 2 Humble Python examples: publisher, subscriber, service, parameters.",
    license="Apache License 2.0",
    tests_require=["pytest"],
)





