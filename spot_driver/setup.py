from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["spot_driver"],
    scripts=[
        "scripts/spot_ros",
        "test/ros_helpers_test.py",
        "test/spot_ros_test.py",
    ],
    package_dir={"": "src"},
)

setup(**d)
