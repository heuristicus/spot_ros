from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["spot_driver"],
    scripts=["scripts/spot_ros", "test/graph_nav_util_test.py", "test/ros_helpers_test.py"],
    package_dir={"": "src"}
)

setup(**d)
