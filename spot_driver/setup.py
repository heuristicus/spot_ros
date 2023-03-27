from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["spot_driver"], scripts=["scripts/spot_ros", 'scripts/world_object_wrapper_test'], package_dir={"": "src"}
)

setup(**d)
