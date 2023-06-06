from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["interactive_marker_utils"],
    package_dir={"": "scripts"}
)

setup(**d)
