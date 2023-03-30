from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["spot_driver"], 
    scripts=["scripts/spot_ros", 
             'scripts/examples/world_object_wrapper_test', 
             'scripts/examples/task_wrapper_example',
             'scripts/examples/hand_pose_example',
             ], 
    package_dir={"": "src"}
)

setup(**d)
