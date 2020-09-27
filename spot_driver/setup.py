
from setuptools import setup

package_name = 'spot_driver'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Dave Niewinski',
    author_email='dniewinski@clearpathrobotics.com',
    maintainer='Dave Niewinski',
    maintainer_email='dniewinski@clearpathrobotics.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: BSD',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='The spot_driver package',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver = spot_driver.spot_ros.py:main',
        ],
    },
)