from setuptools import setup

package_name = 'spot_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Jeroen Fransman',
    maintainer_email='jeroen.fransman@tno.nl',
    description='The Spot driver supplies a ROS2 interface to the Spot SDK',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spot_driver = spot_driver.spot_driver:main'
        ],
    },
)