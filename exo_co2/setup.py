from setuptools import find_packages, setup

package_name = 'exo_co2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='labrnth',
    maintainer_email='lbooth@ucmerrced.edu',
    description='A ROS2 driver for the AtlasScientific EXO-CO2 sensor (via atlas_i2c library)',
    license='LGPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['exo_co2_node = exo_co2.exo_co2:main'
        ],
    },
)