from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'charging_station'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('charging_station/map.qml')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='albertocastro',
    maintainer_email='josealberto.castro@udem.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_motor_publisher = charging_station.serial_motor_publisher:main',
            'serial_motor_subscriber = charging_station.serial_motor_subscriber:main',
            'charging_station_interface = charging_station.charging_station_interface:main',
        ],
    },
)
