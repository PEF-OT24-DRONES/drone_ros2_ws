from setuptools import setup

package_name = 'drone_control'

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
    zip_safe=True,
    maintainer='albertocastro',
    maintainer_email='josealberto.castro@udem.edu',
    description='Package description',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_control = drone_control.keyboard_control:main',
            'offboard_control = drone_control.offboard_control:main',
            'direct_offboard = drone_control.direct_offboard:main',
            'takeoff = drone_control.takeoff:main',
            'mavros_node = drone_control.mavros_node:main',
            'mavros_nogps = drone_control.mavros_nogps:main',
            'rc_control = drone_control.rc_control:main',
            'drone_state = drone_control.drone_state:main',
        ],
    },
)

