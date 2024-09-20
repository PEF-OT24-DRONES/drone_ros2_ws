from setuptools import find_packages, setup

package_name = 'aruco_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'camera_node = aruco_detection.camera_node:main',
            'camera_receiver_node = aruco_detection.camera_receiver_node:main',
            'aruco_pose_detection_node = aruco_detection.aruco_pose_detection:main',
        ],
    },
)
