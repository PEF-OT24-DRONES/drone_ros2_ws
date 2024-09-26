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
    install_requires=['setuptools', 'rclpy', 'sensor_msgs', 'cv_bridge', 'image_transport', 'opencv-python', 'numpy'],
    zip_safe=True,
    maintainer='albertocastro',
    maintainer_email='josealberto.castro@udem.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = aruco_detection.camera_node:main',
            'camera_receiver = aruco_detection.camera_receiver:main',
            'aruco_pose_detection = aruco_detection.aruco_pose_detection:main'
        ],
    },
)
