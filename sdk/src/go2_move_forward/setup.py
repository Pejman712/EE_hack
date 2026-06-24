from setuptools import setup

package_name = 'go2_move_forward'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='todo@example.com',
    description='Simple ROS2 node to move Go2 forward one meter',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_forward = go2_move_forward.move_forward_node:main',
        ],
    },
)
