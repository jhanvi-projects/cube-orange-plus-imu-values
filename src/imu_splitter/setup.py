from setuptools import setup

package_name = 'imu_splitter'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Separate MAVLink IMU streams into ROS 2 IMU topics',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'imu_splitter_node = imu_splitter.imu_splitter_node:main',
        ],
    },
)
