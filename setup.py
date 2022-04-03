from setuptools import setup

package_name = 'pet_ros2_battery_state_pkg'

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
    maintainer='SeniorKullken',
    maintainer_email='stefan.kull@gmail.com',
    description='ROS2-publisher for the Current/Voltage(Battery State) sensor INA219. Publish measurement as ROS2-topics.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "pet_battery_state_ina219_node=pet_ros2_battery_state_pkg.pet_battery_state_ina219_node:main"
        ],
    },
)
