from setuptools import find_packages, setup

package_name = 'robot_driver'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='becheanucornel',
    maintainer_email='becheanucornel28@gmail.com',
    description='Differential Driver AMR - Robot Driver',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motor_controller_node = robot_driver.motor_controller:main',
            'batery_monitor_node = robot_driver.battery_reader:main',
        ],
    },
)
