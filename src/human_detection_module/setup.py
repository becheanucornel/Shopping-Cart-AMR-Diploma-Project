from setuptools import find_packages, setup

package_name = 'human_detection_module'

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
    maintainer='becheanucornel',
    maintainer_email='becheanucornel28@gmail.com',
    description='Human detection package',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'human_detection_node = human_detection_module.human_detection_node:main',
        ],
    },
)
