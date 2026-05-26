import os
from glob import glob
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
        # --- NEW: Tell colcon to copy the model folder ---
        (os.path.join('share', package_name, 'model'), glob('model/*.pt')),
        # If you have launch files, add them here too:
        # (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hephaestus',
    maintainer_email='your_email@example.com',
    description='YOLOv8 Detection Module',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection_node = human_detection_module.detection_node:main',
        ],
    },
)