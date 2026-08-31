from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mirte_workshop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/launch', glob('launch/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mirte',
    maintainer_email='m.wisse@tudelft.nl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "arm_server.py = mirte_workshop.arm_server:main",
            "arm_task_server.py = mirte_workshop.arm_task_server:main",
            "gripper_server.py = mirte_workshop.gripper_server:main",
            "mirte_keyboard.py = mirte_workshop.mirte_keyboard:main",
        ],
    },
)

# Note: the entry_points here have a .py extension. This is unusual.
# I have done this to make running a node consistent with nodes
# from ament_cmake packages.  
