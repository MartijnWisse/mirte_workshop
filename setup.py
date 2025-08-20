from setuptools import find_packages, setup

package_name = 'mirte_workshop'

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
    maintainer='mirte',
    maintainer_email='m.wisse@tudelft.nl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_arm_simple_script = mirte_workshop.test_arm_simple_script:main",
            "arm_server = mirte_workshop.arm_server:main",
            "arm_task_server = mirte_workshop.arm_task_server:main",
            "gripper_server = mirte_workshop.gripper_server:main",
        ],
    },
)
